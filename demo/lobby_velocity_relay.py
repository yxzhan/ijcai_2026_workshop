#!/usr/bin/env python3
"""Velocity commands for a lobby robot, which takes position targets only.

topic_based_ros2_control, with velocity command interfaces, publishes the
velocities its controllers command (sensor_msgs/JointState, velocity filled) on
each --velocity topic; this integrates them into position targets on --commands
(the lobby's joint command topic).

With --state-out, it also publishes the joint states ros2_control should read:
the lobby's (--states), but for the velocity-commanded joints the integrated
targets instead of the lobby's measurements. A controller such as giskard then
closes its loop on those, not through the network to the browser and back,
whose delay makes it overshoot and oscillate; the lobby follows the targets.
Once a group's commands have been zero for --resync-after s and a joint is
more than --resync-beyond from its target (the arm moved in the lobby, not just
sagging under its controller's steady error), its targets are set to the
lobby's measurements again.

With --republish TOPIC=PREFIX, it republishes the lobby's joints whose names
start with PREFIX on TOPIC (where a program reads e.g. a gripper the real robot
publishes on a topic of its own).

    python3 demo/lobby_velocity_relay.py --velocity /fr3/joint_velocity_commands \\
        --state-out /fr3/joint_states_for_control
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Relay(Node):
  def __init__(self, args):
    super().__init__('lobby_velocity_relay')
    self.args = args
    self.measured = {}
    self.target = {}
    self.velocity = {}
    self.last = {}
    self.moving = {}
    self.commands = self.create_publisher(JointState, args.commands, 10)
    self.state_out = self.create_publisher(JointState, args.state_out, 10) if args.state_out else None
    self.republish = []
    for spec in args.republish:
      topic, _, prefix = spec.partition('=')
      self.republish.append((prefix, self.create_publisher(JointState, topic, 10)))
    self.create_subscription(JointState, args.states, self.on_state, 10)
    for topic in args.velocity:
      self.last[topic] = 0.0
      self.moving[topic] = 0.0
      self.create_subscription(JointState, topic, lambda m, t=topic: self.on_velocity(t, m), 10)

  def on_state(self, msg):
    self.measured.update(zip(msg.name, msg.position))
    for prefix, publisher in self.republish:
      i = [k for k, n in enumerate(msg.name) if n.startswith(prefix)]
      if not i:
        continue
      out = JointState(header=msg.header, name=[msg.name[k] for k in i], position=[msg.position[k] for k in i])
      if len(msg.velocity) == len(msg.name):
        out.velocity = [msg.velocity[k] for k in i]
      publisher.publish(out)
    self.publish_state()

  def on_velocity(self, topic, msg):
    now = time.monotonic()
    dt, self.last[topic] = now - self.last[topic], now
    if any(abs(v) > self.args.still for v in msg.velocity):
      self.moving[topic] = now
    restart = dt > self.args.restart_after
    resync = now - self.moving[topic] > self.args.resync_after and any(
        abs(self.measured[n] - self.target[n]) > self.args.resync_beyond
        for n in msg.name if n in self.measured and n in self.target)
    dt = min(dt, self.args.max_step)
    names, positions = [], []
    for name, v in zip(msg.name, msg.velocity):
      if name not in self.measured:
        continue
      if restart or resync or name not in self.target:
        self.target[name] = self.measured[name]
      else:
        self.target[name] += v * dt
      self.velocity[name] = v
      names.append(name)
      positions.append(self.target[name])
    if names:
      out = JointState(name=names, position=positions)
      out.header.stamp = self.get_clock().now().to_msg()
      self.commands.publish(out)
      self.publish_state()

  def publish_state(self):
    if self.state_out is None or not self.measured:
      return
    names = list(self.measured)
    out = JointState(name=names,
                     position=[self.target.get(n, self.measured[n]) for n in names],
                     velocity=[self.velocity.get(n, 0.0) for n in names])
    out.header.stamp = self.get_clock().now().to_msg()
    self.state_out.publish(out)


def main():
  p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
  p.add_argument('--velocity', action='append', default=[], metavar='TOPIC',
                 help='topic_based_ros2_control\'s joint commands topic of velocity-commanded joints (repeatable)')
  p.add_argument('--states', default='/robot_joint_states', help='the lobby\'s joint states')
  p.add_argument('--commands', default='/robot_joint_commands', help='the lobby\'s joint commands (position targets)')
  p.add_argument('--state-out', metavar='TOPIC',
                 help='publish the joint states for ros2_control here: the targets for the velocity-commanded joints')
  p.add_argument('--republish', action='append', default=[], metavar='TOPIC=PREFIX',
                 help='republish the lobby\'s joints starting with PREFIX on TOPIC (repeatable)')
  p.add_argument('--restart-after', type=float, default=0.2,
                 help='s without commands after which integration starts again from the measured positions')
  p.add_argument('--resync-after', type=float, default=1.0,
                 help='s of zero commands after which the targets are the measured positions again')
  p.add_argument('--resync-beyond', type=float, default=0.1,
                 help='rad (or m) a joint has to be off its target for the targets to be resynced')
  p.add_argument('--still', type=float, default=1e-4, help='rad/s (or m/s) below which a command counts as zero')
  p.add_argument('--max-step', type=float, default=0.05, help='s, the longest step integrated at once')
  args = p.parse_args(rclpy.utilities.remove_ros_args()[1:])
  rclpy.init()
  node = Relay(args)
  try:
    rclpy.spin(node)
  except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
    pass


if __name__ == '__main__':
  main()
