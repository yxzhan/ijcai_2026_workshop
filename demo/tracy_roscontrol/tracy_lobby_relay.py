#!/usr/bin/env python3
"""Between the lobby's Tracy and its ros2_control: the lobby takes position
targets (sensor_msgs/JointState on /robot_joint_commands), the arms' forward
velocity controllers give velocities (topic_based_ros2_control publishes them
on /<arm>/joint_velocity_commands). This integrates them into position targets
(from where the arm is when commands start), and republishes the grippers'
joints of the lobby's states (/robot_joint_states) on /<side>_gripper/joint_states,
where giskardpy reads the real Tracy's Robotiq grippers.
"""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARMS = ('left_arm', 'right_arm')
GRIPPERS = {'left_gripper': 'left_robotiq_85_', 'right_gripper': 'right_robotiq_85_'}
RESTART_AFTER = 0.2  # s without commands: start again from the measured position
MAX_STEP = 0.05      # s, the longest step integrated at once


class Relay(Node):
  def __init__(self):
    super().__init__('tracy_lobby_relay')
    self.measured = {}
    self.target = {}
    self.last = {arm: 0.0 for arm in ARMS}
    self.commands = self.create_publisher(JointState, '/robot_joint_commands', 10)
    self.grippers = {g: self.create_publisher(JointState, f'/{g}/joint_states', 10) for g in GRIPPERS}
    self.create_subscription(JointState, '/robot_joint_states', self.on_state, 10)
    for arm in ARMS:
      self.create_subscription(JointState, f'/{arm}/joint_velocity_commands', lambda m, a=arm: self.on_velocity(a, m), 10)

  def on_state(self, msg):
    self.measured.update(zip(msg.name, msg.position))
    for gripper, prefix in GRIPPERS.items():
      i = [k for k, n in enumerate(msg.name) if n.startswith(prefix)]
      if not i:
        continue
      out = JointState(header=msg.header, name=[msg.name[k] for k in i], position=[msg.position[k] for k in i])
      if len(msg.velocity) == len(msg.name):
        out.velocity = [msg.velocity[k] for k in i]
      self.grippers[gripper].publish(out)

  def on_velocity(self, arm, msg):
    now = time.monotonic()
    dt, self.last[arm] = now - self.last[arm], now
    restart = dt > RESTART_AFTER
    dt = min(dt, MAX_STEP)
    names, positions = [], []
    for name, v in zip(msg.name, msg.velocity):
      if name not in self.measured:
        continue
      if restart or name not in self.target:
        self.target[name] = self.measured[name]
      else:
        self.target[name] += v * dt
      names.append(name)
      positions.append(self.target[name])
    if names:
      out = JointState(name=names, position=positions)
      out.header.stamp = self.get_clock().now().to_msg()
      self.commands.publish(out)


def main():
  rclpy.init()
  node = Relay()
  try:
    rclpy.spin(node)
  except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
    pass


if __name__ == '__main__':
  main()
