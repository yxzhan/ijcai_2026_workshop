"""Moves a Franka Emika Panda with giskardpy, its hand to positions given
relative to the robot's base (link0: x forward, y left, z up, in m), the
hand pointing down as in Franka's ready pose: from Franka's ready pose to a
centre, then a small step up, down, forward, back, left and right of it
(back to the centre after each), the hand closed and opened, the ready pose
again. The same program for the real Panda, a fake one and the lobby's:
panda_giskard.launch.py's driver picks which.

    panda_giskard_demo.py                                # once, around 0.45 0 0.45, 5 cm steps
    panda_giskard_demo.py --center 0.4 0.1 0.4 --step 0.03
    panda_giskard_demo.py --cycles 0                     # the points again and again, until Ctrl-C

The position is the `hand` frame's (the hand's base on the flange; the
fingertips are 0.10 m further along its z, here below it). Every point has to
lie in a safe box (SAFE: x 0.25..0.65, |y| <= 0.35, z 0.25..0.65 in link0,
the fingertips above the base's plane, the table), the step at most 0.10 m.

The gripper is robot-lobby's real_robot.ros2 action (not giskard's: the hand
is not velocity commanded).

Needs panda_giskard.launch.py and giskard (universal_robot_velocity)
running, the robot enabled: run_panda_giskard_demo.sh does it all and
passes its arguments on:

    DRIVER=fake demo/panda_giskard/run_panda_giskard_demo.sh --cycles 3
"""

import argparse
import math
import sys
import time

import numpy as np
from control_msgs.action import ParallelGripperCommand
from cramph.statechart import Statechart
from giskardpy.middleware.ros2.giskard_client_executor import GiskardClientExecutor
from giskardpy.motion_statechart.graph_node import EndMotion
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPose
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList, JointState
from krrood_spatial_math.spatial_types import Pose
from rclpy.action import ActionClient
from semantic_digital_twin.adapters.ros import rospy

ARM = [f'joint{i}' for i in range(1, 8)]
READY = [0.0, -math.pi / 4, 0.0, -3 * math.pi / 4, 0.0, math.pi / 2, math.pi / 4]
CENTER = (0.45, 0.0, 0.45)  # m, in link0
STEP = 0.05                 # m
MAX_STEP = 0.10             # m
SAFE = ((0.25, 0.65), (-0.35, 0.35), (0.25, 0.65))  # x, y, z, m, in link0
DIRECTIONS = [('up', (0, 0, 1)), ('down', (0, 0, -1)), ('forward', (1, 0, 0)),
              ('back', (-1, 0, 0)), ('left', (0, 1, 0)), ('right', (0, -1, 0))]
GRIPPER = '/panda/hw/gripper_cmd'


def quaternion(rotation):
  """x, y, z, w of a rotation matrix."""
  m = rotation
  w = math.sqrt(max(0.0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
  x = math.copysign(math.sqrt(max(0.0, 1 + m[0, 0] - m[1, 1] - m[2, 2])) / 2, m[2, 1] - m[1, 2])
  y = math.copysign(math.sqrt(max(0.0, 1 - m[0, 0] + m[1, 1] - m[2, 2])) / 2, m[0, 2] - m[2, 0])
  z = math.copysign(math.sqrt(max(0.0, 1 - m[0, 0] - m[1, 1] + m[2, 2])) / 2, m[1, 0] - m[0, 1])
  return x, y, z, w


def points(center, step):
  """[(name, xyz)]: the centre, then each direction's point and the centre again."""
  c = np.array(center, dtype=float)
  out = [('center', c)]
  for name, d in DIRECTIONS:
    out += [(name, c + step * np.array(d)), ('center', c)]
  return out


def unsafe(xyz):
  return [f'{axis} {v:.3f} not in {lo}..{hi}'
          for axis, v, (lo, hi) in zip('xyz', xyz, SAFE) if not lo <= v <= hi]


class Demo:
  def __init__(self):
    rospy.init_node('panda_giskard_demo')
    self.giskard = GiskardClientExecutor(node_handle=rospy.get_node())
    self.world = self.giskard.world
    self.base = self.world.get_body_by_name('link0')
    self.hand = self.world.get_body_by_name('hand')
    self.gripper = ActionClient(rospy.get_node(), ParallelGripperCommand, GRIPPER)
    self.down = None  # the hand's orientation in the ready pose (x, y, z, w)

  def run(self, goal):
    statechart = Statechart(context=self.giskard.context)
    statechart.add_node(goal)
    statechart.add_node(EndMotion.when_true(goal))
    t0 = time.monotonic()
    self.giskard.compile(statechart)
    self.giskard.execute()
    return time.monotonic() - t0

  def joints(self):
    return [self.world.get_connection_by_name(j).position for j in ARM]

  def hand_pose(self):
    return self.world.compute_forward_kinematics_np(self.base, self.hand)

  def ready(self, what='ready pose'):
    t = self.run(JointPositionList(goal_state=JointState.from_str_dict(dict(zip(ARM, READY)), self.world)))
    err = max(abs(a - b) for a, b in zip(self.joints(), READY))
    print(f'{what}: done in {t:.1f} s, joints off by {err:.4f} rad at most', flush=True)
    if self.down is None:
      self.down = quaternion(self.hand_pose()[:3, :3])

  def move_hand(self, xyz, what):
    """The hand to xyz in link0, pointing down."""
    target = np.array(xyz, dtype=float)
    goal = Pose.from_xyz_quaternion(*target, *self.down, reference_frame=self.base)
    t = self.run(CartesianPose(root_link=self.base, tip_link=self.hand, goal_pose=goal))
    end = self.hand_pose()[:3, 3]
    print(f'{what}: done in {t:.1f} s, hand at {np.round(end, 3).tolist()} (goal {np.round(target, 3).tolist()}), '
          f'off by {np.linalg.norm(end - target) * 1000:.1f} mm', flush=True)

  def grip(self, width, what):
    if not self.gripper.wait_for_server(timeout_sec=30.0):
      sys.exit(f'no {GRIPPER} (is panda_giskard.launch.py running?)')
    goal = ParallelGripperCommand.Goal()
    goal.command.name = ['finger_joint1']
    goal.command.position = [float(width)]
    handle = self.wait(self.gripper.send_goal_async(goal), 30.0, what)
    result = self.wait(handle.get_result_async(), 30.0, what).result
    finger = result.state.position[0] if result.state.position else float('nan')
    print(f'{what}: reached {result.reached_goal}, finger at {finger:.4f} m', flush=True)

  @staticmethod
  def wait(future, timeout, what):
    t0 = time.monotonic()
    while not future.done():  # the shared node spins in rospy's thread
      if time.monotonic() - t0 > timeout:
        sys.exit(f'{what}: no answer within {timeout} s')
      time.sleep(0.05)
    return future.result()


def main():
  parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0], formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument('--center', nargs=3, type=float, metavar=('X', 'Y', 'Z'), default=CENTER,
                      help=f'the hand\'s centre position in link0, m (default {CENTER})')
  parser.add_argument('--step', type=float, default=STEP, help=f'm from the centre (default {STEP}, at most {MAX_STEP})')
  parser.add_argument('--cycles', type=int, default=1, help='times through the points (0: until Ctrl-C)')
  parser.add_argument('--check', action='store_true', help='only check the arguments (no ROS)')
  args = parser.parse_args()
  if not 0 < args.step <= MAX_STEP:
    parser.error(f'--step: {args.step} not in 0..{MAX_STEP}')
  path = points(args.center, args.step)
  for name, p in path:
    if problems := unsafe(p):
      parser.error(f'{name} point {np.round(p, 3).tolist()} outside the safe box: {", ".join(problems)}')
  if args.check:
    return

  demo = Demo()
  try:
    demo.ready()
    cycle = 0
    try:
      while not args.cycles or cycle < args.cycles:
        cycle += 1
        for name, p in path[1:] if cycle > 1 else path:  # a cycle ends on the centre
          demo.move_hand(p, f'#{cycle} {name}')
    except KeyboardInterrupt:
      print('stopped', flush=True)
      try:  # the goal running when Ctrl-C came: giskard stops it
        demo.giskard.cancel_goal_async()
      except Exception:
        pass
      return
    demo.grip(0.0, 'close the hand')
    demo.grip(0.04, 'open the hand')
    demo.ready('ready pose again')
  finally:
    rospy.shutdown()


if __name__ == '__main__':
  main()
