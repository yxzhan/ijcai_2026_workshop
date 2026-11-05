"""Moves the lobby's FR3 with giskardpy, as a program for the real robot would:
a joint goal (Franka's ready pose), a Cartesian goal for the hand (15 cm
forward and 10 cm down in fr3_link0, its orientation kept), then the hand
closed and opened with its gripper action (not giskard's: the hand is not
velocity commanded), and the ready pose again. Prints where each goal ended.

Needs fr3_giskard.launch.py and giskard (universal_robot_velocity) running:
run_fr3_giskard_demo.sh starts them all.

    ~/.local/bin/cram_python_wrapper.sh demo/fr3_giskard/fr3_giskard_demo.py
"""

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

ARM = [f'fr3_joint{i}' for i in range(1, 8)]
READY = [0.0, -math.pi / 4, 0.0, -3 * math.pi / 4, 0.0, math.pi / 2, math.pi / 4]
OFFSET = (0.15, 0.0, -0.10)  # m, the hand's Cartesian goal from where it is, in fr3_link0


def quaternion(rotation):
  """x, y, z, w of a rotation matrix."""
  m = rotation
  w = math.sqrt(max(0.0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
  x = math.copysign(math.sqrt(max(0.0, 1 + m[0, 0] - m[1, 1] - m[2, 2])) / 2, m[2, 1] - m[1, 2])
  y = math.copysign(math.sqrt(max(0.0, 1 - m[0, 0] + m[1, 1] - m[2, 2])) / 2, m[0, 2] - m[2, 0])
  z = math.copysign(math.sqrt(max(0.0, 1 - m[0, 0] - m[1, 1] + m[2, 2])) / 2, m[1, 0] - m[0, 1])
  return x, y, z, w


class Demo:
  def __init__(self):
    rospy.init_node('fr3_giskard_demo')
    self.giskard = GiskardClientExecutor(node_handle=rospy.get_node())
    self.world = self.giskard.world
    self.base = self.world.get_body_by_name('fr3_link0')
    self.hand = self.world.get_body_by_name('fr3_hand')
    self.gripper = ActionClient(rospy.get_node(), ParallelGripperCommand, '/fr3_gripper/gripper_cmd')

  def run(self, goal, what):
    statechart = Statechart(context=self.giskard.context)
    statechart.add_node(goal)
    statechart.add_node(EndMotion.when_true(goal))
    t0 = time.monotonic()
    self.giskard.compile(statechart)
    self.giskard.execute()
    print(f'{what}: done in {time.monotonic() - t0:.1f} s', flush=True)

  def joints(self):
    return [self.world.get_connection_by_name(j).position for j in ARM]

  def hand_pose(self):
    return self.world.compute_forward_kinematics_np(self.base, self.hand)

  def move_joints(self, positions, what):
    self.run(JointPositionList(goal_state=JointState.from_str_dict(dict(zip(ARM, positions)), self.world)), what)
    err = max(abs(a - b) for a, b in zip(self.joints(), positions))
    print(f'  joints off by {err:.4f} rad at most', flush=True)

  def move_hand(self, offset, what):
    start = self.hand_pose()
    target = start[:3, 3] + np.array(offset)
    qx, qy, qz, qw = quaternion(start[:3, :3])
    goal = Pose.from_xyz_quaternion(*target, qx, qy, qz, qw, reference_frame=self.base)
    self.run(CartesianPose(root_link=self.base, tip_link=self.hand, goal_pose=goal), what)
    end = self.hand_pose()
    print(f'  hand at {np.round(end[:3, 3], 3).tolist()} (goal {np.round(target, 3).tolist()}), '
          f'off by {np.linalg.norm(end[:3, 3] - target) * 1000:.1f} mm', flush=True)

  def grip(self, width, what):
    if not self.gripper.wait_for_server(timeout_sec=30.0):
      sys.exit('no /fr3_gripper/gripper_cmd (is fr3_giskard.launch.py running?)')
    goal = ParallelGripperCommand.Goal()
    goal.command.name = ['fr3_finger_joint1']
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
  demo = Demo()
  demo.move_joints(READY, 'ready pose')
  demo.move_hand(OFFSET, 'hand forward and down')
  demo.grip(0.0, 'close the hand')
  demo.grip(0.04, 'open the hand')
  demo.move_joints(READY, 'ready pose again')
  rospy.shutdown()


if __name__ == '__main__':
  main()
