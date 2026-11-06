"""A Franka Emika Panda (FER) under ros2_control, for giskardpy: panda.urdf
(robot_state_publisher, /robot_description) with the arm's joints velocity
commanded and the Panda's velocity limits (times `speed`), the controller
manager with topic_based_ros2_control as the hardware on /panda/hw/...,
the joint state broadcaster and forward_velocity_controller; and behind
those topics robot-lobby's real_robot ROS 2 front end (python3 -m
real_robot.ros2), which drives

    driver:=real    the real Panda (franky on libfranka 0.9.x, Panda System 4.2.x)
    driver:=fake    a simulated stand-in (no hardware, no lobby)
    driver:=lobby   the lobby's Panda over rosbridge (a digital twin)

The gripper is the front end's action, /panda/hw/gripper_cmd
(control_msgs/ParallelGripperCommand on finger_joint1), not a controller.
The robot is disabled until /panda/hw/enable (std_srvs/SetBool) is called:
run_panda_giskard_demo.sh does.

    source /opt/ros/jazzy/setup.bash
    ros2 launch demo/panda_giskard/panda_giskard.launch.py driver:=fake
    ros2 launch demo/panda_giskard/panda_giskard.launch.py driver:=real ip:=172.16.0.2 speed:=0.2

lobby_dir: the robot-lobby checkout with real_robot/ (default $LOBBY_DIR,
else the directory this repository sits in).

panda.urdf: made by ../fr3_roscontrol/mjcf2urdf.py from the lobby's
mujoco_menagerie/franka_emika_panda/panda.xml (joint1..7, finger_joint1;
link0..link7, hand): the kinematic tree only, no collision geometry.
"""

import os
import pathlib
import sys
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = pathlib.Path(__file__).resolve().parent
ARM = [f'joint{i}' for i in range(1, 8)]
HAND = 'finger_joint1'
VELOCITY_LIMITS = [2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61]  # rad/s, the Panda datasheet's
NS = '/panda/hw'
LOBBY_DIR = os.environ.get('LOBBY_DIR') or str(HERE.parents[2])


def panda_urdf(speed):
  robot = ET.parse(HERE / 'panda.urdf').getroot()
  for block in robot.findall('ros2_control'):
    robot.remove(block)
  for joint, limit in zip(ARM, VELOCITY_LIMITS):
    robot.find(f"joint[@name='{joint}']/limit").set('velocity', f'{limit * speed:.4f}')
  block = ET.SubElement(robot, 'ros2_control', name='panda_hw', type='system')
  h = ET.SubElement(block, 'hardware')
  ET.SubElement(h, 'plugin').text = 'topic_based_ros2_control/TopicBasedSystem'
  for key, value in (('joint_states_topic', f'{NS}/joint_states'), ('joint_commands_topic', f'{NS}/joint_commands'),
                     ('trigger_joint_command_threshold', '-1')):  # always publish: the front end's timeout counts on it
    ET.SubElement(h, 'param', name=key).text = value
  for joint in ARM:
    j = ET.SubElement(block, 'joint', name=joint)
    ET.SubElement(j, 'command_interface', name='velocity')
    ET.SubElement(j, 'state_interface', name='position')
    ET.SubElement(j, 'state_interface', name='velocity')
  j = ET.SubElement(block, 'joint', name=HAND)  # read only: the gripper is the front end's action
  ET.SubElement(j, 'state_interface', name='position')
  return ET.tostring(robot, encoding='unicode')


def nodes(context):
  arg = lambda name: LaunchConfiguration(name).perform(context)
  speed = float(arg('speed'))
  description = {'robot_description': panda_urdf(speed)}
  spawner = lambda name: Node(package='controller_manager', executable='spawner',
                              arguments=[name, '--controller-manager-timeout', '60'], output='screen')
  front = [sys.executable, '-m', 'real_robot.ros2', '--robot', 'panda', '--driver', arg('driver'), '--ns', NS,
           '--speed', str(speed), '--ip', arg('ip'), '--dynamics', arg('dynamics')]
  return [
      Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[description], output='screen'),
      Node(package='controller_manager', executable='ros2_control_node',
           parameters=[description, str(HERE / 'controllers.yaml')], output='screen'),
      spawner('joint_state_broadcaster'),
      spawner('forward_velocity_controller'),
      ExecuteProcess(cmd=front, cwd=arg('lobby_dir'), output='screen'),
  ]


def generate_launch_description():
  return LaunchDescription([
      DeclareLaunchArgument('driver', default_value='fake', description='real | fake | lobby'),
      DeclareLaunchArgument('ip', default_value='172.16.0.2', description='real: the Panda\'s FCI address'),
      DeclareLaunchArgument('speed', default_value='0.5', description='of the Panda\'s joint velocity limits (giskard and the front end)'),
      DeclareLaunchArgument('dynamics', default_value='0.15', description='real: franky\'s relative_dynamics_factor'),
      DeclareLaunchArgument('lobby_dir', default_value=LOBBY_DIR, description='the robot-lobby checkout with real_robot/'),
      OpaqueFunction(function=nodes),
  ])
