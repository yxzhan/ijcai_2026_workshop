"""The lobby's FR3 with its hand under ros2_control, for giskardpy: the URDF of
../fr3_roscontrol (robot_state_publisher, /robot_description) with the arm's
joints velocity commanded and FR3's velocity limits, the controller manager
with topic_based_ros2_control as the hardware (the arm's velocity commands
on /fr3/joint_velocity_commands, which ../lobby_velocity_relay.py turns into
the lobby's position targets, its states those targets; the hand's on the
lobby's /robot_joint_commands and /robot_joint_states),
the joint state broadcaster, forward_velocity_controller and fr3_gripper.

    source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
    ros2 launch demo/fr3_giskard/fr3_giskard.launch.py

In the lobby: spawn Franka FR3 with hand, untick its IK (Teleop panel), switch
its ROS on (its profile sets the topics).
"""

import pathlib
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

HERE = pathlib.Path(__file__).resolve().parent
ARM = [f'fr3_joint{i}' for i in range(1, 8)]
HAND = 'fr3_finger_joint1'
VELOCITY_LIMITS = [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26]  # rad/s, the FR3 datasheet's
ARM_COMMANDS = '/fr3/joint_velocity_commands'
# The arm's states as ros2_control reads them: the relay's targets (see lobby_velocity_relay.py --state-out).
ARM_STATES = '/fr3/joint_states_for_control'


def hardware(robot, name, joints, command_interface, states_topic, commands_topic):
  """A topic_based_ros2_control block for those joints."""
  block = ET.SubElement(robot, 'ros2_control', name=name, type='system')
  h = ET.SubElement(block, 'hardware')
  ET.SubElement(h, 'plugin').text = 'topic_based_ros2_control/TopicBasedSystem'
  for key, value in (('joint_states_topic', states_topic), ('joint_commands_topic', commands_topic),
                     ('trigger_joint_command_threshold', '-1')):  # always publish: it compares positions only
    ET.SubElement(h, 'param', name=key).text = value
  for joint in joints:
    j = ET.SubElement(block, 'joint', name=joint)
    ET.SubElement(j, 'command_interface', name=command_interface)
    ET.SubElement(j, 'state_interface', name='position')
    ET.SubElement(j, 'state_interface', name='velocity')


def fr3_urdf():
  robot = ET.parse(HERE.parent / 'fr3_roscontrol' / 'fr3.urdf').getroot()
  for block in robot.findall('ros2_control'):
    robot.remove(block)
  for joint, limit in zip(ARM, VELOCITY_LIMITS):
    robot.find(f"joint[@name='{joint}']/limit").set('velocity', str(limit))
  hardware(robot, 'lobby_arm', ARM, 'velocity', ARM_STATES, ARM_COMMANDS)
  hardware(robot, 'lobby_hand', [HAND], 'position', '/robot_joint_states', '/robot_joint_commands')
  return ET.tostring(robot, encoding='unicode')


def generate_launch_description():
  description = {'robot_description': fr3_urdf()}
  spawner = lambda name: Node(package='controller_manager', executable='spawner',
                              arguments=[name, '--controller-manager-timeout', '60'], output='screen')
  return LaunchDescription([
      Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[description], output='screen'),
      Node(package='controller_manager', executable='ros2_control_node',
           parameters=[description, str(HERE / 'controllers.yaml')], output='screen'),
      spawner('joint_state_broadcaster'),
      spawner('forward_velocity_controller'),
      spawner('fr3_gripper'),
      ExecuteProcess(cmd=['python3', str(HERE.parent / 'lobby_velocity_relay.py'), '--velocity', ARM_COMMANDS,
                          '--state-out', ARM_STATES],
                     output='screen'),
  ])
