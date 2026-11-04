"""The lobby's Tracy under ros2_control, as the real one runs: its URDF from
iai_tracy (iai_tracy_description's tracy.urdf.xacro, in ros2_ws) on
/robot_description with robot_state_publisher (TF from the lobby's joint
states), a controller manager per arm (/left_arm/controller_manager,
/right_arm/controller_manager) with topic_based_ros2_control as the hardware,
each with a joint state broadcaster and a forward velocity controller, and
tracy_lobby_relay.py (the velocities as the lobby's position targets, the
grippers' joint states). giskardpy's tracy_velocity drives that.

    source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
    ros2 launch demo/tracy_roscontrol/tracy_lobby.launch.py

In the lobby: spawn Tracy, untick its IK (Teleop panel), switch its ROS on
(its profile sets the topics).
"""

import pathlib
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

HERE = pathlib.Path(__file__).resolve().parent
ARM_JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


def tracy_urdf():
  """The xacro'd iai_tracy URDF without its own ros2_control blocks (the Robotiq grippers')."""
  xacro = pathlib.Path(get_package_share_directory('iai_tracy_description')) / 'urdf' / 'tracy.urdf.xacro'
  robot = ET.fromstring(subprocess.run(['xacro', str(xacro)], check=True, capture_output=True, text=True).stdout)
  for block in robot.findall('ros2_control'):
    robot.remove(block)
  return robot


def with_arm_hardware(robot, side):
  """The URDF with one ros2_control block: one arm on the lobby's topics, velocity commanded."""
  robot = ET.fromstring(ET.tostring(robot))
  block = ET.SubElement(robot, 'ros2_control', name=f'lobby_{side}_arm', type='system')
  hardware = ET.SubElement(block, 'hardware')
  ET.SubElement(hardware, 'plugin').text = 'topic_based_ros2_control/TopicBasedSystem'
  for key, value in (('joint_states_topic', '/robot_joint_states'),
                     ('joint_commands_topic', f'/{side}_arm/joint_velocity_commands'),
                     ('trigger_joint_command_threshold', '-1')):  # always publish: it compares positions only
    ET.SubElement(hardware, 'param', name=key).text = value
  for joint in ARM_JOINTS:
    j = ET.SubElement(block, 'joint', name=f'{side}_{joint}')
    ET.SubElement(j, 'command_interface', name='velocity')
    ET.SubElement(j, 'state_interface', name='position')
    ET.SubElement(j, 'state_interface', name='velocity')
  return ET.tostring(robot, encoding='unicode')


def generate_launch_description():
  robot = tracy_urdf()
  nodes = [
      Node(package='robot_state_publisher', executable='robot_state_publisher',
           parameters=[{'robot_description': ET.tostring(robot, encoding='unicode')}],
           remappings=[('joint_states', '/robot_joint_states')], output='screen'),
      ExecuteProcess(cmd=['python3', str(HERE / 'tracy_lobby_relay.py')], output='screen'),
  ]
  for side in ('left', 'right'):
    ns = f'{side}_arm'
    # A controller manager in a namespace reads its URDF from <ns>/robot_description: published by
    # a robot_state_publisher of its own, whose TF goes nowhere (the one above publishes it).
    nodes.append(Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=ns,
                      parameters=[{'robot_description': with_arm_hardware(robot, side)}],
                      remappings=[('/tf', 'unused_tf'), ('/tf_static', 'unused_tf_static')], output='screen'))
    nodes.append(Node(package='controller_manager', executable='ros2_control_node', namespace=ns,
                      parameters=[str(HERE / 'controllers.yaml')], output='screen'))
    for controller in ('joint_state_broadcaster', 'forward_velocity_controller'):
      nodes.append(Node(package='controller_manager', executable='spawner', namespace=ns,
                        arguments=[controller, '-c', f'/{ns}/controller_manager', '--controller-manager-timeout', '60'],
                        output='screen'))
  return LaunchDescription(nodes)
