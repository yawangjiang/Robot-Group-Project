import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    desc = get_package_share_directory('robomaster_ep_description')
    moveit = get_package_share_directory('robomaster_ep_moveit_config')
    gz = get_package_share_directory('robomaster_ep_gazebo')
    controllers = os.path.join(gz, 'config', 'ros2_controllers_mock.yaml')
    robot_xml = xacro.process_file(os.path.join(desc, 'urdf', 'robomaster_ep.urdf.xacro'),
        mappings={'use_gazebo': 'false', 'use_fake_hardware': 'true',
                  'controllers_file': controllers}).toxml()
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_xml}]),
        Node(package='controller_manager', executable='ros2_control_node', output='screen',
             parameters=[{'robot_description': robot_xml}, controllers]),
        TimerAction(period=1.0, actions=[Node(
            package='controller_manager', executable='spawner', output='screen',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'])]),
        TimerAction(period=2.0, actions=[Node(
            package='controller_manager', executable='spawner', output='screen',
            arguments=['arm_controller', '--controller-manager', '/controller_manager'])]),
        TimerAction(period=3.0, actions=[Node(
            package='controller_manager', executable='spawner', output='screen',
            arguments=['gripper_controller', '--controller-manager', '/controller_manager'])]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(moveit, 'launch', 'move_group.launch.py')),
                                 launch_arguments={'use_fake_hardware': 'true', 'use_sim_time': 'false'}.items()),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(moveit, 'rviz', 'moveit.rviz')],
             parameters=[{'robot_description': robot_xml}]),
    ])
