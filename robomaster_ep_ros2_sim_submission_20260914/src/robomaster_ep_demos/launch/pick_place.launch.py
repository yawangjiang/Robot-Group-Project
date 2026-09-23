from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cycles', default_value='1'),
        Node(package='robomaster_ep_demos', executable='pick_place', output='screen',
             parameters=[{'cycles': LaunchConfiguration('cycles'), 'use_sim_time': True}]),
    ])
