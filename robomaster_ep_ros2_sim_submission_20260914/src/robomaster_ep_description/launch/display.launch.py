from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    model = PathJoinSubstitution([FindPackageShare('robomaster_ep_description'), 'urdf', 'robomaster_ep.urdf.xacro'])
    description = {'robot_description': Command([FindExecutable(name='xacro'), ' ', model, ' use_gazebo:=false'])}
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[description]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui', condition=None),
        Node(package='rviz2', executable='rviz2', arguments=['-d', PathJoinSubstitution([FindPackageShare('robomaster_ep_description'), 'rviz', 'display.rviz'])]),
    ])
