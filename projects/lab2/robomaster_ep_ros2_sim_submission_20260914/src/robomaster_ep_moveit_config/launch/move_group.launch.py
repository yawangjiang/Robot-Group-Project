import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def load_yaml(package, path):
    with open(os.path.join(get_package_share_directory(package), path), encoding='utf-8') as stream:
        return yaml.safe_load(stream)

def load_text(package, path):
    with open(os.path.join(get_package_share_directory(package), path), encoding='utf-8') as stream:
        return stream.read()

def launch_setup(context):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    fake = LaunchConfiguration('use_fake_hardware').perform(context).lower() == 'true'
    desc_share = get_package_share_directory('robomaster_ep_description')
    gz_share = get_package_share_directory('robomaster_ep_gazebo')
    robot_xml = xacro.process_file(
        os.path.join(desc_share, 'urdf', 'robomaster_ep.urdf.xacro'),
        mappings={'use_gazebo': 'false', 'use_fake_hardware': str(fake).lower(),
                  'controllers_file': os.path.join(gz_share, 'config', 'ros2_controllers.yaml')}).toxml()
    semantic = load_text('robomaster_ep_moveit_config', 'config/robomaster_ep.srdf')
    ompl = load_yaml('robomaster_ep_moveit_config', 'config/ompl_planning.yaml')
    limits = load_yaml('robomaster_ep_moveit_config', 'config/joint_limits.yaml')
    kinematics = load_yaml('robomaster_ep_moveit_config', 'config/kinematics.yaml')
    scene = load_yaml('robomaster_ep_moveit_config', 'config/planning_scene_monitor.yaml')
    # Both Gazebo and mock_components expose the same FollowJointTrajectory
    # actions. Humble 2.5.9 does not ship the legacy fake controller plugin.
    controllers = load_yaml('robomaster_ep_moveit_config', 'config/moveit_controllers.yaml')
    params = [
        {'robot_description': robot_xml, 'robot_description_semantic': semantic,
         'robot_description_kinematics': kinematics, 'robot_description_planning': limits,
         'use_sim_time': use_sim_time, 'publish_robot_description': True,
         'publish_robot_description_semantic': True},
        {'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
         'ompl': ompl}, scene, controllers,
    ]
    return [Node(package='moveit_ros_move_group', executable='move_group', output='screen', parameters=params)]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
