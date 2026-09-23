import os
import tempfile
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def graphics_environment():
    """Use SSH forwarding when present, otherwise attach to the local GNOME display."""
    env = {}
    if os.environ.get('DISPLAY'):
        return env
    local_display = '/tmp/.X11-unix/X0'
    local_authority = f'/run/user/{os.getuid()}/gdm/Xauthority'
    if os.path.exists(local_display) and os.path.exists(local_authority):
        env['DISPLAY'] = ':0'
        env['XAUTHORITY'] = local_authority
        env['XDG_RUNTIME_DIR'] = f'/run/user/{os.getuid()}'
    return env

def setup(context):
    desc = get_package_share_directory('robomaster_ep_description')
    gazebo = get_package_share_directory('robomaster_ep_gazebo')
    moveit = get_package_share_directory('robomaster_ep_moveit_config')
    controllers = os.path.join(gazebo, 'config', 'ros2_controllers.yaml')
    robot_xml = xacro.process_file(os.path.join(desc, 'urdf', 'robomaster_ep.urdf.xacro'),
        mappings={'use_sim': 'true', 'use_gazebo': 'true', 'use_fake_hardware': 'false',
                  'controllers_file': controllers}).toxml()
    robot_file = os.path.join(tempfile.gettempdir(), 'robomaster_ep_gazebo.urdf')
    with open(robot_file, 'w', encoding='utf-8') as stream:
        stream.write(robot_xml)
    world = os.path.join(gazebo, 'worlds', 'pick_place.sdf')
    spawn_req = f'sdf_filename: "{robot_file}", name: "robomaster_ep", allow_renaming: false'
    gui_env = graphics_environment()
    joint_state_spawner = Node(package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '30'], output='screen')
    arm_spawner = Node(package='controller_manager', executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '30'], output='screen')
    gripper_spawner = Node(package='controller_manager', executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '30'], output='screen')
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit, 'launch', 'move_group.launch.py')),
        launch_arguments={'use_sim_time': 'true', 'use_fake_hardware': 'false'}.items())
    rviz = Node(package='rviz2', executable='rviz2', output='screen',
        condition=UnlessCondition(LaunchConfiguration('headless')),
        arguments=['-d', os.path.join(moveit, 'rviz', 'moveit.rviz')],
        parameters=[{'robot_description': robot_xml, 'use_sim_time': True}],
        additional_env=gui_env)
    return [
        ExecuteProcess(cmd=['ign', 'gazebo', '-r', '-s', '-v', '3', world], output='screen'),
        ExecuteProcess(cmd=['ign', 'gazebo', '-g', '-v', '2'], output='screen',
                       condition=UnlessCondition(LaunchConfiguration('headless')),
                       additional_env=gui_env),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen', arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/world/robomaster_world/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            # Let ros_gz_bridge select the request / response mapping for services.
            # The Humble backport on Jetson accepts the explicit four-part syntax
            # but fails to create the ROS service for this mixed gz / ignition ABI.
            '/world/robomaster_world/set_pose@ros_gz_interfaces/srv/SetEntityPose']),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_xml, 'use_sim_time': True}]),
        TimerAction(period=2.0, actions=[ExecuteProcess(cmd=['ign', 'service', '-s',
            '/world/robomaster_world/create', '--reqtype', 'ignition.msgs.EntityFactory',
            '--reptype', 'ignition.msgs.Boolean', '--timeout', '10000', '--req', spawn_req], output='screen')]),
        # controller_manager service calls can overlap on this Jetson and make a
        # controller disappear between load and configure.  Chain each spawner
        # from the successful exit of the previous process instead.
        TimerAction(period=5.0, actions=[joint_state_spawner]),
        RegisterEventHandler(OnProcessExit(target_action=joint_state_spawner,
                                           on_exit=[arm_spawner])),
        RegisterEventHandler(OnProcessExit(target_action=arm_spawner,
                                           on_exit=[gripper_spawner])),
        RegisterEventHandler(OnProcessExit(target_action=gripper_spawner,
                                           on_exit=[move_group, TimerAction(period=2.0, actions=[rviz])])),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('headless_rendering', default_value='true'),
        OpaqueFunction(function=setup),
    ])
