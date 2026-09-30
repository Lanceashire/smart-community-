"""Start the first-run community world, bridge and single-lap patrol."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start_gazebo(context):
    share = get_package_share_directory('community_demo')
    cmd = ['/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz', 'sim']
    if LaunchConfiguration('gui').perform(context).lower() != 'true':
        cmd.append('-s')
    if LaunchConfiguration('paused').perform(context).lower() != 'true':
        cmd.append('-r')
    cmd += ['--gui-config', os.path.join(share, 'gui', 'overview.config'),
            os.path.join(share, 'worlds', 'community.sdf')]
    return [ExecuteProcess(cmd=cmd, output='screen', on_exit=[Shutdown(reason='Gazebo exited')], additional_env={
        'QT_AUTO_SCREEN_SCALE_FACTOR': '0', 'QT_SCALE_FACTOR': '1', 'QT_FONT_DPI': '96',
    })]


def generate_launch_description():
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/patrol_bot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/patrol_bot/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/model/patrol_bot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ], output='screen',
    )
    manager = Node(
        package='sentry_motion_manager', executable='sentry_motion_manager_node',
        name='motion_manager', output='screen', parameters=[{
            'command_output_enabled': True,
            'command_frame_id': 'base_link',
            'nav_command_topic': '/cmd_vel_nav',
            'odometry_topic': '/model/patrol_bot/odometry',
            'output_command_topic': '/cmd_vel',
            'max_linear_x': .20,
            'max_angular_z': .70,
            'max_linear_accel': .50,
            'max_angular_accel': 1.2,
            'nav_timeout_s': .5,
        }],
    )
    vision = Node(package='community_demo', executable='vision', output='screen')
    cmd_bridge = Node(package='community_demo', executable='cmd_bridge', output='screen')
    patrol = Node(package='community_demo', executable='patrol', output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('paused', default_value='false'),
        OpaqueFunction(function=start_gazebo),
        bridge,
        TimerAction(period=4.0, actions=[manager, vision, cmd_bridge, patrol]),
    ])
