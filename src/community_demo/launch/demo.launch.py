"""Start the first-run community world, bridge and single-lap patrol."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    world = os.path.join(get_package_share_directory('community_demo'),
                         'worlds', 'community.sdf')
    gz = ExecuteProcess(
        cmd=['/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz', 'sim', '-s', '-r', world],
        output='screen',
    )
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/patrol_bot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
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
        gz,
        bridge,
        TimerAction(period=4.0, actions=[manager, vision, cmd_bridge, patrol]),
    ])
