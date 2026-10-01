"""Isolated offline camera capture: no patrol, localization or answer-aware runtime."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def start(context):
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    world = LaunchConfiguration('world').perform(context)
    if not os.path.isabs(world):
        world = os.path.join(root, 'worlds', world)
    return [ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', world], output='screen',
        on_exit=[Shutdown(reason='Capture simulator exited')],
        additional_env={'GZ_SIM_RESOURCE_PATH': os.path.join(root, 'models')})]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='training.sdf'),
        OpaqueFunction(function=start),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='capture_bridge', arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/model/patrol_bot/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/patrol_bot/camera_pan@std_msgs/msg/Float64]gz.msgs.Double',
        ], output='screen'),
    ])
