"""Inspect the accurately textured scene without starting the patrol."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('paused',default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('community_demo'),'launch','demo.launch.py')),
            launch_arguments={'gui':LaunchConfiguration('gui'),'paused':LaunchConfiguration('paused'),'patrol':'false'}.items()),
    ])
