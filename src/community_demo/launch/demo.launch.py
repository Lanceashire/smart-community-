"""Start the first-run community world, bridge and single-lap patrol."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def start_gazebo(context):
    share = get_package_share_directory('community_demo')
    cmd = ['/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz', 'sim']
    if LaunchConfiguration('gui').perform(context).lower() != 'true':
        cmd.append('-s')
    if LaunchConfiguration('paused').perform(context).lower() != 'true':
        cmd.append('-r')
    cmd += ['--gui-config', os.path.join(share, 'gui', 'overview.config'),
            os.path.join(share, 'worlds', LaunchConfiguration('world').perform(context))]
    return [ExecuteProcess(cmd=cmd, output='screen', on_exit=[Shutdown(reason='Gazebo exited')], additional_env={
        'GZ_SIM_RESOURCE_PATH': os.path.join(share,'models') + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH',''),
        'QT_AUTO_SCREEN_SCALE_FACTOR': '0', 'QT_SCALE_FACTOR': '1', 'QT_FONT_DPI': '96',
    })]


def generate_launch_description():
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
        parameters=[{f'qos_overrides.{topic}.publisher.{key}': value
                     for topic in ['/camera/image','/lidar/raw/points','/model/patrol_bot/odometry',
                                   '/world/community/model/patrol_bot/joint_state','/model/patrol_bot/ground_truth']
                     for key,value in [('reliability','best_effort'),('depth',2)]}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/patrol_bot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/patrol_bot/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/lidar/raw/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/world/community/model/patrol_bot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/model/patrol_bot/camera_pan@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/patrol_bot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ], output='screen',
    )
    manager = Node(
        package='sentry_motion_manager', executable='sentry_motion_manager_node',
        name='motion_manager', output='screen', parameters=[{
            'command_output_enabled': True,
            'use_sim_time': True,
            'command_frame_id': 'base_link',
            'nav_command_topic': '/cmd_vel_nav',
            'odometry_topic': '/model/patrol_bot/odometry',
            'output_command_topic': '/cmd_vel',
            'max_linear_x': .24,
            'max_angular_z': .70,
            'max_linear_accel': .50,
            'max_angular_accel': 1.2,
            'nav_timeout_s': .5,
        }],
    )
    vision = Node(package='community_demo', executable='vision', output='screen', condition=IfCondition(LaunchConfiguration('vision')))
    cmd_bridge = Node(package='community_demo', executable='cmd_bridge', output='screen')
    patrol = Node(package='community_demo', executable='patrol', parameters=[{'use_sim_time':True}], output='screen', condition=IfCondition(LaunchConfiguration('patrol')))
    localization = Node(package='community_demo', executable='localization', parameters=[{'use_sim_time':True}], output='screen')
    slam = Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', parameters=[os.path.join(get_package_share_directory('community_demo'),'config','slam.yaml')], output='screen')
    signals = Node(package='community_demo', executable='traffic_signals', output='screen', parameters=[{'use_sim_time':True}])
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='community.sdf'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('patrol', default_value='true'),
        DeclareLaunchArgument('vision', default_value='true'),
        OpaqueFunction(function=start_gazebo),
        bridge, localization, slam,
        Node(package='nav2_map_server',executable='map_server',name='map_server',parameters=[os.path.join(get_package_share_directory('community_demo'),'config/navigation.yaml'),{'yaml_filename':os.path.join(get_package_share_directory('community_demo'),'maps/roads.yaml')}],output='screen'),
        Node(package='nav2_planner',executable='planner_server',name='planner_server',parameters=[os.path.join(get_package_share_directory('community_demo'),'config/navigation.yaml')],output='screen'),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='nav_lifecycle',parameters=[{'use_sim_time':True,'autostart':True,'node_names':['map_server','planner_server']}],output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='slam_lifecycle', parameters=[{'use_sim_time':True,'autostart':True,'bond_timeout':0.0,'node_names':['slam_toolbox']}], output='screen'),
        TimerAction(period=4.0, actions=[manager, vision, cmd_bridge, signals, patrol]),
    ])
