"""Start the first-run community world, bridge and single-lap patrol."""

import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
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
        'GZ_SIM_SYSTEM_PLUGIN_PATH': os.path.join(get_package_prefix('community_signal_system'),'lib') + ':' + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH',''),
        'QT_AUTO_SCREEN_SCALE_FACTOR': '0', 'QT_SCALE_FACTOR': '1', 'QT_FONT_DPI': '96',
    })]


def generate_launch_description():
    stress=LaunchConfiguration('stress')
    def sensor_topic(normal,perturbed):
        return PythonExpression(['"',perturbed,'" if "',stress,'" == "true" else "',normal,'"'])
    camera_remap=('/camera/image',sensor_topic('/camera/image','/stress/camera/image'))
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
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
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
            'max_linear_x': .26,
            'max_angular_z': .70,
            'max_linear_accel': .50,
            'max_angular_accel': 1.2,
            'nav_timeout_s': .5,
        }],
    )
    vision = Node(package='community_demo', executable='vision', output='screen', condition=IfCondition(LaunchConfiguration('vision')))
    cmd_bridge = Node(package='community_demo', executable='cmd_bridge', output='screen')
    patrol = Node(package='community_demo', executable='patrol', parameters=[{'use_sim_time':True}], remappings=[camera_remap], output='screen', condition=IfCondition(LaunchConfiguration('patrol')))
    localization = Node(package='community_demo', executable='localization', parameters=[{'use_sim_time':True}],
        remappings=[camera_remap,('/model/patrol_bot/odometry',sensor_topic('/model/patrol_bot/odometry','/stress/odometry')),('/lidar/raw/points',sensor_topic('/lidar/raw/points','/stress/points'))],output='screen')
    from launch_ros.parameter_descriptions import ParameterValue
    perturbations=Node(package='community_demo',executable='sensor_stress',parameters=[{'use_sim_time':True,'seed':ParameterValue(LaunchConfiguration('stress_seed'),value_type=int)}],output='screen',condition=IfCondition(stress))
    slam = Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', parameters=[os.path.join(get_package_share_directory('community_demo'),'config','slam.yaml')], output='screen', condition=IfCondition(LaunchConfiguration('slam')))
    signals = Node(package='community_demo', executable='traffic_signals', output='screen', parameters=[{'use_sim_time':True}])
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='community.sdf'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('patrol', default_value='true'),
        DeclareLaunchArgument('vision', default_value='true'),
        DeclareLaunchArgument('stress', default_value='false'),
        DeclareLaunchArgument('stress_seed', default_value='20261004'),
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument('navigation_map', default_value=os.path.join(get_package_share_directory('community_demo'),'maps','roads.yaml')),
        DeclareLaunchArgument('navigation_config', default_value=os.path.join(get_package_share_directory('community_demo'),'config','navigation.yaml')),
        OpaqueFunction(function=start_gazebo),
        bridge, perturbations, localization, slam,
        Node(package='rviz2',executable='rviz2',arguments=['-d',os.path.join(get_package_share_directory('community_demo'),'config','mapping.rviz')],parameters=[{'use_sim_time':True}],condition=IfCondition(LaunchConfiguration('rviz')),output='screen'),
        Node(package='community_demo',executable='slam_scan',parameters=[{'use_sim_time':True}],
             output='screen',condition=IfCondition(LaunchConfiguration('slam'))),
        Node(package='tf2_ros', executable='static_transform_publisher', name='saved_map_odom_anchor',
             arguments=['--frame-id','slam_map','--child-frame-id','odom'],
             parameters=[{'use_sim_time':True}], condition=UnlessCondition(LaunchConfiguration('slam'))),
        Node(package='nav2_map_server',executable='map_server',name='map_server',parameters=[LaunchConfiguration('navigation_config'),{'yaml_filename':LaunchConfiguration('navigation_map')}],output='screen'),
        Node(package='nav2_planner',executable='planner_server',name='planner_server',parameters=[LaunchConfiguration('navigation_config')],output='screen'),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='nav_lifecycle',parameters=[{'use_sim_time':True,'autostart':True,'node_names':['map_server','planner_server']}],output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='slam_lifecycle', parameters=[{'use_sim_time':True,'autostart':True,'bond_timeout':0.0,'node_names':['slam_toolbox']}], output='screen', condition=IfCondition(LaunchConfiguration('slam'))),
        TimerAction(period=4.0, actions=[manager, vision, cmd_bridge, signals, patrol]),
    ])
