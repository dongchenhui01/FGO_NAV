#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare('underwater_nav_fgo').find('underwater_nav_fgo')
    
    # 声明启动参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'system_params.yaml']),
        description='Path to the configuration file'
    )
    
    data_file_arg = DeclareLaunchArgument(
        'data_file',
        default_value='',
        description='Path to the CSV data file'
    )
    
    playback_rate_arg = DeclareLaunchArgument(
        'playback_rate',
        default_value='1.0',
        description='Playback rate multiplier'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )
    
    loop_playback_arg = DeclareLaunchArgument(
        'loop_playback',
        default_value='false',
        description='Whether to loop the data playback'
    )
    
    # 时间中心参数
    time_window_size_arg = DeclareLaunchArgument(
        'time_window_size',
        default_value='2.0',
        description='Time window size for time-centric optimization (seconds)'
    )
    
    interpolation_method_arg = DeclareLaunchArgument(
        'interpolation_method',
        default_value='gp',
        description='Interpolation method: linear, gp, spline'
    )
    
    optimization_frequency_arg = DeclareLaunchArgument(
        'optimization_frequency',
        default_value='20.0',
        description='Optimization frequency (Hz)'
    )
    
    query_frequency_arg = DeclareLaunchArgument(
        'query_frequency',
        default_value='50.0',
        description='Continuous trajectory query frequency (Hz)'
    )
    
    enable_continuous_query_arg = DeclareLaunchArgument(
        'enable_continuous_query',
        default_value='true',
        description='Enable continuous trajectory querying'
    )
    
    # 时间中心水下导航节点
    time_centric_navigation_node = Node(
        package='factor_graph_optimizer',
        executable='time_centric_navigation_node',
        name='time_centric_navigation',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'optimization_frequency': LaunchConfiguration('optimization_frequency'),
                'publish_tf': True,
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'max_path_length': 1000,
                
                # 时间中心参数
                'time_window_size': LaunchConfiguration('time_window_size'),
                'interpolation_method': LaunchConfiguration('interpolation_method'),
                'enable_continuous_query': LaunchConfiguration('enable_continuous_query'),
                'query_frequency': LaunchConfiguration('query_frequency'),
            }
        ],
        remappings=[
            ('imu_data', '/sensors/imu'),
            ('dvl_data', '/sensors/dvl'),
            ('navigation_state', '/navigation/state'),
            ('pose', '/navigation/pose'),
            ('velocity', '/navigation/velocity'),
            ('trajectory', '/navigation/trajectory')
        ]
    )
    
    # CSV数据播放器节点
    csv_player_node = Node(
        package='data_preprocessing',
        executable='csv_player_node',
        name='csv_player',
        output='screen',
        parameters=[
            {
                'data_file': LaunchConfiguration('data_file'),
                'playback_rate': LaunchConfiguration('playback_rate'),
                'loop_playback': LaunchConfiguration('loop_playback'),
                'start_paused': False,
                'publish_gps_reference': True
            }
        ],
        remappings=[
            ('imu_data', '/sensors/imu'),
            ('dvl_data', '/sensors/dvl'),
            ('gps_reference', '/sensors/gps_reference')
        ],
        condition=IfCondition(LaunchConfiguration('data_file'))
    )
    
    # 可视化节点
    visualization_node = Node(
        package='visualization',
        executable='trajectory_visualization_node',
        name='trajectory_visualization',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'visualization_frequency': 5.0,
                'output_directory': '/tmp/underwater_nav_results',
                'frame_id': 'odom',
                'publish_markers': True,
                'publish_paths': True,
                'max_path_length': 1000
            }
        ],
        remappings=[
            ('navigation_state', '/navigation/state'),
            ('gps_reference', '/sensors/gps_reference'),
            ('trajectory_markers', '/visualization/trajectory_markers'),
            ('error_markers', '/visualization/error_markers'),
            ('estimated_path', '/visualization/estimated_path'),
            ('reference_path', '/visualization/reference_path')
        ]
    )
    
    # 静态变换发布器
    static_tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
    )
    
    static_tf_base_to_dvl = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_dvl',
        arguments=['0', '0', '-0.5', '0', '0', '0', 'base_link', 'dvl_link']
    )
    
    # RViz可视化
    rviz_config_file = PathJoinSubstitution([pkg_share, 'rviz', 'time_centric_navigation.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    # 时间中心演示节点 (可选)
    demo_node = Node(
        package='factor_graph_optimizer',
        executable='time_centric_demo_node',
        name='time_centric_demo',
        output='screen',
        parameters=[
            {
                'demo_mode': True,
                'query_interval': 0.5,
                'demo_duration': 60.0
            }
        ],
        condition=IfCondition('false')  # 默认不启动，可以通过参数控制
    )
    
    return LaunchDescription([
        # 启动参数
        config_file_arg,
        data_file_arg,
        playback_rate_arg,
        use_rviz_arg,
        loop_playback_arg,
        
        # 时间中心参数
        time_window_size_arg,
        interpolation_method_arg,
        optimization_frequency_arg,
        query_frequency_arg,
        enable_continuous_query_arg,
        
        # 节点
        time_centric_navigation_node,
        csv_player_node,
        visualization_node,
        static_tf_base_to_imu,
        static_tf_base_to_dvl,
        rviz_node,
        demo_node,
    ])
