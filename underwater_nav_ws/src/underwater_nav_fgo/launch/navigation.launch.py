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
    
    # 水下导航节点
    navigation_node = Node(
        package='factor_graph_optimizer',
        executable='underwater_navigation_node',
        name='underwater_navigation',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'optimization_frequency': 20.0,
                'publish_tf': True,
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'max_path_length': 1000
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
    
    # 静态变换发布器 - base_link到传感器的变换
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
    rviz_config_file = PathJoinSubstitution([pkg_share, 'rviz', 'navigation.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    return LaunchDescription([
        # 启动参数
        config_file_arg,
        data_file_arg,
        playback_rate_arg,
        use_rviz_arg,
        loop_playback_arg,
        
        # 节点
        navigation_node,
        csv_player_node,
        visualization_node,
        static_tf_base_to_imu,
        static_tf_base_to_dvl,
        rviz_node,
    ])
