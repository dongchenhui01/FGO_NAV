#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # 声明启动参数
    csv_file_arg = DeclareLaunchArgument(
        'csv_file_path',
        default_value='/home/dongchenhui/underwater_nav_fgo/data/sensor_fusion_log_20250712_102606.csv',
        description='CSV数据文件路径'
    )
    
    playback_rate_arg = DeclareLaunchArgument(
        'playback_rate',
        default_value='1.0',
        description='数据播放速率'
    )
    
    # 数据播放节点
    csv_player_node = Node(
        package='data_preprocessing',
        executable='csv_player_node',
        name='csv_player',
        parameters=[{
            'csv_file_path': LaunchConfiguration('csv_file_path'),
            'playback_rate': LaunchConfiguration('playback_rate'),
            'loop_playback': True
        }],
        output='screen'
    )
    
    # 时间中心导航节点
    time_centric_nav_node = Node(
        package='factor_graph_optimizer',
        executable='time_centric_navigation_node',
        name='time_centric_navigator',
        parameters=[{
            'time_window_size': 5.0,
            'interpolation_method': 'gp',
            'optimization_frequency': 10.0
        }],
        output='screen'
    )
    
    # 可视化节点
    visualization_node = Node(
        package='factor_graph_optimizer',
        executable='visualization_node',
        name='underwater_fgo_visualizer',
        output='screen'
    )
    
    # RViz2可视化
    rviz_config_file = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'underwater_nav.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        output='screen'
    )
    
    # rqt_plot用于实时数据监控
    rqt_plot_node = ExecuteProcess(
        cmd=['rqt_plot', 
             '/interpolation_confidence/data[0]',
             '/interpolation_confidence/data[1]',
             '/interpolation_confidence/data[2]'],
        output='screen'
    )
    
    return LaunchDescription([
        csv_file_arg,
        playback_rate_arg,
        csv_player_node,
        time_centric_nav_node,
        visualization_node,
        rviz_node,
        # rqt_plot_node,  # 可选：取消注释以启用实时绘图
    ])
