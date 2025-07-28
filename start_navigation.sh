#!/bin/bash

# LSTM_TimeFGO_nav 水下导航系统启动脚本
# 作者: dongchenhui
# 日期: 2025-07-19

echo "=========================================="
echo "  LSTM_TimeFGO_nav 水下导航系统启动脚本"
echo "=========================================="

# 设置项目路径
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$PROJECT_DIR/underwater_nav_ws"
DATA_DIR="$PROJECT_DIR/underwater_nav_fgo/data"
CONFIG_DIR="$PROJECT_DIR/underwater_nav_fgo/config"

echo "项目目录: $PROJECT_DIR"
echo "工作空间: $WS_DIR"

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "正在设置ROS2环境..."
    source /opt/ros/galactic/setup.bash
fi

# 进入工作空间
cd "$WS_DIR"

# 检查是否已编译
if [ ! -d "install" ]; then
    echo "项目未编译，正在编译..."
    colcon build --parallel-workers 1
    if [ $? -ne 0 ]; then
        echo "❌ 编译失败！请检查依赖项。"
        exit 1
    fi
    echo "✅ 编译完成！"
fi

# 设置工作空间环境
source install/setup.bash

# 检查数据文件
DATA_FILE="$DATA_DIR/sensor_fusion_log_20250712_102606.csv"
if [ ! -f "$DATA_FILE" ]; then
    echo "❌ 数据文件不存在: $DATA_FILE"
    echo "请确保数据文件存在。"
    exit 1
fi

echo "✅ 数据文件检查通过: $DATA_FILE"

# 启动选项
echo ""
echo "请选择启动模式:"
echo "1) 完整系统启动 (推荐)"
echo "2) 分步启动 (调试用)"
echo "3) 仅启动可视化"
echo "4) 测试数据播放器"
echo "5) 退出"

read -p "请输入选择 (1-5): " choice

case $choice in
    1)
        echo "🚀 启动完整水下导航系统..."
        echo "正在启动各个节点..."
        
        # 启动静态变换
        echo "启动静态变换发布器..."
        ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link &
        sleep 2
        
        # 启动数据播放器
        echo "启动CSV数据播放器..."
        ros2 run data_preprocessing csv_player_node \
            --ros-args \
            -p data_file:="$DATA_FILE" \
            -p playback_rate:=2.0 \
            -p start_paused:=false \
            -p publish_gps_reference:=true \
            --log-level INFO &
        sleep 3
        
        # 启动时间中心导航节点
        echo "启动时间中心导航节点..."
        ros2 run factor_graph_optimizer time_centric_navigation_node \
            --ros-args --log-level INFO &
        sleep 2
        
        # 启动GPS参考发布器
        echo "启动GPS参考发布器..."
        ros2 run factor_graph_optimizer gps_reference_publisher &
        sleep 1
        
        # 启动可视化节点
        echo "启动可视化节点..."
        ros2 run factor_graph_optimizer visualization_node &
        sleep 2
        
        # 启动RViz
        echo "启动RViz可视化..."
        RVIZ_CONFIG="$PROJECT_DIR/underwater_nav_fgo/rviz/underwater_nav_2d.rviz"
        if [ -f "$RVIZ_CONFIG" ]; then
            rviz2 -d "$RVIZ_CONFIG" &
        else
            rviz2 &
        fi
        
        echo "✅ 所有节点已启动！"
        echo "📊 请在RViz中查看导航结果"
        echo "🔍 使用 'ros2 topic list' 查看所有话题"
        echo "📈 使用 'ros2 topic hz /imu_data' 检查数据流"
        ;;
        
    2)
        echo "🔧 分步启动模式"
        echo "请手动执行以下命令:"
        echo ""
        echo "# 1. 启动静态变换"
        echo "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link"
        echo ""
        echo "# 2. 启动数据播放器"
        echo "ros2 run data_preprocessing csv_player_node --ros-args -p data_file:=\"$DATA_FILE\" -p playback_rate:=2.0"
        echo ""
        echo "# 3. 启动导航节点"
        echo "ros2 run factor_graph_optimizer time_centric_navigation_node"
        echo ""
        echo "# 4. 启动可视化"
        echo "ros2 run factor_graph_optimizer visualization_node"
        echo "rviz2 -d \"$PROJECT_DIR/underwater_nav_fgo/rviz/underwater_nav_2d.rviz\""
        ;;
        
    3)
        echo "📊 启动可视化..."
        RVIZ_CONFIG="$PROJECT_DIR/underwater_nav_fgo/rviz/underwater_nav_2d.rviz"
        if [ -f "$RVIZ_CONFIG" ]; then
            rviz2 -d "$RVIZ_CONFIG"
        else
            rviz2
        fi
        ;;
        
    4)
        echo "🧪 测试数据播放器..."
        ros2 run data_preprocessing csv_player_node \
            --ros-args \
            -p data_file:="$DATA_FILE" \
            -p playback_rate:=1.0 \
            -p start_paused:=false \
            -p publish_gps_reference:=true \
            --log-level DEBUG
        ;;
        
    5)
        echo "👋 退出"
        exit 0
        ;;
        
    *)
        echo "❌ 无效选择"
        exit 1
        ;;
esac

echo ""
echo "🎯 系统运行中..."
echo "按 Ctrl+C 停止所有节点"

# 等待用户中断
wait
