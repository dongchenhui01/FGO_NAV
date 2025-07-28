#!/bin/bash

# AUV仿真环境启动脚本
# 启动ArduSub SITL + Gazebo + MAVROS

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 获取脚本目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# 检查环境
check_environment() {
    log_info "检查仿真环境..."
    
    # 检查ROS环境
    if [ -z "$ROS_DISTRO" ]; then
        log_error "ROS环境未加载，请运行: source /opt/ros/galactic/setup.bash"
        exit 1
    fi
    
    # 检查ArduPilot
    if [ ! -d "$SCRIPT_DIR/ardupilot" ]; then
        log_error "ArduPilot未安装，请先运行 ./setup_environment.sh"
        exit 1
    fi
    
    # 检查工作空间
    if [ ! -f ~/auv_colcon_ws/install/setup.bash ]; then
        log_error "ROS工作空间未找到，请先运行 ./setup_environment.sh"
        exit 1
    fi
    
    log_success "环境检查通过"
}

# 启动ArduSub SITL
start_sitl() {
    log_info "启动ArduSub SITL..."
    
    cd "$SCRIPT_DIR/ardupilot"
    
    # 启动SITL（在后台运行）
    xterm -title "ArduSub SITL" -e bash -c "
        source ~/.bashrc
        cd '$SCRIPT_DIR/ardupilot'
        python3 Tools/autotest/sim_vehicle.py -v ArduSub --console --out=udp:127.0.0.1:14550
        exec bash
    " &
    
    # 等待SITL启动
    log_info "等待SITL启动..."
    sleep 10
    
    log_success "ArduSub SITL已启动"
}

# 启动Gazebo
start_gazebo() {
    log_info "启动Gazebo仿真..."
    
    # 设置Gazebo环境变量
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/auv_colcon_ws/src/ardupilot_gazebo/models
    export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/auv_colcon_ws/src/ardupilot_gazebo/build
    
    # 启动Gazebo（在新终端中）
    xterm -title "Gazebo Simulation" -e bash -c "
        source ~/.bashrc
        source ~/auv_colcon_ws/install/setup.bash
        export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:~/auv_colcon_ws/src/ardupilot_gazebo/models
        export GAZEBO_PLUGIN_PATH=\$GAZEBO_PLUGIN_PATH:~/auv_colcon_ws/src/ardupilot_gazebo/build
        gazebo --verbose
        exec bash
    " &
    
    # 等待Gazebo启动
    log_info "等待Gazebo启动..."
    sleep 15
    
    log_success "Gazebo仿真已启动"
}

# 启动MAVROS
start_mavros() {
    log_info "启动MAVROS..."
    
    # 启动MAVROS（在新终端中）
    xterm -title "MAVROS" -e bash -c "
        source ~/.bashrc
        source ~/auv_colcon_ws/install/setup.bash
        ros2 launch mavros apm.launch fcu_url:=udp://:14550@127.0.0.1:14551
        exec bash
    " &
    
    # 等待MAVROS启动
    log_info "等待MAVROS启动..."
    sleep 5
    
    log_success "MAVROS已启动"
}

# 启动数据监控
start_monitoring() {
    log_info "启动数据监控..."
    
    # 启动ROS话题监控（在新终端中）
    xterm -title "ROS Topics Monitor" -e bash -c "
        source ~/.bashrc
        source ~/auv_colcon_ws/install/setup.bash
        echo '=== 可用的ROS话题 ==='
        ros2 topic list
        echo ''
        echo '=== MAVROS状态监控 ==='
        echo '使用以下命令监控数据:'
        echo 'ros2 topic echo /mavros/state'
        echo 'ros2 topic echo /mavros/imu/data'
        echo 'ros2 topic echo /mavros/global_position/global'
        echo 'ros2 topic echo /mavros/local_position/pose'
        exec bash
    " &
    
    log_success "数据监控已启动"
}

# 显示使用说明
show_usage() {
    log_info "仿真环境启动完成！"
    echo ""
    echo "=== 使用说明 ==="
    echo "1. ArduSub SITL: 飞控仿真，提供MAVLink接口"
    echo "2. Gazebo: 3D物理仿真环境"
    echo "3. MAVROS: ROS与MAVLink的桥梁"
    echo "4. 监控终端: 显示可用的ROS话题"
    echo ""
    echo "=== 常用命令 ==="
    echo "# 查看所有话题"
    echo "ros2 topic list"
    echo ""
    echo "# 监控IMU数据"
    echo "ros2 topic echo /mavros/imu/data"
    echo ""
    echo "# 监控GPS数据"
    echo "ros2 topic echo /mavros/global_position/global"
    echo ""
    echo "# 监控本地位置"
    echo "ros2 topic echo /mavros/local_position/pose"
    echo ""
    echo "# 监控系统状态"
    echo "ros2 topic echo /mavros/state"
    echo ""
    echo "=== 控制AUV ==="
    echo "# 解锁AUV"
    echo "ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \"{value: true}\""
    echo ""
    echo "# 设置模式为GUIDED"
    echo "ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \"{custom_mode: 'GUIDED'}\""
    echo ""
    echo "按Ctrl+C退出所有进程"
}

# 清理函数
cleanup() {
    log_info "正在关闭仿真环境..."
    
    # 杀死所有相关进程
    pkill -f "sim_vehicle.py" || true
    pkill -f "gazebo" || true
    pkill -f "mavros" || true
    
    log_success "仿真环境已关闭"
    exit 0
}

# 设置信号处理
trap cleanup SIGINT SIGTERM

# 主函数
main() {
    log_info "启动AUV仿真环境..."
    
    check_environment
    
    # 按顺序启动各个组件
    start_sitl
    start_gazebo
    start_mavros
    start_monitoring
    
    show_usage
    
    # 保持脚本运行
    log_info "仿真环境正在运行，按Ctrl+C退出..."
    while true; do
        sleep 1
    done
}

# 检查参数
case "${1:-}" in
    --help|-h)
        echo "AUV仿真环境启动脚本"
        echo ""
        echo "用法: $0 [选项]"
        echo ""
        echo "选项:"
        echo "  --help, -h     显示此帮助信息"
        echo "  --check        仅检查环境，不启动仿真"
        echo ""
        echo "示例:"
        echo "  $0              # 启动完整仿真环境"
        echo "  $0 --check     # 检查环境配置"
        exit 0
        ;;
    --check)
        check_environment
        log_success "环境检查完成，可以启动仿真"
        exit 0
        ;;
    "")
        main
        ;;
    *)
        log_error "未知参数: $1"
        echo "使用 $0 --help 查看帮助"
        exit 1
        ;;
esac
