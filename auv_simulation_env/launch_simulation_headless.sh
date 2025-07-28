#!/bin/bash

# AUV仿真环境无GUI启动脚本
# 适用于没有图形界面或远程连接的环境

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
        log_error "ArduPilot未安装"
        exit 1
    fi
    
    # 检查工作空间
    if [ ! -f ~/auv_colcon_ws/install/setup.bash ]; then
        log_error "ROS工作空间未找到"
        exit 1
    fi
    
    log_success "环境检查通过"
}

# 启动ArduSub SITL (后台)
start_sitl() {
    log_info "启动ArduSub SITL (后台模式)..."
    
    cd "$SCRIPT_DIR/ardupilot"
    
    # 启动SITL在后台
    nohup python3 Tools/autotest/sim_vehicle.py -v ArduSub --console --out=udp:127.0.0.1:14550 > sitl.log 2>&1 &
    SITL_PID=$!
    echo $SITL_PID > /tmp/auv_sitl.pid
    
    # 等待SITL启动
    log_info "等待SITL启动..."
    sleep 10
    
    if kill -0 $SITL_PID 2>/dev/null; then
        log_success "ArduSub SITL已启动 (PID: $SITL_PID)"
    else
        log_error "ArduSub SITL启动失败"
        exit 1
    fi
}

# 启动MAVROS (后台)
start_mavros() {
    log_info "启动MAVROS (后台模式)..."
    
    # 等待SITL完全启动
    sleep 5
    
    # 启动MAVROS在后台
    source ~/.bashrc
    source ~/auv_colcon_ws/install/setup.bash
    nohup ros2 launch mavros apm.launch fcu_url:=udp://:14550@127.0.0.1:14551 > mavros.log 2>&1 &
    MAVROS_PID=$!
    echo $MAVROS_PID > /tmp/auv_mavros.pid
    
    # 等待MAVROS启动
    log_info "等待MAVROS启动..."
    sleep 5
    
    if kill -0 $MAVROS_PID 2>/dev/null; then
        log_success "MAVROS已启动 (PID: $MAVROS_PID)"
    else
        log_error "MAVROS启动失败"
        exit 1
    fi
}

# 启动数据回放 (可选)
start_data_replay() {
    if [ -f "data/real_data/sensor_fusion_log_20250712_102606.csv" ]; then
        log_info "启动数据回放 (后台模式)..."
        
        nohup python3 scripts/data_replay.py data/real_data/sensor_fusion_log_20250712_102606.csv --rate 1.0 > data_replay.log 2>&1 &
        REPLAY_PID=$!
        echo $REPLAY_PID > /tmp/auv_replay.pid
        
        log_success "数据回放已启动 (PID: $REPLAY_PID)"
    fi
}

# 显示状态
show_status() {
    log_info "仿真环境状态:"
    echo ""
    
    # 检查SITL
    if [ -f /tmp/auv_sitl.pid ] && kill -0 $(cat /tmp/auv_sitl.pid) 2>/dev/null; then
        echo "✅ ArduSub SITL: 运行中 (PID: $(cat /tmp/auv_sitl.pid))"
    else
        echo "❌ ArduSub SITL: 未运行"
    fi
    
    # 检查MAVROS
    if [ -f /tmp/auv_mavros.pid ] && kill -0 $(cat /tmp/auv_mavros.pid) 2>/dev/null; then
        echo "✅ MAVROS: 运行中 (PID: $(cat /tmp/auv_mavros.pid))"
    else
        echo "❌ MAVROS: 未运行"
    fi
    
    # 检查数据回放
    if [ -f /tmp/auv_replay.pid ] && kill -0 $(cat /tmp/auv_replay.pid) 2>/dev/null; then
        echo "✅ 数据回放: 运行中 (PID: $(cat /tmp/auv_replay.pid))"
    else
        echo "⚪ 数据回放: 未启动"
    fi
    
    echo ""
    echo "=== 使用说明 ==="
    echo "查看ROS话题: ros2 topic list"
    echo "监控MAVROS状态: ros2 topic echo /mavros/state"
    echo "监控IMU数据: ros2 topic echo /mavros/imu/data"
    echo "查看SITL日志: tail -f sitl.log"
    echo "查看MAVROS日志: tail -f mavros.log"
    echo "停止仿真: ./stop_simulation.sh"
}

# 清理函数
cleanup() {
    log_info "正在关闭仿真环境..."
    
    # 停止所有进程
    if [ -f /tmp/auv_replay.pid ]; then
        kill $(cat /tmp/auv_replay.pid) 2>/dev/null || true
        rm -f /tmp/auv_replay.pid
    fi
    
    if [ -f /tmp/auv_mavros.pid ]; then
        kill $(cat /tmp/auv_mavros.pid) 2>/dev/null || true
        rm -f /tmp/auv_mavros.pid
    fi
    
    if [ -f /tmp/auv_sitl.pid ]; then
        kill $(cat /tmp/auv_sitl.pid) 2>/dev/null || true
        rm -f /tmp/auv_sitl.pid
    fi
    
    log_success "仿真环境已关闭"
    exit 0
}

# 设置信号处理
trap cleanup SIGINT SIGTERM

# 主函数
main() {
    log_info "启动AUV仿真环境 (无GUI模式)..."
    
    check_environment
    start_sitl
    start_mavros
    start_data_replay
    show_status
    
    log_info "仿真环境已启动，按Ctrl+C退出..."
    
    # 保持脚本运行
    while true; do
        sleep 5
        # 检查进程是否还在运行
        if [ -f /tmp/auv_sitl.pid ] && ! kill -0 $(cat /tmp/auv_sitl.pid) 2>/dev/null; then
            log_error "ArduSub SITL进程意外退出"
            cleanup
        fi
    done
}

# 检查参数
case "${1:-}" in
    --help|-h)
        echo "AUV仿真环境启动脚本 (无GUI版本)"
        echo ""
        echo "用法: $0 [选项]"
        echo ""
        echo "选项:"
        echo "  --help, -h     显示此帮助信息"
        echo "  --status       显示当前状态"
        echo "  --stop         停止仿真"
        echo ""
        exit 0
        ;;
    --status)
        show_status
        exit 0
        ;;
    --stop)
        cleanup
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
