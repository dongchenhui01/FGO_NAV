#!/bin/bash

# AUV仿真环境停止脚本

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_info "停止AUV仿真环境..."

# 停止数据回放
if [ -f /tmp/auv_replay.pid ]; then
    PID=$(cat /tmp/auv_replay.pid)
    if kill -0 $PID 2>/dev/null; then
        kill $PID
        log_info "已停止数据回放 (PID: $PID)"
    fi
    rm -f /tmp/auv_replay.pid
fi

# 停止MAVROS
if [ -f /tmp/auv_mavros.pid ]; then
    PID=$(cat /tmp/auv_mavros.pid)
    if kill -0 $PID 2>/dev/null; then
        kill $PID
        log_info "已停止MAVROS (PID: $PID)"
    fi
    rm -f /tmp/auv_mavros.pid
fi

# 停止ArduSub SITL
if [ -f /tmp/auv_sitl.pid ]; then
    PID=$(cat /tmp/auv_sitl.pid)
    if kill -0 $PID 2>/dev/null; then
        kill $PID
        log_info "已停止ArduSub SITL (PID: $PID)"
    fi
    rm -f /tmp/auv_sitl.pid
fi

# 清理其他相关进程
pkill -f "sim_vehicle.py" 2>/dev/null || true
pkill -f "mavros" 2>/dev/null || true
pkill -f "data_replay.py" 2>/dev/null || true

log_success "AUV仿真环境已完全停止"
