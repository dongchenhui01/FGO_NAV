#!/bin/bash

# AUV仿真环境自动安装脚本
# 适配Ubuntu 20.04 + ROS 2 Galactic

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

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

# 检查系统
check_system() {
    log_info "检查系统环境..."
    
    # 检查Ubuntu版本
    if ! lsb_release -a 2>/dev/null | grep -q "20.04"; then
        log_warning "当前不是Ubuntu 20.04，可能存在兼容性问题"
    fi
    
    # 检查ROS版本
    if [ -z "$ROS_DISTRO" ]; then
        log_error "未检测到ROS环境，请先安装ROS 2 Galactic"
        exit 1
    elif [ "$ROS_DISTRO" != "galactic" ]; then
        log_warning "当前ROS版本是$ROS_DISTRO，推荐使用Galactic"
    fi
    
    # 检查磁盘空间
    available_space=$(df / | awk 'NR==2 {print $4}')
    if [ $available_space -lt 26214400 ]; then  # 25GB in KB
        log_error "磁盘空间不足，至少需要25GB可用空间"
        exit 1
    fi
    
    log_success "系统检查通过"
}

# 安装基础依赖
install_dependencies() {
    log_info "安装基础依赖..."
    
    sudo apt update
    sudo apt install -y \
        git \
        python3-pip \
        python3-venv \
        python3-dev \
        g++ \
        cmake \
        build-essential \
        curl \
        wget \
        unzip \
        software-properties-common \
        python3-setuptools \
        python3-wheel
    
    log_success "基础依赖安装完成"
}

# 安装ArduPilot SITL
install_ardupilot() {
    log_info "安装ArduPilot SITL..."
    
    # 获取当前脚本所在目录
    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
    
    cd "$SCRIPT_DIR"
    
    if [ ! -d "ardupilot" ]; then
        log_info "克隆ArduPilot代码库..."
        git clone https://github.com/ArduPilot/ardupilot.git
    else
        log_info "ArduPilot目录已存在，跳过克隆"
    fi
    
    cd ardupilot
    
    # 切换到稳定分支
    git checkout Copter-4.3
    
    # 初始化子模块
    log_info "初始化子模块..."
    git submodule update --init --recursive
    
    # 安装依赖
    log_info "安装ArduPilot依赖..."
    Tools/environment_install/install-prereqs-ubuntu.sh -y
    
    # 添加到PATH
    ARDUPILOT_PATH="$SCRIPT_DIR/ardupilot/Tools/autotest"
    if ! grep -q "$ARDUPILOT_PATH" ~/.bashrc; then
        echo "export PATH=\$PATH:$ARDUPILOT_PATH" >> ~/.bashrc
        log_info "已添加ArduPilot路径到~/.bashrc"
    fi
    
    log_success "ArduPilot SITL安装完成"
}

# 安装Gazebo
install_gazebo() {
    log_info "安装Gazebo..."
    
    # 安装Gazebo 11
    sudo apt install -y gazebo11 libgazebo11-dev
    
    # 安装Gazebo ROS包
    sudo apt install -y ros-galactic-gazebo-ros-pkgs ros-galactic-gazebo-ros
    
    log_success "Gazebo安装完成"
}

# 安装MAVROS
install_mavros() {
    log_info "安装MAVROS..."
    
    # 安装MAVROS包
    sudo apt install -y \
        ros-galactic-mavros \
        ros-galactic-mavros-extras \
        ros-galactic-mavros-msgs
    
    # 安装GeographicLib数据集
    log_info "安装GeographicLib数据集..."
    sudo /opt/ros/galactic/share/mavros/install_geographiclib_datasets.sh
    
    log_success "MAVROS安装完成"
}

# 创建ROS工作空间
create_workspace() {
    log_info "创建ROS工作空间..."
    
    # 创建工作空间目录
    mkdir -p ~/auv_colcon_ws/src
    cd ~/auv_colcon_ws/src
    
    # 克隆ArduPilot Gazebo插件
    if [ ! -d "ardupilot_gazebo" ]; then
        log_info "克隆ArduPilot Gazebo插件..."
        git clone https://github.com/ArduPilot/ardupilot_gazebo.git
    fi
    
    # 创建自定义包
    log_info "创建自定义ROS包..."
    if [ ! -d "auv_simulation" ]; then
        ros2 pkg create --build-type ament_python auv_simulation
    fi
    
    if [ ! -d "auv_data_replay" ]; then
        ros2 pkg create --build-type ament_python auv_data_replay
    fi
    
    if [ ! -d "auv_algorithm_test" ]; then
        ros2 pkg create --build-type ament_python auv_algorithm_test
    fi
    
    # 安装依赖
    cd ~/auv_colcon_ws
    log_info "安装ROS依赖..."
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    
    # 编译工作空间
    log_info "编译ROS工作空间..."
    colcon build
    
    # 添加到bashrc
    WORKSPACE_SETUP="source ~/auv_colcon_ws/install/setup.bash"
    if ! grep -q "$WORKSPACE_SETUP" ~/.bashrc; then
        echo "$WORKSPACE_SETUP" >> ~/.bashrc
        log_info "已添加工作空间setup到~/.bashrc"
    fi
    
    log_success "ROS工作空间创建完成"
}

# 创建目录结构
create_directories() {
    log_info "创建项目目录结构..."
    
    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
    cd "$SCRIPT_DIR"
    
    # 创建目录
    mkdir -p config/gazebo_worlds
    mkdir -p data/real_data
    mkdir -p data/simulation_data
    mkdir -p scripts
    mkdir -p ros_packages
    
    log_success "目录结构创建完成"
}

# 验证安装
verify_installation() {
    log_info "验证安装..."
    
    # 重新加载环境
    source ~/.bashrc
    
    # 检查ArduPilot
    if command -v sim_vehicle.py &> /dev/null; then
        log_success "ArduPilot SITL安装验证通过"
    else
        log_error "ArduPilot SITL验证失败"
        return 1
    fi
    
    # 检查Gazebo
    if command -v gazebo &> /dev/null; then
        log_success "Gazebo安装验证通过"
    else
        log_error "Gazebo验证失败"
        return 1
    fi
    
    # 检查ROS工作空间
    if [ -f ~/auv_colcon_ws/install/setup.bash ]; then
        log_success "ROS工作空间验证通过"
    else
        log_error "ROS工作空间验证失败"
        return 1
    fi
    
    log_success "所有组件验证通过！"
}

# 主函数
main() {
    log_info "开始安装AUV仿真环境..."
    log_info "这可能需要20-30分钟，请耐心等待..."
    
    check_system
    install_dependencies
    install_ardupilot
    install_gazebo
    install_mavros
    create_workspace
    create_directories
    verify_installation
    
    log_success "AUV仿真环境安装完成！"
    log_info "请运行 'source ~/.bashrc' 或重新打开终端来加载环境变量"
    log_info "然后可以运行 './launch_simulation.sh' 来启动仿真"
}

# 运行主函数
main "$@"
