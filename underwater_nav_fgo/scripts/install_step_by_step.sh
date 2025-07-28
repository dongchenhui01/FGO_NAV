#!/bin/bash

# 分步骤安装依赖脚本
# 避免长时间卡住，可以分步执行

set -e

echo "=== 分步骤安装水下导航系统依赖 ==="
echo

# 检查是否为root用户
if [ "$EUID" -eq 0 ]; then
    echo "错误: 请不要以root用户运行此脚本"
    exit 1
fi

# 函数：安装Python包
install_python_packages() {
    echo "步骤1: 安装Python包..."
    pip3 install --user scipy
    echo "✓ Python包安装完成"
}

# 函数：安装系统包
install_system_packages() {
    echo "步骤2: 安装系统包..."
    echo "需要sudo权限来安装系统包"
    
    sudo apt update
    sudo apt install -y \
        libyaml-cpp-dev \
        libopencv-dev \
        libsuitesparse-dev \
        libblas-dev \
        liblapack-dev
    
    echo "✓ 系统包安装完成"
}

# 函数：安装ROS2
install_ros2() {
    echo "步骤3: 安装ROS2 Galactic..."
    echo "这可能需要几分钟..."
    
    # 添加ROS2 apt仓库
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    sudo apt update
    sudo apt install -y ros-galactic-desktop
    
    # 安装开发工具
    sudo apt install -y \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool
    
    # 初始化rosdep
    if [ ! -d "/etc/ros/rosdep" ]; then
        sudo rosdep init
    fi
    rosdep update
    
    echo "✓ ROS2安装完成"
}

# 函数：安装GTSAM
install_gtsam() {
    echo "步骤4: 安装GTSAM..."
    echo "这可能需要10-20分钟，请耐心等待..."
    
    # 创建临时目录
    TEMP_DIR=$(mktemp -d)
    cd $TEMP_DIR
    
    # 下载GTSAM
    git clone https://github.com/borglab/gtsam.git
    cd gtsam
    git checkout 4.1.1
    
    # 创建构建目录
    mkdir build
    cd build
    
    # 配置CMake
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DGTSAM_BUILD_PYTHON=OFF \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_EXAMPLES=OFF \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
        -DCMAKE_INSTALL_PREFIX=/usr/local
    
    # 编译 (限制并行数)
    NPROC=$(nproc)
    if [ $NPROC -gt 2 ]; then
        NPROC=2  # 限制并行数避免内存不足
    fi
    
    make -j$NPROC
    sudo make install
    sudo ldconfig
    
    # 清理
    cd /
    rm -rf $TEMP_DIR
    
    echo "✓ GTSAM安装完成"
}

# 函数：设置环境
setup_environment() {
    echo "步骤5: 设置环境..."
    
    # 创建工作空间
    WORKSPACE_DIR="$HOME/underwater_nav_ws"
    mkdir -p $WORKSPACE_DIR/src
    
    # 设置环境变量
    BASHRC_FILE="$HOME/.bashrc"
    
    if ! grep -q "source /opt/ros/galactic/setup.bash" $BASHRC_FILE; then
        echo "source /opt/ros/galactic/setup.bash" >> $BASHRC_FILE
    fi
    
    if ! grep -q "source $WORKSPACE_DIR/install/setup.bash" $BASHRC_FILE; then
        echo "source $WORKSPACE_DIR/install/setup.bash" >> $BASHRC_FILE
    fi
    
    if ! grep -q 'export PATH="$HOME/.local/bin:$PATH"' $BASHRC_FILE; then
        echo 'export PATH="$HOME/.local/bin:$PATH"' >> $BASHRC_FILE
    fi
    
    echo "✓ 环境设置完成"
}

# 主菜单
show_menu() {
    echo
    echo "请选择要执行的步骤:"
    echo "1) 安装Python包 (快速)"
    echo "2) 安装系统包 (需要sudo)"
    echo "3) 安装ROS2 (需要sudo，较慢)"
    echo "4) 安装GTSAM (需要sudo，很慢)"
    echo "5) 设置环境"
    echo "6) 全部安装 (自动执行所有步骤)"
    echo "7) 检查依赖状态"
    echo "0) 退出"
    echo
}

# 主循环
while true; do
    show_menu
    read -p "请输入选择 (0-7): " choice
    
    case $choice in
        1)
            install_python_packages
            ;;
        2)
            install_system_packages
            ;;
        3)
            install_ros2
            ;;
        4)
            install_gtsam
            ;;
        5)
            setup_environment
            ;;
        6)
            echo "开始全部安装..."
            install_python_packages
            install_system_packages
            install_ros2
            install_gtsam
            setup_environment
            echo "✓ 全部安装完成！"
            ;;
        7)
            /home/dongchenhui/underwater_nav_fgo/scripts/check_dependencies.sh
            ;;
        0)
            echo "退出安装程序"
            exit 0
            ;;
        *)
            echo "无效选择，请重新输入"
            ;;
    esac
    
    echo
    read -p "按Enter键继续..."
done
