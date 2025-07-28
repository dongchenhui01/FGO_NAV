#!/bin/bash

# 水下导航系统依赖安装脚本
# 适用于Ubuntu 22.04 + ROS2 Humble

set -e

echo "=== 水下导航系统依赖安装脚本 ==="
echo "系统要求: Ubuntu 22.04 + ROS2 Humble"
echo

# 检查系统版本
if ! grep -q "22.04" /etc/os-release; then
    echo "警告: 此脚本针对Ubuntu 22.04优化，其他版本可能需要调整"
fi

# 检查ROS2安装
if ! command -v ros2 &> /dev/null; then
    echo "错误: 未检测到ROS2安装，请先安装ROS2 Humble"
    exit 1
fi

echo "检测到ROS2安装，继续..."

# 更新包管理器
echo "更新包管理器..."
sudo apt update

# 安装基础依赖
echo "安装基础依赖..."
sudo apt install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

# 安装数学库
echo "安装数学和线性代数库..."
sudo apt install -y \
    libeigen3-dev \
    libboost-all-dev \
    libyaml-cpp-dev \
    libopencv-dev

# 安装ROS2相关包
echo "安装ROS2相关包..."
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-tf2-tools \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-rviz2

# 检查GTSAM是否已安装
if ! pkg-config --exists gtsam; then
    echo "安装GTSAM (Georgia Tech Smoothing and Mapping)..."
    
    # 创建临时目录
    TEMP_DIR=$(mktemp -d)
    cd $TEMP_DIR
    
    # 下载GTSAM
    echo "下载GTSAM源码..."
    git clone https://github.com/borglab/gtsam.git
    cd gtsam
    
    # 切换到稳定版本
    git checkout 4.2.0
    
    # 创建构建目录
    mkdir build
    cd build
    
    # 配置CMake
    echo "配置GTSAM构建..."
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DGTSAM_BUILD_PYTHON=OFF \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_EXAMPLES=OFF \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
    
    # 编译 (使用所有可用核心)
    echo "编译GTSAM (这可能需要几分钟)..."
    make -j$(nproc)
    
    # 安装
    echo "安装GTSAM..."
    sudo make install
    
    # 更新链接器缓存
    sudo ldconfig
    
    # 清理临时文件
    cd /
    rm -rf $TEMP_DIR
    
    echo "GTSAM安装完成"
else
    echo "GTSAM已安装，跳过"
fi

# 验证GTSAM安装
if pkg-config --exists gtsam; then
    GTSAM_VERSION=$(pkg-config --modversion gtsam)
    echo "GTSAM版本: $GTSAM_VERSION"
else
    echo "错误: GTSAM安装失败"
    exit 1
fi

# 安装Python依赖
echo "安装Python依赖..."
pip3 install --user \
    numpy \
    matplotlib \
    pandas \
    scipy

# 初始化rosdep (如果尚未初始化)
if [ ! -d "/etc/ros/rosdep" ]; then
    echo "初始化rosdep..."
    sudo rosdep init
fi

echo "更新rosdep..."
rosdep update

# 创建工作空间目录结构
WORKSPACE_DIR="$HOME/underwater_nav_ws"
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "创建工作空间: $WORKSPACE_DIR"
    mkdir -p $WORKSPACE_DIR/src
fi

# 设置环境变量
echo "设置环境变量..."
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

if ! grep -q "source $WORKSPACE_DIR/install/setup.bash" ~/.bashrc; then
    echo "source $WORKSPACE_DIR/install/setup.bash" >> ~/.bashrc
fi

# 创建便捷脚本
cat > $WORKSPACE_DIR/build.sh << 'EOF'
#!/bin/bash
cd $(dirname $0)
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
echo "构建完成！"
EOF

chmod +x $WORKSPACE_DIR/build.sh

cat > $WORKSPACE_DIR/clean.sh << 'EOF'
#!/bin/bash
cd $(dirname $0)
rm -rf build install log
echo "清理完成！"
EOF

chmod +x $WORKSPACE_DIR/clean.sh

echo
echo "=== 安装完成 ==="
echo "工作空间位置: $WORKSPACE_DIR"
echo
echo "下一步操作:"
echo "1. 将项目代码复制到 $WORKSPACE_DIR/src/"
echo "2. 运行: cd $WORKSPACE_DIR && ./build.sh"
echo "3. 重新打开终端或运行: source ~/.bashrc"
echo
