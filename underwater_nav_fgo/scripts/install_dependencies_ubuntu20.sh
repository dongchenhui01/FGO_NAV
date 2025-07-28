#!/bin/bash

# 水下导航系统依赖安装脚本 - Ubuntu 20.04 专用版本
# 适用于Ubuntu 20.04 + ROS2 Galactic

set -e

echo "=== 水下导航系统依赖安装脚本 (Ubuntu 20.04) ==="
echo "系统要求: Ubuntu 20.04 + ROS2 Galactic"
echo

# 检查系统版本
if ! grep -q "20.04" /etc/os-release; then
    echo "警告: 此脚本针对Ubuntu 20.04优化，其他版本可能需要调整"
fi

# 检查是否为root用户
if [ "$EUID" -eq 0 ]; then
    echo "错误: 请不要以root用户运行此脚本"
    exit 1
fi

# 更新包管理器
echo "更新包管理器..."
sudo apt update
sudo apt upgrade -y

# 安装基础依赖
echo "安装基础依赖..."
sudo apt install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    build-essential \
    cmake \
    git \
    wget \
    python3-pip \
    python3-dev \
    python3-setuptools \
    python3-wheel

# 安装ROS2 Galactic
echo "检查ROS2安装状态..."
if ! command -v ros2 &> /dev/null; then
    echo "安装ROS2 Galactic..."
    
    # 添加ROS2 apt仓库
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    # 更新包列表
    sudo apt update
    
    # 安装ROS2 Galactic桌面版
    sudo apt install -y ros-galactic-desktop
    
    # 安装开发工具
    sudo apt install -y \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        ros-galactic-tf2-tools \
        ros-galactic-robot-state-publisher \
        ros-galactic-joint-state-publisher \
        ros-galactic-xacro \
        ros-galactic-rviz2
    
    echo "ROS2 Galactic安装完成"
else
    echo "ROS2已安装，跳过"
fi

# 初始化rosdep
if [ ! -d "/etc/ros/rosdep" ]; then
    echo "初始化rosdep..."
    sudo rosdep init
fi

echo "更新rosdep..."
rosdep update

# 安装数学和科学计算库
echo "安装数学和科学计算库..."
sudo apt install -y \
    libeigen3-dev \
    libboost-all-dev \
    libyaml-cpp-dev \
    libopencv-dev \
    libblas-dev \
    liblapack-dev \
    libsuitesparse-dev

# 安装Python科学计算包
echo "安装Python科学计算包..."
pip3 install --user \
    numpy \
    scipy \
    matplotlib \
    pandas \
    pyyaml

# 检查并安装GTSAM
echo "检查GTSAM安装状态..."
if ! pkg-config --exists gtsam 2>/dev/null && ! ldconfig -p | grep -q libgtsam; then
    echo "安装GTSAM (Georgia Tech Smoothing and Mapping)..."
    
    # 创建临时目录
    TEMP_DIR=$(mktemp -d)
    cd $TEMP_DIR
    
    # 下载GTSAM
    echo "下载GTSAM源码..."
    git clone https://github.com/borglab/gtsam.git
    cd gtsam
    
    # 切换到适合Ubuntu 20.04的版本
    git checkout 4.1.1
    
    # 创建构建目录
    mkdir build
    cd build
    
    # 配置CMake (针对Ubuntu 20.04优化)
    echo "配置GTSAM构建..."
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DGTSAM_BUILD_PYTHON=OFF \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_EXAMPLES=OFF \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
        -DGTSAM_ALLOW_DEPRECATED_SINCE_V42=OFF \
        -DCMAKE_INSTALL_PREFIX=/usr/local
    
    # 编译 (使用适当的并行数)
    NPROC=$(nproc)
    if [ $NPROC -gt 4 ]; then
        NPROC=4  # 限制并行数避免内存不足
    fi
    
    echo "编译GTSAM (使用 $NPROC 个并行进程，这可能需要几分钟)..."
    make -j$NPROC
    
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
echo "验证GTSAM安装..."
if ldconfig -p | grep -q libgtsam; then
    echo "✓ GTSAM库文件已正确安装"
else
    echo "✗ GTSAM库文件未找到，可能安装失败"
    exit 1
fi

# 创建工作空间目录结构
WORKSPACE_DIR="$HOME/underwater_nav_ws"
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "创建工作空间: $WORKSPACE_DIR"
    mkdir -p $WORKSPACE_DIR/src
fi

# 设置环境变量
echo "设置环境变量..."
BASHRC_FILE="$HOME/.bashrc"

# 添加ROS2环境设置
if ! grep -q "source /opt/ros/galactic/setup.bash" $BASHRC_FILE; then
    echo "source /opt/ros/galactic/setup.bash" >> $BASHRC_FILE
    echo "添加ROS2环境设置到 ~/.bashrc"
fi

# 添加工作空间设置
if ! grep -q "source $WORKSPACE_DIR/install/setup.bash" $BASHRC_FILE; then
    echo "source $WORKSPACE_DIR/install/setup.bash" >> $BASHRC_FILE
    echo "添加工作空间设置到 ~/.bashrc"
fi

# 添加Python用户包路径
if ! grep -q 'export PATH="$HOME/.local/bin:$PATH"' $BASHRC_FILE; then
    echo 'export PATH="$HOME/.local/bin:$PATH"' >> $BASHRC_FILE
    echo "添加Python用户包路径到 ~/.bashrc"
fi

# 创建便捷脚本
echo "创建便捷脚本..."

cat > $WORKSPACE_DIR/build.sh << 'EOF'
#!/bin/bash
cd $(dirname $0)
source /opt/ros/galactic/setup.bash
echo "开始编译项目..."
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
if [ $? -eq 0 ]; then
    source install/setup.bash
    echo "✓ 构建完成！"
else
    echo "✗ 构建失败！"
    exit 1
fi
EOF

chmod +x $WORKSPACE_DIR/build.sh

cat > $WORKSPACE_DIR/clean.sh << 'EOF'
#!/bin/bash
cd $(dirname $0)
rm -rf build install log
echo "✓ 清理完成！"
EOF

chmod +x $WORKSPACE_DIR/clean.sh

cat > $WORKSPACE_DIR/setup_env.sh << 'EOF'
#!/bin/bash
source /opt/ros/galactic/setup.bash
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "✓ 环境已设置"
else
    echo "⚠ 请先编译项目: ./build.sh"
fi
EOF

chmod +x $WORKSPACE_DIR/setup_env.sh

echo
echo "=== 安装验证 ==="
echo "检查关键依赖..."

# 检查编译器
echo -n "GCC: "
gcc --version | head -1

echo -n "G++: "
g++ --version | head -1

# 检查CMake
echo -n "CMake: "
cmake --version | head -1

# 检查Python
echo -n "Python3: "
python3 --version

# 检查ROS2
echo -n "ROS2: "
source /opt/ros/galactic/setup.bash
ros2 --version 2>/dev/null || echo "需要重新打开终端"

# 检查Eigen
if [ -f "/usr/include/eigen3/Eigen/Core" ]; then
    echo "✓ Eigen3: 已安装"
else
    echo "✗ Eigen3: 未找到"
fi

# 检查Boost
if pkg-config --exists boost 2>/dev/null; then
    echo "✓ Boost: $(pkg-config --modversion boost)"
else
    echo "✓ Boost: 已安装 (无版本信息)"
fi

# 检查OpenCV
if pkg-config --exists opencv4 2>/dev/null; then
    echo "✓ OpenCV: $(pkg-config --modversion opencv4)"
elif pkg-config --exists opencv 2>/dev/null; then
    echo "✓ OpenCV: $(pkg-config --modversion opencv)"
else
    echo "✓ OpenCV: 已安装"
fi

# 检查GTSAM
if ldconfig -p | grep -q libgtsam; then
    echo "✓ GTSAM: 已安装"
else
    echo "✗ GTSAM: 未找到"
fi

# 检查Python包
echo "检查Python包..."
python3 -c "import numpy; print('✓ NumPy:', numpy.__version__)" 2>/dev/null || echo "✗ NumPy: 未安装"
python3 -c "import scipy; print('✓ SciPy:', scipy.__version__)" 2>/dev/null || echo "✗ SciPy: 未安装"
python3 -c "import matplotlib; print('✓ Matplotlib:', matplotlib.__version__)" 2>/dev/null || echo "✗ Matplotlib: 未安装"
python3 -c "import pandas; print('✓ Pandas:', pandas.__version__)" 2>/dev/null || echo "✗ Pandas: 未安装"

echo
echo "=== 安装完成 ==="
echo "工作空间位置: $WORKSPACE_DIR"
echo
echo "下一步操作:"
echo "1. 重新打开终端或运行: source ~/.bashrc"
echo "2. 将项目代码复制到工作空间: cp -r /home/dongchenhui/underwater_nav_fgo $WORKSPACE_DIR/src/"
echo "3. 编译项目: cd $WORKSPACE_DIR && ./build.sh"
echo "4. 生成测试数据: python3 src/underwater_nav_fgo/scripts/generate_sample_data.py -o test_data.csv"
echo "5. 运行系统: ros2 launch underwater_nav_fgo navigation.launch.py data_file:=test_data.csv"
echo
echo "便捷脚本:"
echo "- 编译项目: cd $WORKSPACE_DIR && ./build.sh"
echo "- 清理构建: cd $WORKSPACE_DIR && ./clean.sh"
echo "- 设置环境: cd $WORKSPACE_DIR && source ./setup_env.sh"
echo
