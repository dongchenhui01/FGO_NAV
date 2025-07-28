#!/bin/bash

# 依赖检查脚本 - 不需要sudo权限
# 检查水下导航系统所需的所有依赖项

echo "=== 水下导航系统依赖检查 ==="
echo "系统信息: $(lsb_release -d | cut -f2)"
echo "用户: $(whoami)"
echo

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查函数
check_command() {
    local cmd=$1
    local name=$2
    if command -v $cmd &> /dev/null; then
        echo -e "${GREEN}✓${NC} $name: $(which $cmd)"
        return 0
    else
        echo -e "${RED}✗${NC} $name: 未安装"
        return 1
    fi
}

check_package() {
    local pkg=$1
    local name=$2
    if dpkg -l | grep -q "^ii.*$pkg"; then
        local version=$(dpkg -l | grep "^ii.*$pkg" | awk '{print $3}' | head -1)
        echo -e "${GREEN}✓${NC} $name: $version"
        return 0
    else
        echo -e "${RED}✗${NC} $name: 未安装"
        return 1
    fi
}

check_file() {
    local file=$1
    local name=$2
    if [ -f "$file" ]; then
        echo -e "${GREEN}✓${NC} $name: $file"
        return 0
    else
        echo -e "${RED}✗${NC} $name: $file 不存在"
        return 1
    fi
}

check_python_module() {
    local module=$1
    local name=$2
    if python3 -c "import $module" 2>/dev/null; then
        local version=$(python3 -c "import $module; print($module.__version__)" 2>/dev/null || echo "未知版本")
        echo -e "${GREEN}✓${NC} $name: $version"
        return 0
    else
        echo -e "${RED}✗${NC} $name: 未安装"
        return 1
    fi
}

# 开始检查
echo "1. 基础开发工具"
echo "=================="
check_command gcc "GCC编译器"
check_command g++ "G++编译器"
check_command cmake "CMake"
check_command make "Make"
check_command git "Git"
check_command python3 "Python3"
check_command pip3 "Pip3"

echo
echo "2. 系统包"
echo "=========="
check_package "build-essential" "构建工具"
check_package "libeigen3-dev" "Eigen3开发包"
check_package "libboost-all-dev" "Boost开发包"
check_package "libyaml-cpp-dev" "YAML-CPP开发包"
check_package "libopencv-dev" "OpenCV开发包"

echo
echo "3. ROS2相关"
echo "==========="
check_command ros2 "ROS2"
if command -v ros2 &> /dev/null; then
    echo "   ROS2版本: $(ros2 --version 2>/dev/null || echo '需要source环境')"
fi

# 检查ROS2包
ROS_PACKAGES=("ros-galactic-desktop" "python3-colcon-common-extensions" "python3-rosdep")
for pkg in "${ROS_PACKAGES[@]}"; do
    check_package "$pkg" "$pkg"
done

echo
echo "4. 头文件和库"
echo "============="
check_file "/usr/include/eigen3/Eigen/Core" "Eigen3头文件"
check_file "/usr/include/boost/version.hpp" "Boost头文件"
check_file "/usr/include/yaml-cpp/yaml.h" "YAML-CPP头文件"

# 检查GTSAM
echo
echo "5. GTSAM库"
echo "=========="
if ldconfig -p 2>/dev/null | grep -q libgtsam; then
    echo -e "${GREEN}✓${NC} GTSAM库: 已安装"
    GTSAM_INSTALLED=true
else
    echo -e "${RED}✗${NC} GTSAM库: 未安装"
    GTSAM_INSTALLED=false
fi

if [ -f "/usr/local/include/gtsam/config.h" ]; then
    echo -e "${GREEN}✓${NC} GTSAM头文件: /usr/local/include/gtsam/"
elif [ -f "/usr/include/gtsam/config.h" ]; then
    echo -e "${GREEN}✓${NC} GTSAM头文件: /usr/include/gtsam/"
else
    echo -e "${RED}✗${NC} GTSAM头文件: 未找到"
fi

echo
echo "6. Python模块"
echo "============="
check_python_module "numpy" "NumPy"
check_python_module "scipy" "SciPy"
check_python_module "matplotlib" "Matplotlib"
check_python_module "pandas" "Pandas"
check_python_module "yaml" "PyYAML"

echo
echo "7. 工作空间检查"
echo "==============="
WORKSPACE_DIR="$HOME/underwater_nav_ws"
if [ -d "$WORKSPACE_DIR" ]; then
    echo -e "${GREEN}✓${NC} 工作空间目录: $WORKSPACE_DIR"
    if [ -d "$WORKSPACE_DIR/src" ]; then
        echo -e "${GREEN}✓${NC} src目录: $WORKSPACE_DIR/src"
    else
        echo -e "${YELLOW}⚠${NC} src目录: 不存在，将自动创建"
    fi
else
    echo -e "${YELLOW}⚠${NC} 工作空间目录: 不存在，将自动创建"
fi

echo
echo "8. 环境变量检查"
echo "==============="
if grep -q "source /opt/ros" ~/.bashrc; then
    echo -e "${GREEN}✓${NC} ROS2环境变量: 已配置在 ~/.bashrc"
else
    echo -e "${YELLOW}⚠${NC} ROS2环境变量: 未配置"
fi

if [ -n "$ROS_DISTRO" ]; then
    echo -e "${GREEN}✓${NC} 当前ROS发行版: $ROS_DISTRO"
else
    echo -e "${YELLOW}⚠${NC} ROS环境变量: 未加载 (需要source)"
fi

echo
echo "=== 检查总结 ==="

# 统计结果
MISSING_DEPS=()

# 检查关键依赖
if ! command -v gcc &> /dev/null; then MISSING_DEPS+=("gcc"); fi
if ! command -v cmake &> /dev/null; then MISSING_DEPS+=("cmake"); fi
if ! command -v python3 &> /dev/null; then MISSING_DEPS+=("python3"); fi
if ! dpkg -l | grep -q "^ii.*libeigen3-dev"; then MISSING_DEPS+=("libeigen3-dev"); fi
if ! dpkg -l | grep -q "^ii.*libboost-all-dev"; then MISSING_DEPS+=("libboost-all-dev"); fi
if ! $GTSAM_INSTALLED; then MISSING_DEPS+=("gtsam"); fi
if ! command -v ros2 &> /dev/null; then MISSING_DEPS+=("ros2"); fi

if [ ${#MISSING_DEPS[@]} -eq 0 ]; then
    echo -e "${GREEN}✓ 所有关键依赖都已安装！${NC}"
    echo
    echo "下一步操作:"
    echo "1. 如果ROS2环境未加载，运行: source /opt/ros/galactic/setup.bash"
    echo "2. 创建工作空间: mkdir -p ~/underwater_nav_ws/src"
    echo "3. 复制项目: cp -r /home/dongchenhui/underwater_nav_fgo ~/underwater_nav_ws/src/"
    echo "4. 编译项目: cd ~/underwater_nav_ws && colcon build"
else
    echo -e "${RED}✗ 缺少以下关键依赖:${NC}"
    for dep in "${MISSING_DEPS[@]}"; do
        echo "   - $dep"
    done
    echo
    echo "建议操作:"
    echo "1. 运行安装脚本 (需要sudo权限): sudo /home/dongchenhui/underwater_nav_fgo/scripts/install_dependencies_ubuntu20.sh"
    echo "2. 或者手动安装缺少的依赖"
fi

echo
echo "详细安装说明请参考: /home/dongchenhui/underwater_nav_fgo/README.md"
