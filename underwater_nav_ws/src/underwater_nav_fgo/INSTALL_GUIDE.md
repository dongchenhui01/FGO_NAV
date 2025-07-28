# 水下导航系统安装指南 (Ubuntu 20.04)

## 当前系统状态

✅ **已安装的依赖:**
- GCC/G++ 编译器
- CMake 3.16
- Python3 + Pip3
- Eigen3 开发包
- Boost 开发包
- NumPy, Matplotlib, Pandas, PyYAML
- SciPy (刚刚安装)

❌ **需要安装的依赖:**
- YAML-CPP 开发包
- OpenCV 开发包
- ROS2 Galactic
- GTSAM 库

## 手动安装步骤

### 步骤1: 安装系统包 (需要sudo权限)

```bash
# 更新包管理器
sudo apt update

# 安装缺少的系统包
sudo apt install -y \
    libyaml-cpp-dev \
    libopencv-dev \
    libsuitesparse-dev \
    libblas-dev \
    liblapack-dev
```

### 步骤2: 安装ROS2 Galactic (需要sudo权限)

```bash
# 添加ROS2 apt仓库
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 更新包列表并安装ROS2
sudo apt update
sudo apt install -y ros-galactic-desktop

# 安装开发工具
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

# 初始化rosdep
sudo rosdep init
rosdep update
```

### 步骤3: 安装GTSAM (需要sudo权限，耗时较长)

```bash
# 创建临时目录
cd /tmp
git clone https://github.com/borglab/gtsam.git
cd gtsam
git checkout 4.1.1

# 创建构建目录
mkdir build && cd build

# 配置CMake
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_BUILD_PYTHON=OFF \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_EXAMPLES=OFF \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DCMAKE_INSTALL_PREFIX=/usr/local

# 编译 (使用2个并行进程避免内存不足)
make -j2

# 安装
sudo make install
sudo ldconfig

# 清理
cd / && rm -rf /tmp/gtsam
```

### 步骤4: 设置环境变量

```bash
# 添加到 ~/.bashrc
echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
echo "source ~/underwater_nav_ws/install/setup.bash" >> ~/.bashrc
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc

# 创建工作空间
mkdir -p ~/underwater_nav_ws/src
```

## 自动化安装选项

### 选项1: 使用分步安装脚本
```bash
/home/dongchenhui/underwater_nav_fgo/scripts/install_step_by_step.sh
```

### 选项2: 使用完整安装脚本 (如果不卡住)
```bash
sudo /home/dongchenhui/underwater_nav_fgo/scripts/install_dependencies_ubuntu20.sh
```

## 验证安装

安装完成后，运行检查脚本验证：
```bash
/home/dongchenhui/underwater_nav_fgo/scripts/check_dependencies.sh
```

## 编译和运行项目

### 1. 复制项目到工作空间
```bash
cp -r /home/dongchenhui/underwater_nav_fgo ~/underwater_nav_ws/src/
```

### 2. 编译项目
```bash
cd ~/underwater_nav_ws
source /opt/ros/galactic/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 3. 生成测试数据
```bash
cd ~/underwater_nav_ws
source install/setup.bash
python3 src/underwater_nav_fgo/scripts/generate_sample_data.py -o test_data.csv -d 60
```

### 4. 运行系统
```bash
ros2 launch underwater_nav_fgo navigation.launch.py data_file:=test_data.csv use_rviz:=true
```

## 故障排除

### 常见问题

1. **GTSAM编译失败 - 内存不足**
   - 解决方案: 使用 `make -j1` 或 `make -j2` 限制并行编译

2. **ROS2安装失败 - 网络问题**
   - 解决方案: 检查网络连接，或使用国内镜像源

3. **权限问题**
   - 解决方案: 确保使用sudo权限安装系统包

4. **环境变量未生效**
   - 解决方案: 重新打开终端或运行 `source ~/.bashrc`

### 检查命令

```bash
# 检查ROS2
ros2 --version

# 检查GTSAM
ldconfig -p | grep gtsam

# 检查Python包
python3 -c "import numpy, scipy, matplotlib, pandas; print('Python包正常')"

# 检查编译工具
gcc --version && cmake --version
```

## 预期安装时间

- 系统包安装: 2-5分钟
- ROS2安装: 10-15分钟
- GTSAM编译安装: 15-30分钟
- 总计: 约30-50分钟

## 磁盘空间要求

- ROS2: ~2GB
- GTSAM编译: ~1GB (临时)
- 项目编译: ~500MB
- 总计: ~3-4GB

---

**注意**: 如果在安装过程中遇到问题，可以分步骤执行，每步完成后运行检查脚本验证状态。
