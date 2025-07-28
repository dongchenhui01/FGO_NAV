# AUV仿真环境详细安装指南

## 系统检查

首先检查当前系统环境：

```bash
# 检查Ubuntu版本
lsb_release -a

# 检查ROS版本
echo $ROS_DISTRO

# 检查可用空间
df -h

# 检查内存
free -h
```

## 适配当前环境 (Ubuntu 20.04 + ROS 2 Galactic)

由于您当前使用的是Ubuntu 20.04和ROS 2 Galactic，我们需要适配相应的版本。

### 第1步: 更新系统和安装基础依赖

```bash
# 更新系统
sudo apt update && sudo apt upgrade -y

# 安装基础工具
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
    software-properties-common
```

### 第2步: 安装ArduPilot SITL

```bash
# 进入仿真环境目录
cd ~/LSTM_TimeFGO_nav/auv_simulation_env

# 克隆ArduPilot
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot

# 切换到稳定分支
git checkout Copter-4.3

# 初始化子模块
git submodule update --init --recursive

# 安装依赖
Tools/environment_install/install-prereqs-ubuntu.sh -y

# 重新加载环境
source ~/.bashrc

# 添加SITL路径到环境变量
echo "export PATH=\$PATH:\$HOME/LSTM_TimeFGO_nav/auv_simulation_env/ardupilot/Tools/autotest" >> ~/.bashrc
source ~/.bashrc
```

### 第3步: 验证ArduPilot SITL安装

```bash
# 测试SITL是否正常工作
cd ~/LSTM_TimeFGO_nav/auv_simulation_env/ardupilot
sim_vehicle.py --help

# 如果成功，应该看到帮助信息
```

### 第4步: 安装Gazebo (适配Ubuntu 20.04)

```bash
# 安装Gazebo 11 (Ubuntu 20.04默认版本)
sudo apt install -y gazebo11 libgazebo11-dev

# 安装Gazebo ROS包
sudo apt install -y ros-galactic-gazebo-ros-pkgs ros-galactic-gazebo-ros

# 验证Gazebo安装
gazebo --version
```

### 第5步: 安装MAVROS

```bash
# 安装MAVROS和相关包
sudo apt install -y \
    ros-galactic-mavros \
    ros-galactic-mavros-extras \
    ros-galactic-mavros-msgs

# 安装GeographicLib数据集
sudo /opt/ros/galactic/share/mavros/install_geographiclib_datasets.sh
```

### 第6步: 创建ROS工作空间

```bash
# 创建工作空间
mkdir -p ~/auv_colcon_ws/src
cd ~/auv_colcon_ws/src

# 克隆必要的包
git clone https://github.com/ArduPilot/ardupilot_gazebo.git

# 由于bluerov_ros_playground可能不兼容ROS 2 Galactic，我们创建自己的包
cd ~/auv_colcon_ws/src
ros2 pkg create --build-type ament_python auv_simulation
ros2 pkg create --build-type ament_python auv_data_replay
ros2 pkg create --build-type ament_python auv_algorithm_test

# 安装依赖
cd ~/auv_colcon_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 编译工作空间
colcon build

# 添加到环境变量
echo "source ~/auv_colcon_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 常见问题解决

### 问题1: SITL编译失败
```bash
# 如果遇到Python版本问题
sudo apt install python3.8-dev python3.8-venv

# 如果遇到权限问题
sudo chown -R $USER:$USER ~/LSTM_TimeFGO_nav/auv_simulation_env/ardupilot
```

### 问题2: Gazebo启动失败
```bash
# 检查显卡驱动
nvidia-smi  # 如果是NVIDIA显卡

# 安装额外的Gazebo插件
sudo apt install -y gazebo11-plugin-base
```

### 问题3: MAVROS连接失败
```bash
# 检查MAVROS服务
ros2 service list | grep mavros

# 检查MAVLink连接
ros2 topic echo /mavros/state
```

## 验证安装

### 1. 测试ArduPilot SITL
```bash
cd ~/LSTM_TimeFGO_nav/auv_simulation_env/ardupilot
sim_vehicle.py -v ArduSub --console --map
```

### 2. 测试Gazebo
```bash
gazebo --verbose
```

### 3. 测试ROS工作空间
```bash
source ~/auv_colcon_ws/install/setup.bash
ros2 pkg list | grep auv
```

### 4. 测试MAVROS
```bash
# 在一个终端启动SITL
sim_vehicle.py -v ArduSub --console

# 在另一个终端启动MAVROS
ros2 launch mavros apm.launch fcu_url:=udp://:14550@
```

## 下一步

安装完成后，您可以：

1. 运行基础仿真测试
2. 集成您的真实数据
3. 开发数据回放节点
4. 创建算法测试框架
5. 进行Ground Truth对比

## 性能优化建议

1. **内存优化**: 如果内存不足，可以减少Gazebo的渲染质量
2. **CPU优化**: 使用多线程编译 `colcon build --parallel-workers 4`
3. **显卡优化**: 确保使用硬件加速渲染

## 故障排除

如果遇到问题，请检查：
1. 所有环境变量是否正确设置
2. ROS 2 Galactic是否正确source
3. 网络连接是否正常（下载依赖时）
4. 磁盘空间是否充足
5. 权限设置是否正确
