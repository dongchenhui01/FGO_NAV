# AUV仿真环境搭建指南

## 项目目标

为验证自定义AUV算法而搭建的专业仿真环境，用于复现您在现实环境中采集的DVL、IMU和GPS数据。

### 核心需求
1. **验证算法精度**: 通过Gazebo物理仿真提供Ground Truth来衡量算法准确性
2. **评估算法性能**: 测量算法的计算效率、实时性和资源占用
3. **实现闭环测试**: 将算法输出反馈给控制器，观察整个系统行为

## 技术架构

**黄金组合**: ArduSub SITL + Gazebo + ROS 2

- **ArduSub SITL**: 软件在环仿真，模拟飞控硬件
- **Gazebo**: 3D物理仿真引擎，提供逼真水下环境和Ground Truth
- **ROS 2**: 连接SITL、Gazebo和自定义算法的通信框架
- **MAVROS**: ROS与ArduPilot (MAVLink协议)之间的桥梁

## 系统要求

### 操作系统
- **推荐**: Ubuntu 22.04 LTS
- **当前系统**: Ubuntu 20.04 (使用ROS 2 Galactic)

### 硬件配置
- CPU: 四核或更高
- 内存: 至少8GB RAM (推荐16GB)
- 硬盘: 至少25GB可用空间
- 显卡: 带有独立驱动的NVIDIA或AMD显卡

## 安装步骤

### 第1步: 安装核心依赖
```bash
sudo apt update
sudo apt upgrade -y
sudo apt install git python3-pip python3-venv g++ -y
```

### 第2步: 安装ArduPilot SITL
```bash
# 克隆ArduPilot代码库
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot

# 初始化子模块
git submodule update --init --recursive

# 安装SITL依赖
Tools/environment_install/install-prereqs-ubuntu.sh -y

# 添加到环境变量
echo "export PATH=$PATH:$HOME/auv_simulation_env/ardupilot/Tools/autotest" >> ~/.bashrc
source ~/.bashrc
```

### 第3步: 安装MAVROS (适配ROS 2 Galactic)
```bash
# 安装MAVROS
sudo apt install ros-galactic-mavros ros-galactic-mavros-extras -y

# 安装GeographicLib数据集
sudo /opt/ros/galactic/share/mavros/install_geographiclib_datasets.sh
```

### 第4步: 创建ROS工作空间
```bash
# 创建工作空间
mkdir -p ~/auv_colcon_ws/src
cd ~/auv_colcon_ws/src

# 克隆仿真插件
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
git clone https://github.com/bluerobotics/bluerov_ros_playground.git

# 编译工作空间
cd ~/auv_colcon_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build

# 添加到环境变量
echo "source ~/auv_colcon_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 目录结构

```
auv_simulation_env/
├── README.md                    # 本文件
├── INSTALL_GUIDE.md            # 详细安装指南
├── setup_environment.sh        # 自动安装脚本
├── launch_simulation.sh        # 启动仿真脚本
├── ardupilot/                  # ArduPilot SITL源码
├── config/                     # 配置文件
│   ├── sitl_params.txt         # SITL参数配置
│   └── gazebo_worlds/          # Gazebo世界文件
├── data/                       # 数据文件
│   ├── real_data/              # 您的真实采集数据
│   └── simulation_data/        # 仿真生成数据
├── scripts/                    # 工具脚本
│   ├── data_replay.py          # 数据回放脚本
│   └── algorithm_test.py       # 算法测试脚本
└── ros_packages/               # 自定义ROS包
    ├── auv_data_replay/        # 数据回放包
    ├── auv_algorithm_test/     # 算法测试包
    └── auv_ground_truth/       # Ground Truth对比包
```

## 使用流程

### 1. 启动仿真环境
```bash
cd auv_simulation_env
./launch_simulation.sh
```

### 2. 加载真实数据
```bash
# 将您的DVL、IMU、GPS数据放入data/real_data/
# 运行数据回放节点
ros2 run auv_data_replay data_replay_node
```

### 3. 运行算法测试
```bash
# 启动您的算法节点
ros2 run auv_algorithm_test your_algorithm_node

# 启动Ground Truth对比
ros2 run auv_ground_truth comparison_node
```

## 验证步骤

1. **环境验证**: 确保Gazebo中的BlueROV2模型能正常显示和控制
2. **数据验证**: 确保能正确读取和回放您的真实数据
3. **算法验证**: 对比算法输出与Ground Truth的精度
4. **性能验证**: 测量算法的实时性和资源占用

## 下一步

1. 运行自动安装脚本
2. 验证基础环境
3. 集成您的真实数据
4. 开发算法测试节点
5. 进行闭环仿真测试
