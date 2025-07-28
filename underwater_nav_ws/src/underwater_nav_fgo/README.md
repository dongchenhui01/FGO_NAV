# Underwater Navigation with IMU+DVL Factor Graph Optimization

## 项目概述

本项目基于因子图优化(Factor Graph Optimization, FGO)技术，实现IMU+DVL传感器的水下组合导航系统。系统专注于精准的位置和速度估计，适用于水下机器人、AUV等应用场景。

## 系统特点

- **高精度定位**: 融合IMU和DVL数据，实现厘米级定位精度
- **实时性能**: 基于滑动窗口的在线优化算法
- **鲁棒性强**: 处理传感器噪声和数据缺失
- **模块化设计**: 易于扩展和维护

## 技术架构

### 核心算法
- **IMU预积分**: 高效处理高频IMU数据
- **DVL速度约束**: 利用多普勒测速仪的精确速度测量
- **因子图优化**: 基于GTSAM的非线性优化
- **滑动窗口**: 保持计算效率的在线处理

### 坐标系定义
- **载体坐标系**: 前-右-下 (FRD)
- **导航坐标系**: 东-北-天 (ENU)
- **DVL坐标系**: 与载体坐标系对齐

## 环境要求

### 系统环境
- Ubuntu 22.04 LTS
- ROS2 Humble
- C++17 编译器
- CMake 3.16+

### 依赖库
```bash
# 核心依赖
sudo apt install libeigen3-dev libboost-all-dev libyaml-cpp-dev
sudo apt install libopencv-dev ros-humble-desktop

# GTSAM安装
git clone https://github.com/borglab/gtsam.git
cd gtsam
mkdir build && cd build
cmake -DGTSAM_BUILD_PYTHON=OFF ..
make -j$(nproc)
sudo make install
```

## 数据格式

### 输入数据 (CSV格式)
支持的传感器数据包括：
- **IMU数据**: 加速度计、陀螺仪、磁力计
- **DVL数据**: 三轴速度测量
- **GPS数据**: 作为参考真值用于精度评估

详细字段说明请参考数据格式文档。

### 输出结果
- **导航状态**: 位置、速度、姿态
- **协方差信息**: 估计不确定性
- **轨迹文件**: 可视化和分析用

## 快速开始

### 1. 安装依赖
```bash
# 运行依赖安装脚本
chmod +x scripts/install_dependencies.sh
./scripts/install_dependencies.sh
```

### 2. 生成示例数据
```bash
# 生成测试数据
python3 scripts/generate_sample_data.py -o data/sample_data.csv -d 300
```

### 3. 编译项目
```bash
# 创建工作空间并编译
mkdir -p ~/underwater_nav_ws/src
cp -r . ~/underwater_nav_ws/src/underwater_nav_fgo
cd ~/underwater_nav_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 4. 运行导航系统
```bash
# 启动完整的导航系统
ros2 launch underwater_nav_fgo navigation.launch.py \
    data_file:=~/underwater_nav_ws/src/underwater_nav_fgo/data/sample_data.csv \
    use_rviz:=true
```

### 5. 监控系统状态
```bash
# 查看导航状态
ros2 topic echo /navigation/state

# 查看轨迹
ros2 topic echo /navigation/trajectory

# 查看优化统计
ros2 service call /navigation/get_stats std_srvs/srv/Empty
```

## 配置参数

主要配置文件位于 `config/` 目录：
- `imu_params.yaml`: IMU参数配置
- `dvl_params.yaml`: DVL参数配置
- `optimizer_params.yaml`: 优化器参数
- `system_params.yaml`: 系统参数

## 性能指标

### 定位精度
- **水平精度**: < 0.1m (1σ)
- **垂直精度**: < 0.2m (1σ)
- **速度精度**: < 0.05m/s (1σ)

### 计算性能
- **处理频率**: 100Hz (IMU), 10Hz (DVL)
- **延迟**: < 50ms
- **CPU占用**: < 30% (单核)

## 项目结构

```
underwater_nav_fgo/
├── src/
│   ├── underwater_nav_msgs/          # ROS2消息定义
│   ├── data_preprocessing/           # 数据预处理模块
│   ├── factor_graph_optimizer/       # 因子图优化核心
│   ├── state_estimator/             # 状态估计器
│   └── visualization/               # 可视化工具
├── config/                          # 配置文件
├── launch/                          # ROS2启动文件
├── data/                           # 测试数据
├── scripts/                        # 工具脚本
└── docs/                           # 文档
```

## 算法原理

### 因子图模型
系统使用以下因子类型：

1. **IMU预积分因子**: 连接相邻关键帧的IMU测量
2. **DVL速度因子**: 约束载体坐标系下的速度
3. **磁力计因子**: 提供航向角约束
4. **先验因子**: 初始状态约束

### 状态向量
每个关键帧的状态包括：
- 位置: p ∈ ℝ³
- 速度: v ∈ ℝ³  
- 姿态: R ∈ SO(3)
- IMU偏差: b_a, b_g ∈ ℝ³

### 优化目标
最小化加权最小二乘目标函数：
```
min Σ ||h_i(x) - z_i||²_Σ_i
```

## 开发指南

### 添加新传感器
1. 在 `underwater_nav_msgs` 中定义消息类型
2. 在 `factor_graph_optimizer` 中实现对应因子
3. 更新配置文件和启动脚本

### 调试工具
- ROS2 topic监控
- GTSAM因子图可视化
- 实时性能监控

### 测试框架
```bash
# 运行单元测试
cd ~/underwater_nav_ws
colcon test --packages-select factor_graph_optimizer
colcon test-result --verbose
```

## 故障排除

### 常见问题
1. **GTSAM编译失败**: 检查Eigen版本兼容性
2. **优化收敛慢**: 调整噪声参数和初始值
3. **内存占用高**: 减少滑动窗口大小

### 性能调优
- 调整关键帧间隔
- 优化因子权重
- 使用鲁棒核函数

## 许可证

本项目采用 MIT 许可证。详见 LICENSE 文件。

## 贡献

欢迎提交Issue和Pull Request。请遵循项目的代码规范和提交指南。

## 联系方式

如有问题或建议，请通过GitHub Issues联系。

## 参考文献

1. Dellaert, F., & Kaess, M. (2017). Factor graphs for robot perception. Foundations and Trends in Robotics.
2. Forster, C., et al. (2017). On-Manifold Preintegration for Real-Time Visual-Inertial Odometry. IEEE Transactions on Robotics.
3. Indelman, V., et al. (2015). Information fusion in navigation systems via factor graph based incremental smoothing. Robotics and Autonomous Systems.
