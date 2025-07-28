# LSTM_TimeFGO_nav - 水下导航时间中心因子图优化系统

## 📋 项目概述

**LSTM_TimeFGO_nav** 是一个基于时间中心因子图优化的高精度水下导航系统，结合了LSTM神经网络和连续时间因子图优化技术，用于实现实时、鲁棒的水下载体定位与导航。

### 🎯 核心特性
- **时间中心因子图优化**: 基于连续时间轨迹表示的高精度定位
- **多传感器融合**: IMU、DVL、GPS、磁力计数据融合
- **LSTM神经网络**: 学习传感器运动特性，提高预测精度
- **实时处理**: < 10ms优化求解时间，支持实时导航
- **鲁棒性**: 支持传感器故障和异常值处理

## 🏗️ 项目架构

### 整体结构
```
LSTM_TimeFGO_nav/
├── underwater_nav_ws/           # ROS2工作空间 (核心导航系统)
├── lstm_dvl_training/           # LSTM神经网络训练模块
├── auv_simulation_env/          # AUV仿真验证环境
├── gtsam_project/               # GTSAM相关工具和API
├── underwater_nav_fgo/          # 配置、数据和启动文件
├── start_navigation.sh          # 一键启动脚本
└── docs/                        # 文档和实验结果
```

### 核心模块详解

#### 1. 水下导航系统 (`underwater_nav_ws/`)
**主要功能**: 实现时间中心因子图优化的核心导航算法

**核心组件**:
- **数据预处理模块** (`data_preprocessing/`)
  - CSV数据播放器，支持多种传感器数据格式
  - 时间戳同步和高精度时间处理
  - 数据质量检查和有效性验证

- **因子图优化器** (`factor_graph_optimizer/`)
  - 时间中心因子图构建和优化
  - ISAM2增量优化算法实现
  - 多传感器因子（IMU、DVL、GPS、磁力计）
  - 连续时间约束和轨迹平滑

- **可视化模块** (`visualization/`)
  - 实时轨迹显示和传感器数据可视化
  - RViz配置和自定义显示插件

**技术特点**:
- 使用GTSAM 4.1+作为底层优化引擎
- 支持连续时间轨迹表示
- 实现滑动窗口边缘化策略
- 提供鲁棒核函数处理异常值

#### 2. LSTM神经网络训练模块 (`lstm_dvl_training/`)
**主要功能**: 训练LSTM模型学习传感器运动特性，提高预测精度

**核心特性**:
- **3层LSTM架构**: [128, 64, 32]隐藏单元配置
- **时间序列预测**: 从历史状态序列预测下一时刻位置
- **多传感器输入**: IMU、DVL、GPS数据融合训练
- **自适应学习**: 支持梯度裁剪和学习率调整

**训练流程**:
```python
# 快速开始
python quick_start.py

# 分步执行
python data_processor.py    # 数据预处理
python train_lstm.py       # 模型训练
python evaluate.py         # 模型评估
```

#### 3. AUV仿真验证环境 (`auv_simulation_env/`)
**主要功能**: 提供完整的仿真环境验证算法精度和性能

**技术架构**:
- **ArduSub SITL**: 软件在环仿真，模拟飞控硬件
- **Gazebo**: 3D物理仿真引擎，提供Ground Truth
- **ROS 2**: 通信框架，连接各组件
- **MAVROS**: ROS与ArduPilot的桥梁

**验证能力**:
- 算法精度验证（对比Ground Truth）
- 性能测试（计算效率、实时性）
- 闭环仿真测试
- 传感器故障模拟

## 🔧 技术栈

### 核心框架
- **ROS2 Galactic**: 机器人操作系统
- **GTSAM 4.1+**: 几何SLAM工具箱
- **Eigen3**: 线性代数库
- **Boost**: C++扩展库

### 编程语言
- **C++17**: 主要开发语言（导航算法）
- **Python 3.8**: 数据分析和机器学习
- **CMake**: 构建系统

### 算法技术
- **时间中心因子图优化**: 连续时间轨迹表示
- **高斯过程回归**: 时间插值和平滑
- **ISAM2增量优化**: 实时因子图求解
- **LSTM神经网络**: 深度学习预测
- **多传感器融合**: IMU/DVL/GPS/磁力计

## 🚀 快速开始

### 环境要求
- **操作系统**: Ubuntu 20.04 LTS
- **ROS2**: Galactic版本
- **GTSAM**: 4.1或更高版本
- **Python**: 3.8+
- **硬件**: 至少8GB RAM，推荐16GB

### 一键启动
```bash
# 克隆项目
git clone <repository_url>
cd LSTM_TimeFGO_nav

# 一键启动完整系统
./start_navigation.sh
```

### 手动启动
```bash
# 1. 编译项目
cd underwater_nav_ws
source /opt/ros/galactic/setup.bash
colcon build --parallel-workers 1

# 2. 设置环境
source install/setup.bash

# 3. 启动各个节点
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link
ros2 run data_preprocessing csv_player_node --ros-args -p data_file:=../data/sensor_fusion_log.csv
ros2 run factor_graph_optimizer time_centric_navigation_node
ros2 run factor_graph_optimizer visualization_node
rviz2 -d ../rviz/underwater_nav_2d.rviz
```

## 📊 系统特性

### 因子图结构
```
因子图节点:
├── X(i): 位姿节点 (Pose3)
├── V(i): 速度节点 (Vector3)  
└── B(i): IMU偏差节点 (imuBias::ConstantBias)

因子类型:
├── 先验因子 (Prior Factors)
├── 时间中心IMU因子 (Time-Centric IMU Factors)
├── DVL速度因子 (DVL Velocity Factors)
├── 磁力计因子 (Magnetometer Factors)
├── GPS位置因子 (GPS Position Factors)
└── 连续时间约束因子 (Continuous-Time Constraint Factors)
```

### 性能指标
- **实时性**: < 10ms 优化求解时间
- **精度**: 亚米级定位精度 (< 1m)
- **鲁棒性**: 支持传感器故障和异常值处理
- **可扩展性**: 模块化设计，易于添加新传感器

### 数据处理能力
- **IMU数据**: 100Hz高频处理
- **DVL数据**: 10Hz速度约束
- **GPS数据**: 1Hz位置更新
- **磁力计**: 航向角约束

## 📈 实验结果

### 数据集信息
- **数据文件**: `sensor_fusion_log_20250712_102606.csv`
- **数据时长**: 约30分钟航行数据
- **数据量**: 34,690行记录
- **传感器**: IMU + DVL + GPS + 磁力计

### 可视化结果
项目包含多个实验结果可视化：
- `gtsam_trajectory_comparison.png`: 轨迹对比图
- `gtsam_position_comparison.png`: 位置精度对比
- `gtsam_error_comparison.png`: 误差分析
- `gtsam_rmse_comparison.png`: RMSE对比

### LSTM训练结果
- **训练数据**: 从第34588行开始的GPS有效数据
- **模型架构**: 3层LSTM [128, 64, 32]
- **预测精度**: 位置预测RMSE < 2m
- **训练时间**: 约30分钟（GPU加速）

## 🔬 实验验证

### 仿真环境验证
```bash
# 启动仿真环境
cd auv_simulation_env
./launch_simulation.sh

# 运行算法测试
ros2 run auv_algorithm_test your_algorithm_node
```

### 性能测试
- **计算效率**: CPU使用率 < 50%
- **内存占用**: < 500MB
- **实时性**: 处理延迟 < 100ms
- **精度验证**: 与Ground Truth对比误差 < 1m

## 📚 参考文献

主要基于以下论文实现：
- **Zhang et al. (2024)**: "GNSS/Multi-sensor Fusion Using Continuous-Time Factor Graph Optimization for Robust Localization"
- **相关论文**: "An Underwater Multisource Fusion Anomaly Detection Navigation Algorithm Based on Factor Graph and LSTM"

## 🛠️ 开发指南

### 添加新传感器
1. 在`underwater_nav_msgs`中定义新消息类型
2. 在`factor_graph_optimizer`中实现新因子
3. 在`data_preprocessing`中添加数据处理逻辑
4. 更新配置文件和启动文件

### 调试工具
```bash
# 检查节点状态
ros2 node list

# 检查话题数据
ros2 topic list
ros2 topic hz /imu_data

# 检查参数
ros2 param list /time_centric_navigation_node

# 查看日志
ros2 run factor_graph_optimizer time_centric_navigation_node --ros-args --log-level DEBUG
```

## 🔧 故障排除

### 常见问题
1. **编译错误**: 检查GTSAM和依赖库版本
2. **节点启动失败**: 确认ROS2环境正确配置
3. **数据播放问题**: 检查CSV文件路径和格式
4. **可视化问题**: 确认RViz配置文件路径
5. **LSTM训练失败**: 检查Python环境和依赖包

### 调试命令
```bash
# 检查系统状态
ros2 node list
ros2 topic list
ros2 topic hz /imu_data

# 检查参数
ros2 param list /time_centric_navigation_node

# 查看详细日志
ros2 run factor_graph_optimizer time_centric_navigation_node --ros-args --log-level DEBUG
```

## 👥 贡献指南

欢迎提交Issue和Pull Request来改进项目！

### 贡献流程
1. Fork项目仓库
2. 创建功能分支
3. 提交代码更改
4. 创建Pull Request

## 📄 许可证

本项目采用MIT许可证。详见LICENSE文件。

## 📞 联系信息

- **开发者**: dongchenhui
- **邮箱**: 241314010016@hhu.edu.cn
- **项目地址**: `/home/dongchenhui/LSTM_TimeFGO_nav`

---

**最后更新**: 2025-01-27  
**版本**: v1.0.0
