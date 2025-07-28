# LSTM_TimeFGO_nav 项目信息

## 📋 项目概述

**项目名称**: LSTM_TimeFGO_nav - 水下导航时间中心因子图优化系统  
**创建日期**: 2025-07-19  
**作者**: dongchenhui  
**版本**: v1.0.0  

## 🎯 项目目标

开发一个基于时间中心因子图优化的高精度水下导航系统，结合多传感器融合技术，实现实时、鲁棒的水下载体定位与导航。

## 🔧 技术栈

### 核心框架
- **ROS2 Galactic**: 机器人操作系统
- **GTSAM 4.1+**: 几何SLAM工具箱
- **Eigen3**: 线性代数库
- **Boost**: C++扩展库

### 编程语言
- **C++17**: 主要开发语言
- **Python 3.8**: 数据分析和可视化
- **CMake**: 构建系统

### 算法技术
- **时间中心因子图优化**: 连续时间轨迹表示
- **高斯过程回归**: 时间插值和平滑
- **ISAM2增量优化**: 实时因子图求解
- **多传感器融合**: IMU/DVL/GPS/磁力计

## 📊 系统架构

### 核心模块

1. **数据预处理模块** (`data_preprocessing`)
   - CSV数据播放器
   - 传感器数据同步
   - 数据质量检查

2. **因子图优化器** (`factor_graph_optimizer`)
   - 时间中心因子图构建
   - ISAM2增量优化
   - 多传感器因子实现

3. **消息定义** (`underwater_nav_msgs`)
   - 自定义ROS2消息类型
   - 传感器数据结构

4. **可视化模块**
   - RViz配置
   - 实时轨迹显示
   - 传感器数据可视化

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

## 📁 文件结构详解

```
LSTM_TimeFGO_nav/
├── README.md                    # 项目主说明文档
├── PROJECT_INFO.md             # 项目详细信息 (本文件)
├── start_navigation.sh         # 系统启动脚本
├── 
├── underwater_nav_ws/          # ROS2工作空间
│   ├── src/underwater_nav_fgo/ # 主要源代码包
│   │   ├── data_preprocessing/     # 数据预处理
│   │   │   ├── src/csv_player_node.cpp
│   │   │   └── include/data_preprocessing/
│   │   ├── factor_graph_optimizer/ # 因子图优化
│   │   │   ├── src/
│   │   │   │   ├── time_centric_navigation_node.cpp
│   │   │   │   ├── underwater_fgo.cpp
│   │   │   │   ├── time_centric_methods.cpp
│   │   │   │   └── visualization_node.cpp
│   │   │   └── include/factor_graph_optimizer/
│   │   │       ├── time_centric_fgo.hpp
│   │   │       ├── time_centric_factors.hpp
│   │   │       └── underwater_fgo.hpp
│   │   └── underwater_nav_msgs/    # 消息定义
│   │       └── msg/
│   │           ├── ImuData.msg
│   │           ├── DvlData.msg
│   │           └── NavigationState.msg
│   ├── build/                  # 编译输出
│   ├── install/                # 安装文件
│   └── log/                    # 日志文件
│
├── underwater_nav_fgo/         # 配置和数据
│   ├── config/                 # 配置文件
│   │   ├── time_centric_params.yaml
│   │   └── sensor_params.yaml
│   ├── data/                   # 传感器数据
│   │   └── sensor_fusion_log_20250712_102606.csv
│   ├── launch/                 # 启动文件
│   │   └── time_centric_navigation.launch.py
│   ├── rviz/                   # 可视化配置
│   │   └── underwater_nav_2d.rviz
│   └── scripts/                # 工具脚本
│
├── gtsam_project/              # GTSAM相关
│   ├── gtsam_env/             # Python环境
│   └── gtsam_api_3d_nav.py    # 3D导航API
│
└── docs/                       # 文档和结果
    ├── Zhang等-2024-论文.pdf   # 参考论文
    ├── gtsam_trajectory_comparison.png
    ├── gtsam_position_comparison.png
    ├── gtsam_error_comparison.png
    └── gtsam_rmse_comparison.png
```

## 🚀 使用方法

### 快速启动
```bash
cd LSTM_TimeFGO_nav
./start_navigation.sh
```

### 手动启动
```bash
cd LSTM_TimeFGO_nav/underwater_nav_ws
source /opt/ros/galactic/setup.bash
source install/setup.bash

# 启动各个节点
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link
ros2 run data_preprocessing csv_player_node --ros-args -p data_file:=../data/sensor_fusion_log.csv
ros2 run factor_graph_optimizer time_centric_navigation_node
ros2 run factor_graph_optimizer visualization_node
rviz2 -d ../config/underwater_nav_2d.rviz
```

## 📈 性能指标

### 实时性能
- **优化求解时间**: < 10ms
- **数据处理频率**: 100Hz (IMU), 10Hz (DVL), 1Hz (GPS)
- **内存使用**: < 500MB

### 定位精度
- **位置精度**: 亚米级 (< 1m)
- **速度精度**: < 0.1 m/s
- **航向精度**: < 5°

### 系统鲁棒性
- **传感器故障容忍**: 支持单传感器失效
- **异常值处理**: 鲁棒核函数
- **长时间运行**: > 1小时连续运行

## 🔬 实验数据

### 数据集信息
- **文件**: `sensor_fusion_log_20250712_102606.csv`
- **时长**: 约30分钟航行数据
- **数据量**: 34,690行记录
- **传感器**: IMU + DVL + GPS + 磁力计

### 数据格式
```csv
timestamp,imu_accel_x,imu_accel_y,imu_accel_z,imu_gyro_x,imu_gyro_y,imu_gyro_z,
dvl_vel_x,dvl_vel_y,dvl_vel_z,gps_east,gps_north,gps_up,gps_quality,
mag_x,mag_y,mag_z
```

## 🛠️ 开发说明

### 编译依赖
```bash
# 安装ROS2 Galactic
sudo apt install ros-galactic-desktop

# 安装GTSAM
sudo apt install libgtsam-dev

# 安装其他依赖
sudo apt install libeigen3-dev libboost-all-dev
```

### 调试工具
```bash
# 检查节点状态
ros2 node list

# 检查话题数据
ros2 topic list
ros2 topic hz /imu_data
ros2 topic echo /navigation_state

# 检查参数
ros2 param list /time_centric_navigation_node
```

## 📚 参考资料

1. **主要论文**: Zhang et al. (2024) - "GNSS/Multi-sensor Fusion Using Continuous-Time Factor Graph Optimization for Robust Localization"
2. **GTSAM文档**: https://gtsam.org/
3. **ROS2文档**: https://docs.ros.org/en/galactic/

## 🔄 版本历史

- **v1.0.0** (2025-07-19): 初始版本
  - 实现时间中心因子图优化
  - 支持多传感器融合
  - 完成ROS2集成

## 📞 联系信息

- **开发者**: dongchenhui
- **邮箱**: 241314010016@hhu.edu.cn
- **项目地址**: `/home/dongchenhui/LSTM_TimeFGO_nav`

## 📄 许可证

本项目采用MIT许可证，详见LICENSE文件。
