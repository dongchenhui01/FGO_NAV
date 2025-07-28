# LSTM_TimeFGO_nav 项目结构详解

## 📁 项目整体结构

```
LSTM_TimeFGO_nav/
├── 📄 README.md                           # 项目主说明文档
├── 📄 PROJECT_INFO.md                     # 项目详细信息
├── 📄 PROJECT_SUMMARY.md                  # 项目详细总结
├── 📄 PROJECT_STRUCTURE.md                # 项目结构说明 (本文件)
├── 🚀 start_navigation.sh                 # 一键启动脚本
├── 📊 simple_csv_test.cpp                 # CSV数据测试程序
├── 📊 test_csv_reader.cpp                 # CSV读取器测试
├── 📚 Zhang 等 - 2024 - GNSSMultisensor Fusion Using Continuous-Time Factor Graph Optimization for Robust Localization.pdf
├── 📈 gtsam_error_comparison.png          # 误差对比图
├── 📈 gtsam_position_comparison.png       # 位置对比图
├── 📈 gtsam_rmse_comparison.png          # RMSE对比图
├── 📈 gtsam_trajectory_comparison.png    # 轨迹对比图
│
├── 🏗️ underwater_nav_ws/                  # ROS2工作空间 (核心导航系统)
│   ├── 📁 src/
│   │   └── 📁 underwater_nav_fgo/        # 主要源代码包
│   │       ├── 📁 data_preprocessing/     # 数据预处理模块
│   │       ├── 📁 factor_graph_optimizer/ # 因子图优化模块
│   │       ├── 📁 underwater_nav_msgs/    # 消息定义模块
│   │       ├── 📁 visualization/          # 可视化模块
│   │       └── 📁 state_estimator/        # 状态估计模块
│   ├── 📁 build/                          # 编译输出目录
│   ├── 📁 install/                        # 安装文件目录
│   └── 📁 log/                            # 日志文件目录
│
├── 🧠 lstm_dvl_training/                  # LSTM神经网络训练模块
│   ├── 📄 README.md                       # LSTM模块说明
│   ├── 📄 GETTING_STARTED.md              # 快速开始指南
│   ├── 📄 requirements.txt                 # Python依赖
│   ├── 🚀 install_dependencies.sh         # 依赖安装脚本
│   ├── 🚀 quick_start.py                  # 一键训练脚本
│   ├── 🐍 data_processor.py               # 数据预处理
│   ├── 🐍 lstm_model.py                   # LSTM模型定义
│   ├── 🐍 train_lstm.py                   # 训练脚本
│   ├── 🐍 evaluate.py                     # 评估脚本
│   ├── 🐍 config.py                       # 配置参数
│   ├── 🐍 utils.py                        # 工具函数
│   ├── 📁 data/                           # 处理后的训练数据
│   ├── 📁 models/                         # 保存的模型
│   ├── 📁 logs/                           # 训练日志
│   └── 📁 results/                        # 训练结果和可视化
│
├── 🎮 auv_simulation_env/                 # AUV仿真验证环境
│   ├── 📄 README.md                       # 仿真环境说明
│   ├── 📄 INSTALL_GUIDE.md                # 安装指南
│   ├── 📄 QUICK_START.md                  # 快速开始
│   ├── 🚀 setup_environment.sh            # 环境设置脚本
│   ├── 🚀 launch_simulation.sh            # 启动仿真脚本
│   ├── 🚀 launch_simulation_headless.sh   # 无头模式启动
│   ├── 🚀 stop_simulation.sh              # 停止仿真脚本
│   ├── 🚀 check_installation.sh           # 安装检查脚本
│   ├── 📦 Copter-4.3.8.tar.gz            # ArduPilot源码包
│   ├── 📁 ardupilot/                      # ArduPilot源码
│   ├── 📁 config/                         # 配置文件
│   ├── 📁 data/                           # 数据文件
│   ├── 📁 results/                        # 仿真结果
│   ├── 📁 scripts/                        # 工具脚本
│   └── 📁 ros_packages/                   # ROS包
│
├── 🔧 gtsam_project/                      # GTSAM相关工具和API
│   ├── 📁 gtsam_env/                      # Python环境
│   └── 🐍 gtsam_api_3d_nav.py            # 3D导航API
│
└── 📁 underwater_nav_fgo/                 # 配置、数据和启动文件
    ├── 📁 config/                         # 配置文件
    ├── 📁 data/                           # 传感器数据
    ├── 📁 launch/                         # 启动文件
    ├── 📁 rviz/                           # 可视化配置
    └── 📁 scripts/                        # 脚本文件
```

## 🔍 核心模块详细结构

### 1. 水下导航系统 (`underwater_nav_ws/`)

```
underwater_nav_ws/src/underwater_nav_fgo/
├── 📁 data_preprocessing/                 # 数据预处理模块
│   ├── 📄 CMakeLists.txt                  # 构建配置
│   ├── 📄 package.xml                     # 包信息
│   ├── 📁 include/data_preprocessing/     # 头文件
│   ├── 📁 src/                           # 源代码
│   │   └── 📄 csv_player_node.cpp        # CSV数据播放器
│   └── 📁 test/                          # 测试文件
│
├── 📁 factor_graph_optimizer/             # 因子图优化模块
│   ├── 📄 CMakeLists.txt                 # 构建配置
│   ├── 📄 package.xml                    # 包信息
│   ├── 📁 config/                        # 配置文件
│   ├── 📁 include/factor_graph_optimizer/ # 头文件
│   │   ├── 📄 time_centric_fgo.hpp       # 时间中心FGO
│   │   ├── 📄 time_centric_factors.hpp   # 时间中心因子
│   │   └── 📄 underwater_fgo.hpp         # 水下FGO
│   ├── 📁 src/                           # 源代码
│   │   ├── 📄 time_centric_navigation_node.cpp  # 主导航节点
│   │   ├── 📄 underwater_fgo.cpp         # 水下FGO实现
│   │   ├── 📄 time_centric_methods.cpp   # 时间中心方法
│   │   ├── 📄 visualization_node.cpp     # 可视化节点
│   │   ├── 📄 dvl_factor.cpp             # DVL因子
│   │   ├── 📄 gps_reference_publisher.cpp # GPS参考发布器
│   │   ├── 📄 lever_arm_compensation.cpp # 杆臂补偿
│   │   ├── 📄 test_lever_arm.cpp         # 杆臂测试
│   │   └── 📄 underwater_navigation_node.cpp # 水下导航节点
│   ├── 📁 launch/                        # 启动文件
│   └── 📁 test/                          # 测试文件
│
├── 📁 underwater_nav_msgs/                # 消息定义模块
│   ├── 📄 CMakeLists.txt                 # 构建配置
│   ├── 📄 package.xml                    # 包信息
│   └── 📁 msg/                           # 消息定义
│       ├── 📄 ImuData.msg               # IMU数据消息
│       ├── 📄 DvlData.msg               # DVL数据消息
│       └── 📄 NavigationState.msg       # 导航状态消息
│
├── 📁 visualization/                      # 可视化模块
│   ├── 📄 CMakeLists.txt                 # 构建配置
│   ├── 📄 package.xml                    # 包信息
│   └── 📁 src/                           # 源代码
│
└── 📁 state_estimator/                    # 状态估计模块
    ├── 📄 CMakeLists.txt                 # 构建配置
    ├── 📄 package.xml                    # 包信息
    └── 📁 src/                           # 源代码
```

### 2. LSTM训练模块 (`lstm_dvl_training/`)

```
lstm_dvl_training/
├── 📄 README.md                          # 模块说明
├── 📄 GETTING_STARTED.md                 # 快速开始
├── 📄 requirements.txt                    # Python依赖
├── 🚀 install_dependencies.sh            # 依赖安装
├── 🚀 quick_start.py                     # 一键训练
├── 🐍 data_processor.py                  # 数据预处理
├── 🐍 lstm_model.py                      # LSTM模型
├── 🐍 train_lstm.py                      # 训练脚本
├── 🐍 evaluate.py                        # 评估脚本
├── 🐍 config.py                          # 配置参数
├── 🐍 utils.py                           # 工具函数
├── 📁 data/                              # 训练数据
│   ├── 📄 processed_data.pkl             # 处理后数据
│   ├── 📄 scaler.pkl                     # 标准化器
│   └── 📄 data_info.json                 # 数据信息
├── 📁 models/                            # 训练模型
│   ├── 📄 best_model.pth                 # 最佳模型
│   ├── 📄 latest_model.pth               # 最新模型
│   └── 📄 model_config.json              # 模型配置
├── 📁 logs/                              # 训练日志
│   └── 📄 training_log.txt               # 训练日志
└── 📁 results/                           # 训练结果
    ├── 📄 data_overview.png              # 数据概览
    ├── 📄 training_curves.png            # 训练曲线
    ├── 📄 prediction_comparison.png       # 预测对比
    ├── 📄 error_analysis.png             # 误差分析
    └── 📄 evaluation_results.json        # 评估结果
```

### 3. AUV仿真环境 (`auv_simulation_env/`)

```
auv_simulation_env/
├── 📄 README.md                          # 环境说明
├── 📄 INSTALL_GUIDE.md                   # 安装指南
├── 📄 QUICK_START.md                     # 快速开始
├── 🚀 setup_environment.sh               # 环境设置
├── 🚀 launch_simulation.sh               # 启动仿真
├── 🚀 launch_simulation_headless.sh      # 无头模式
├── 🚀 stop_simulation.sh                 # 停止仿真
├── 🚀 check_installation.sh              # 安装检查
├── 📦 Copter-4.3.8.tar.gz               # ArduPilot源码
├── 📁 ardupilot/                         # ArduPilot源码
├── 📁 config/                            # 配置文件
│   ├── 📄 sitl_params.txt                # SITL参数
│   └── 📁 gazebo_worlds/                 # Gazebo世界
├── 📁 data/                              # 数据文件
│   ├── 📁 real_data/                     # 真实数据
│   └── 📁 simulation_data/               # 仿真数据
├── 📁 results/                           # 仿真结果
├── 📁 scripts/                           # 工具脚本
│   ├── 🐍 data_replay.py                 # 数据回放
│   └── 🐍 algorithm_test.py              # 算法测试
└── 📁 ros_packages/                      # ROS包
    ├── 📁 auv_data_replay/               # 数据回放包
    ├── 📁 auv_algorithm_test/            # 算法测试包
    └── 📁 auv_ground_truth/              # Ground Truth包
```

## 🔗 模块间关系

### 数据流关系
```
传感器数据 → 数据预处理 → 因子图优化 → 导航结果
     ↓              ↓            ↓           ↓
  CSV文件    →   CSV播放器  →  FGO节点  →  可视化
     ↓              ↓            ↓           ↓
  LSTM训练  →   数据处理器  →  模型训练  →  预测结果
     ↓              ↓            ↓           ↓
  仿真环境  →   数据回放   →  算法测试  →  精度验证
```

### 依赖关系
```
ROS2 Galactic (基础框架)
    ↓
GTSAM 4.1+ (优化引擎)
    ↓
Eigen3 + Boost (数学库)
    ↓
自定义算法 (时间中心FGO)
    ↓
LSTM神经网络 (深度学习)
    ↓
仿真验证 (ArduSub + Gazebo)
```

## 📊 文件统计

### 代码文件统计
- **C++源文件**: ~15个
- **Python脚本**: ~8个
- **配置文件**: ~10个
- **启动脚本**: ~5个
- **文档文件**: ~8个

### 模块大小统计
- **水下导航系统**: 最大模块，包含核心算法
- **LSTM训练模块**: 中等大小，专注于深度学习
- **仿真环境**: 较大模块，包含完整仿真环境
- **配置和工具**: 较小模块，提供支持功能

## 🎯 使用场景

### 1. 研究用途
- 时间中心因子图优化算法研究
- 多传感器融合技术验证
- LSTM在水下导航中的应用研究

### 2. 教学用途
- 水下导航系统教学
- ROS2开发实践
- 机器学习在机器人中的应用

### 3. 工程应用
- AUV导航系统开发
- 传感器融合算法实现
- 实时导航系统部署

## 🔧 开发环境

### 系统要求
- **操作系统**: Ubuntu 20.04 LTS
- **ROS2**: Galactic版本
- **GTSAM**: 4.1或更高版本
- **Python**: 3.8+
- **硬件**: 至少8GB RAM，推荐16GB

### 开发工具
- **IDE**: VSCode, CLion
- **版本控制**: Git
- **构建系统**: CMake
- **包管理**: pip, colcon

---

**文档版本**: v1.0.0  
**最后更新**: 2025-01-27 