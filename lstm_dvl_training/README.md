# LSTM DVL Training Module

这个文件夹专门用于训练LSTM模型来学习DVL（多普勒测速仪）的运动特性，基于论文《GNSS/Multisensor Fusion Using Continuous-Time Factor Graph Optimization for Robust Localization》中的方法。

## 目录结构

```
lstm_dvl_training/
├── README.md                 # 本文件
├── data_processor.py         # 数据预处理模块
├── lstm_model.py            # LSTM模型定义
├── train_lstm.py            # 训练脚本
├── config.py                # 配置参数
├── utils.py                 # 工具函数
├── data/                    # 处理后的训练数据
├── models/                  # 保存的模型
├── logs/                    # 训练日志
└── results/                 # 训练结果和可视化
```

## 训练目标

根据论文Figure 7的描述，训练LSTM模型学习从历史时刻的状态序列到下一时刻位置的映射关系：

- **输入**: 过去N个时间步的状态序列（GPS位置、DVL速度、IMU数据）
- **输出**: 下一时刻的GPS位置预测

## 模型架构

按照论文参数设置：
- **网络层数**: 3层LSTM
- **每层单元数**: 128, 64, 32
- **初始学习率**: 0.01
- **梯度阈值**: 0.75

## 使用方法

### 快速开始（推荐）

一键完成整个训练流程：
```bash
python quick_start.py
```

### 分步执行

1. **数据预处理**：
```bash
python data_processor.py --input_csv ../underwater_nav_fgo/data/sensor_fusion_log_20250712_102606.csv
```

2. **训练模型**：
```bash
python train_lstm.py
```

3. **评估模型**：
```bash
python evaluate.py --model_path models/best_model.pth
```

4. **特征重要性分析**（可选）：
```bash
python evaluate.py --model_path models/best_model.pth --analyze_features
```

### 高级用法

- 跳过数据处理（如果已处理）：
```bash
python quick_start.py --skip_data_processing
```

- 仅评估现有模型：
```bash
python quick_start.py --skip_data_processing --skip_training --model_path models/best_model.pth
```

## 数据说明

使用您现有的CSV数据，从第34588行开始有有效的GPS数据，包含：
- **IMU数据**：加速度(acc_x,y,z)、角速度(gyr_x,y,z)、磁力计(mag_x,y,z)
- **DVL数据**：速度(dvl_vx,vy,vz)
- **GPS数据**：位置(gps_lon,lat,alt)、速度(gps_east_vel,north_vel,up_vel)

### 数据预处理流程

1. 从CSV第34588行开始提取有效GPS数据
2. 创建时间序列窗口（默认10个时间步）
3. 数据标准化（StandardScaler）
4. 分割为训练集(70%)、验证集(15%)、测试集(15%)

## 训练参数（论文设置）

根据论文《An Underwater Multisource Fusion Anomaly Detection Navigation Algorithm Based on Factor Graph and LSTM》：

| 参数 | 值 | 说明 |
|------|----|----|
| LSTM层数 | 3 | 堆叠的LSTM层 |
| 每层单元数 | [128, 64, 32] | 递减的隐藏单元 |
| 学习率 | 0.01 | 初始学习率 |
| 梯度裁剪 | 0.75 | 梯度阈值 |
| 序列长度 | 10 | 输入时间窗口 |
| 批次大小 | 32 | 训练批次大小 |

## 输出文件

训练完成后，将在以下目录生成文件：

- `data/`: 处理后的训练数据和标准化器
- `models/`: 训练好的模型权重
- `logs/`: 训练日志
- `results/`: 评估结果和可视化图表
  - `data_overview.png`: 数据概览
  - `training_curves.png`: 训练曲线
  - `prediction_comparison.png`: 预测对比
  - `error_analysis.png`: 误差分析
  - `evaluation_results.json`: 详细评估指标

## 性能指标

模型将输出以下评估指标：
- **RMSE**: 均方根误差
- **MAE**: 平均绝对误差
- **水平误差**: 2D平面定位误差
- **各维度误差**: East、North、Up方向的独立误差
