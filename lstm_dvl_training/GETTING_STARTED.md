# LSTM DVL训练 - 快速开始指南

## 概述

这个模块实现了基于论文《An Underwater Multisource Fusion Anomaly Detection Navigation Algorithm Based on Factor Graph and LSTM》的LSTM模型训练，用于学习DVL（多普勒测速仪）的运动特性。

## 安装依赖

### 方法1：使用安装脚本（推荐）
```bash
./install_dependencies.sh
```

### 方法2：手动安装
```bash
pip3 install torch numpy pandas matplotlib seaborn scikit-learn
```

## 快速开始

### 一键训练（最简单）
```bash
python3 quick_start.py
```

这个命令会自动完成：
1. 数据预处理（从您的CSV文件）
2. LSTM模型训练
3. 模型评估和可视化

### 验证安装
```bash
python3 lstm_model.py
```
如果看到模型信息输出，说明安装成功。

## 训练流程详解

### 1. 数据预处理
```bash
python3 data_processor.py
```

**输入**：`../underwater_nav_fgo/data/sensor_fusion_log_20250712_102606.csv`
**输出**：
- `data/X_train.npy`, `data/y_train.npy` - 训练数据
- `data/X_val.npy`, `data/y_val.npy` - 验证数据  
- `data/X_test.npy`, `data/y_test.npy` - 测试数据
- `data/scaler_X.pkl`, `data/scaler_y.pkl` - 数据标准化器

### 2. 模型训练
```bash
python3 train_lstm.py
```

**训练参数**（按论文设置）：
- 3层LSTM，单元数：[128, 64, 32]
- 学习率：0.01
- 梯度裁剪：0.75
- 批次大小：32

**输出**：
- `models/best_model.pth` - 最佳模型
- `models/checkpoint_epoch_*.pth` - 训练检查点
- `results/training_curves.png` - 训练曲线

### 3. 模型评估
```bash
python3 evaluate.py --model_path models/best_model.pth
```

**输出**：
- `results/prediction_comparison.png` - 预测对比图
- `results/error_analysis.png` - 误差分析图
- `results/evaluation_results.json` - 详细评估指标

## 高级功能

### 特征重要性分析
```bash
python3 evaluate.py --model_path models/best_model.pth --analyze_features
```

### 自定义配置
编辑 `config.py` 文件来修改：
- 网络架构参数
- 训练超参数
- 数据处理参数

## 预期结果

训练完成后，您将获得：

1. **训练好的LSTM模型**，能够根据历史的IMU、DVL、GPS数据预测下一时刻的GPS位置
2. **详细的性能评估**，包括RMSE、MAE等指标
3. **可视化结果**，包括轨迹对比、误差分析等
4. **特征重要性分析**，了解哪些传感器数据对预测最重要

## 故障排除

### 常见问题

1. **ModuleNotFoundError: No module named 'torch'**
   ```bash
   ./install_dependencies.sh
   ```

2. **CUDA相关错误**
   - 如果没有GPU，模型会自动使用CPU
   - 如果有GPU但出现CUDA错误，请安装对应版本的PyTorch

3. **内存不足**
   - 减小 `config.py` 中的 `BATCH_SIZE`
   - 减小 `SEQUENCE_LENGTH`

4. **数据文件未找到**
   - 确保CSV文件路径正确：`../underwater_nav_fgo/data/sensor_fusion_log_20250712_102606.csv`

### 检查数据
```bash
python3 -c "
import pandas as pd
df = pd.read_csv('../underwater_nav_fgo/data/sensor_fusion_log_20250712_102606.csv')
print(f'Total rows: {len(df)}')
print(f'Valid GPS rows: {len(df[df.gps_lon != 0])}')
"
```

## 论文参考

本实现基于以下论文：
- **标题**: An Underwater Multisource Fusion Anomaly Detection Navigation Algorithm Based on Factor Graph and LSTM
- **架构**: 输入层 → 3层LSTM → 全连接层 → 输出层
- **参数**: 学习率0.01，梯度裁剪0.75，LSTM单元[128,64,32]

## 联系支持

如果遇到问题，请检查：
1. 依赖是否正确安装
2. 数据文件是否存在且格式正确
3. 配置参数是否合理
