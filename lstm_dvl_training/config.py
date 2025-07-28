"""
LSTM DVL Training Configuration
基于论文参数设置的配置文件
"""

import os

class Config:
    # 数据路径
    DATA_DIR = "data"
    MODEL_DIR = "models"
    LOG_DIR = "logs"
    RESULTS_DIR = "results"
    
    # 原始数据路径
    INPUT_CSV = "../underwater_nav_fgo/data/sensor_fusion_log_20250712_102606.csv"
    
    # 数据预处理参数
    VALID_DATA_START_LINE = 674  # 有效GPS数据开始行（从674行开始使用所有数据）
    SEQUENCE_LENGTH = 10  # 时间窗口长度（过去10个时间步）
    PREDICTION_HORIZON = 1  # 预测未来1个时间步
    
    # 特征选择 (根据CSV文件的实际列名)
    INPUT_FEATURES = [
        # IMU数据
        'acc_x', 'acc_y', 'acc_z',
        'gyr_x', 'gyr_y', 'gyr_z',
        'mag_x', 'mag_y', 'mag_z',
        # DVL数据
        'dvl_vx', 'dvl_vy', 'dvl_vz',
        # GPS位置（作为状态输入）
        'gps_east', 'gps_north', 'gps_up',
        # GPS速度
        'gps_east_vel', 'gps_north_vel', 'gps_up_vel'
    ]
    
    OUTPUT_FEATURES = [
        'gps_east', 'gps_north', 'gps_up'  # 预测下一时刻的GPS位置
    ]
    
    # 数据分割比例
    TRAIN_RATIO = 0.7
    VAL_RATIO = 0.15
    TEST_RATIO = 0.15
    
    # 模型架构参数（按照论文设置）
    LSTM_LAYERS = 3  # 3层LSTM
    LSTM_UNITS = [128, 64, 32]  # 每层单元数
    DROPOUT_RATE = 0.2
    
    # 训练参数（按照论文设置）
    INITIAL_LEARNING_RATE = 0.01  # 初始学习率
    GRADIENT_CLIP_VALUE = 0.75    # 梯度阈值
    BATCH_SIZE = 32
    MAX_EPOCHS = 1000
    EARLY_STOPPING_PATIENCE = 50
    
    # 学习率调度
    LR_SCHEDULER = {
        'type': 'ReduceLROnPlateau',
        'factor': 0.5,
        'patience': 20,
        'min_lr': 1e-6
    }
    
    # 损失函数
    LOSS_FUNCTION = 'mse'  # 均方误差
    
    # 评估指标
    METRICS = ['mae', 'rmse']
    
    # 设备设置
    DEVICE = 'cuda'  # 如果有GPU的话
    
    # 随机种子
    RANDOM_SEED = 42
    
    # 数据标准化
    NORMALIZE_DATA = True
    NORMALIZATION_METHOD = 'standard'  # 'standard' or 'minmax'
    
    # 保存设置
    SAVE_BEST_MODEL = True
    SAVE_CHECKPOINT_EVERY = 10  # 每10个epoch保存一次检查点
    
    # 可视化设置
    PLOT_TRAINING_CURVES = True
    PLOT_PREDICTIONS = True
    
    def __init__(self):
        # 创建必要的目录
        for dir_name in [self.DATA_DIR, self.MODEL_DIR, self.LOG_DIR, self.RESULTS_DIR]:
            os.makedirs(dir_name, exist_ok=True)
    
    def get_input_dim(self):
        """获取输入特征维度"""
        return len(self.INPUT_FEATURES)
    
    def get_output_dim(self):
        """获取输出特征维度"""
        return len(self.OUTPUT_FEATURES)
    
    def print_config(self):
        """打印配置信息"""
        print("=" * 50)
        print("LSTM DVL Training Configuration")
        print("=" * 50)
        print(f"Input CSV: {self.INPUT_CSV}")
        print(f"Valid data starts from line: {self.VALID_DATA_START_LINE}")
        print(f"Sequence length: {self.SEQUENCE_LENGTH}")
        print(f"Input features ({len(self.INPUT_FEATURES)}): {self.INPUT_FEATURES}")
        print(f"Output features ({len(self.OUTPUT_FEATURES)}): {self.OUTPUT_FEATURES}")
        print(f"LSTM architecture: {self.LSTM_LAYERS} layers with units {self.LSTM_UNITS}")
        print(f"Learning rate: {self.INITIAL_LEARNING_RATE}")
        print(f"Gradient clip: {self.GRADIENT_CLIP_VALUE}")
        print(f"Batch size: {self.BATCH_SIZE}")
        print(f"Max epochs: {self.MAX_EPOCHS}")
        print("=" * 50)

# 创建全局配置实例
config = Config()
