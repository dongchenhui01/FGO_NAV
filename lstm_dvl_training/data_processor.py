"""
数据预处理模块
处理CSV数据，创建LSTM训练所需的时间序列数据
"""

import pandas as pd
import numpy as np
import pickle
from sklearn.preprocessing import StandardScaler, MinMaxScaler
from sklearn.model_selection import train_test_split
import matplotlib.pyplot as plt
import seaborn as sns
from config import config
import argparse
import os

class DataProcessor:
    def __init__(self, config):
        self.config = config
        self.scaler_X = None
        self.scaler_y = None
        
    def load_csv_data(self, csv_path):
        """加载CSV数据"""
        print(f"Loading data from {csv_path}...")

        # 读取CSV文件
        df = pd.read_csv(csv_path)
        print(f"Total rows in CSV: {len(df)}")

        # 从指定行开始使用数据
        df_from_start = df.iloc[self.config.VALID_DATA_START_LINE:].copy()
        print(f"Using data from row {self.config.VALID_DATA_START_LINE}: {len(df_from_start)} rows")

        # 检查GPS数据是否有效
        gps_valid = (df_from_start['gps_lon'] != 0) & (df_from_start['gps_lat'] != 0)
        df_valid = df_from_start[gps_valid].copy()
        print(f"Rows with valid GPS data: {len(df_valid)}")

        if len(df_valid) > 0:
            print(f"GPS data range: rows {df_valid.index[0]} to {df_valid.index[-1]}")
            print(f"GPS coordinates span:")
            print(f"  Longitude: {df_valid['gps_lon'].min():.8f} to {df_valid['gps_lon'].max():.8f}")
            print(f"  Latitude: {df_valid['gps_lat'].min():.8f} to {df_valid['gps_lat'].max():.8f}")
            print(f"  Altitude: {df_valid['gps_alt'].min():.3f} to {df_valid['gps_alt'].max():.3f}")

        return df_valid
    
    def preprocess_features(self, df):
        """预处理特征数据"""
        print("Preprocessing features...")
        
        # 选择输入特征
        X_features = df[self.config.INPUT_FEATURES].copy()
        y_features = df[self.config.OUTPUT_FEATURES].copy()
        
        # 检查缺失值
        print(f"Missing values in X: {X_features.isnull().sum().sum()}")
        print(f"Missing values in y: {y_features.isnull().sum().sum()}")
        
        # 填充缺失值（如果有）
        X_features = X_features.fillna(method='ffill').fillna(method='bfill')
        y_features = y_features.fillna(method='ffill').fillna(method='bfill')
        
        # 数据标准化
        if self.config.NORMALIZE_DATA:
            if self.config.NORMALIZATION_METHOD == 'standard':
                self.scaler_X = StandardScaler()
                self.scaler_y = StandardScaler()
            else:
                self.scaler_X = MinMaxScaler()
                self.scaler_y = MinMaxScaler()
            
            X_scaled = self.scaler_X.fit_transform(X_features)
            y_scaled = self.scaler_y.fit_transform(y_features)
            
            X_features = pd.DataFrame(X_scaled, columns=X_features.columns, index=X_features.index)
            y_features = pd.DataFrame(y_scaled, columns=y_features.columns, index=y_features.index)
        
        return X_features, y_features
    
    def create_sequences(self, X, y):
        """创建时间序列数据"""
        print("Creating time sequences...")
        
        X_sequences = []
        y_sequences = []
        
        seq_len = self.config.SEQUENCE_LENGTH
        pred_horizon = self.config.PREDICTION_HORIZON
        
        for i in range(len(X) - seq_len - pred_horizon + 1):
            # 输入序列：过去seq_len个时间步
            X_seq = X.iloc[i:i+seq_len].values
            # 输出：未来pred_horizon个时间步后的位置
            y_seq = y.iloc[i+seq_len+pred_horizon-1].values
            
            X_sequences.append(X_seq)
            y_sequences.append(y_seq)
        
        X_sequences = np.array(X_sequences)
        y_sequences = np.array(y_sequences)
        
        print(f"Created {len(X_sequences)} sequences")
        print(f"X shape: {X_sequences.shape}")
        print(f"y shape: {y_sequences.shape}")
        
        return X_sequences, y_sequences
    
    def split_data(self, X, y):
        """分割训练、验证和测试数据"""
        print("Splitting data...")
        
        # 首先分出测试集
        X_temp, X_test, y_temp, y_test = train_test_split(
            X, y, test_size=self.config.TEST_RATIO, 
            random_state=self.config.RANDOM_SEED, shuffle=False
        )
        
        # 再从剩余数据中分出训练集和验证集
        val_size = self.config.VAL_RATIO / (self.config.TRAIN_RATIO + self.config.VAL_RATIO)
        X_train, X_val, y_train, y_val = train_test_split(
            X_temp, y_temp, test_size=val_size,
            random_state=self.config.RANDOM_SEED, shuffle=False
        )
        
        print(f"Train set: {X_train.shape[0]} samples")
        print(f"Validation set: {X_val.shape[0]} samples")
        print(f"Test set: {X_test.shape[0]} samples")
        
        return (X_train, y_train), (X_val, y_val), (X_test, y_test)
    
    def save_processed_data(self, train_data, val_data, test_data):
        """保存处理后的数据"""
        print("Saving processed data...")
        
        # 保存数据
        np.save(os.path.join(self.config.DATA_DIR, 'X_train.npy'), train_data[0])
        np.save(os.path.join(self.config.DATA_DIR, 'y_train.npy'), train_data[1])
        np.save(os.path.join(self.config.DATA_DIR, 'X_val.npy'), val_data[0])
        np.save(os.path.join(self.config.DATA_DIR, 'y_val.npy'), val_data[1])
        np.save(os.path.join(self.config.DATA_DIR, 'X_test.npy'), test_data[0])
        np.save(os.path.join(self.config.DATA_DIR, 'y_test.npy'), test_data[1])
        
        # 保存标准化器
        if self.scaler_X is not None:
            with open(os.path.join(self.config.DATA_DIR, 'scaler_X.pkl'), 'wb') as f:
                pickle.dump(self.scaler_X, f)
            with open(os.path.join(self.config.DATA_DIR, 'scaler_y.pkl'), 'wb') as f:
                pickle.dump(self.scaler_y, f)
        
        print("Data saved successfully!")
    
    def visualize_data(self, df):
        """可视化数据"""
        print("Creating data visualizations...")
        
        # 创建图形
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        
        # GPS轨迹
        axes[0, 0].plot(df['gps_east'], df['gps_north'], 'b-', alpha=0.7)
        axes[0, 0].set_xlabel('East (m)')
        axes[0, 0].set_ylabel('North (m)')
        axes[0, 0].set_title('GPS Trajectory')
        axes[0, 0].grid(True)
        
        # DVL速度
        time_idx = range(len(df))
        axes[0, 1].plot(time_idx, df['dvl_vx'], label='vx', alpha=0.7)
        axes[0, 1].plot(time_idx, df['dvl_vy'], label='vy', alpha=0.7)
        axes[0, 1].plot(time_idx, df['dvl_vz'], label='vz', alpha=0.7)
        axes[0, 1].set_xlabel('Time Index')
        axes[0, 1].set_ylabel('Velocity (m/s)')
        axes[0, 1].set_title('DVL Velocities')
        axes[0, 1].legend()
        axes[0, 1].grid(True)
        
        # IMU加速度
        axes[1, 0].plot(time_idx, df['acc_x'], label='ax', alpha=0.7)
        axes[1, 0].plot(time_idx, df['acc_y'], label='ay', alpha=0.7)
        axes[1, 0].plot(time_idx, df['acc_z'], label='az', alpha=0.7)
        axes[1, 0].set_xlabel('Time Index')
        axes[1, 0].set_ylabel('Acceleration (m/s²)')
        axes[1, 0].set_title('IMU Accelerations')
        axes[1, 0].legend()
        axes[1, 0].grid(True)
        
        # 数据统计
        feature_stats = df[self.config.INPUT_FEATURES].describe()
        axes[1, 1].axis('off')
        table_data = feature_stats.round(3).values
        table = axes[1, 1].table(cellText=table_data,
                                rowLabels=feature_stats.index,
                                colLabels=feature_stats.columns,
                                cellLoc='center',
                                loc='center')
        table.auto_set_font_size(False)
        table.set_fontsize(8)
        axes[1, 1].set_title('Feature Statistics')
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.config.RESULTS_DIR, 'data_overview.png'), dpi=300, bbox_inches='tight')
        plt.show()
    
    def process_all(self, csv_path=None):
        """完整的数据处理流程"""
        if csv_path is None:
            csv_path = self.config.INPUT_CSV
        
        # 1. 加载数据
        df = self.load_csv_data(csv_path)
        
        # 2. 可视化原始数据 (跳过以加快训练)
        print("Skipping data visualization to speed up training...")
        
        # 3. 预处理特征
        X, y = self.preprocess_features(df)
        
        # 4. 创建时间序列
        X_seq, y_seq = self.create_sequences(X, y)
        
        # 5. 分割数据
        train_data, val_data, test_data = self.split_data(X_seq, y_seq)
        
        # 6. 保存数据
        self.save_processed_data(train_data, val_data, test_data)
        
        return train_data, val_data, test_data

def main():
    parser = argparse.ArgumentParser(description='Process data for LSTM training')
    parser.add_argument('--input_csv', type=str, default=None,
                       help='Path to input CSV file')
    args = parser.parse_args()
    
    # 创建数据处理器
    processor = DataProcessor(config)
    
    # 处理数据
    train_data, val_data, test_data = processor.process_all(args.input_csv)
    
    print("Data processing completed!")

if __name__ == "__main__":
    main()
