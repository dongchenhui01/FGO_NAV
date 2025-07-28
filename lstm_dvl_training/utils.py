"""
工具函数
用于LSTM DVL训练的辅助功能
"""

import numpy as np
import torch
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.metrics import mean_squared_error, mean_absolute_error
import pickle
import os

def calculate_metrics(predictions, targets):
    """
    计算评估指标
    
    Args:
        predictions: 预测值 (N, 3) - [east, north, up]
        targets: 真实值 (N, 3) - [east, north, up]
    
    Returns:
        metrics: 包含各种指标的字典
    """
    # 转换为numpy数组
    if torch.is_tensor(predictions):
        predictions = predictions.cpu().numpy()
    if torch.is_tensor(targets):
        targets = targets.cpu().numpy()
    
    # 计算各维度的指标
    metrics = {}
    
    # 整体指标
    mse = mean_squared_error(targets, predictions)
    rmse = np.sqrt(mse)
    mae = mean_absolute_error(targets, predictions)
    
    metrics['overall'] = {
        'mse': mse,
        'rmse': rmse,
        'mae': mae
    }
    
    # 各维度指标
    dimensions = ['east', 'north', 'up']
    for i, dim in enumerate(dimensions):
        if predictions.shape[1] > i and targets.shape[1] > i:
            dim_mse = mean_squared_error(targets[:, i], predictions[:, i])
            dim_rmse = np.sqrt(dim_mse)
            dim_mae = mean_absolute_error(targets[:, i], predictions[:, i])
            
            metrics[dim] = {
                'mse': dim_mse,
                'rmse': dim_rmse,
                'mae': dim_mae
            }
    
    # 2D水平误差（东北方向）
    if predictions.shape[1] >= 2 and targets.shape[1] >= 2:
        horizontal_error = np.sqrt((predictions[:, 0] - targets[:, 0])**2 + 
                                 (predictions[:, 1] - targets[:, 1])**2)
        metrics['horizontal'] = {
            'mean_error': np.mean(horizontal_error),
            'std_error': np.std(horizontal_error),
            'max_error': np.max(horizontal_error),
            'percentile_95': np.percentile(horizontal_error, 95)
        }
    
    return metrics

def print_metrics(metrics):
    """打印评估指标"""
    print("=" * 60)
    print("LSTM DVL Model Evaluation Metrics")
    print("=" * 60)
    
    # 整体指标
    if 'overall' in metrics:
        print("Overall Performance:")
        print(f"  RMSE: {metrics['overall']['rmse']:.4f} m")
        print(f"  MAE:  {metrics['overall']['mae']:.4f} m")
        print(f"  MSE:  {metrics['overall']['mse']:.4f} m²")
        print()
    
    # 各维度指标
    dimensions = ['east', 'north', 'up']
    for dim in dimensions:
        if dim in metrics:
            print(f"{dim.capitalize()} Direction:")
            print(f"  RMSE: {metrics[dim]['rmse']:.4f} m")
            print(f"  MAE:  {metrics[dim]['mae']:.4f} m")
            print(f"  MSE:  {metrics[dim]['mse']:.4f} m²")
            print()
    
    # 水平误差
    if 'horizontal' in metrics:
        print("Horizontal (2D) Error:")
        print(f"  Mean:  {metrics['horizontal']['mean_error']:.4f} m")
        print(f"  Std:   {metrics['horizontal']['std_error']:.4f} m")
        print(f"  Max:   {metrics['horizontal']['max_error']:.4f} m")
        print(f"  95th:  {metrics['horizontal']['percentile_95']:.4f} m")
        print()
    
    print("=" * 60)

def plot_predictions(predictions, targets, save_path=None, title="LSTM DVL Predictions"):
    """
    可视化预测结果
    
    Args:
        predictions: 预测值 (N, 3)
        targets: 真实值 (N, 3)
        save_path: 保存路径
        title: 图表标题
    """
    # 转换为numpy数组
    if torch.is_tensor(predictions):
        predictions = predictions.cpu().numpy()
    if torch.is_tensor(targets):
        targets = targets.cpu().numpy()
    
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    
    # 轨迹对比（2D）
    axes[0, 0].plot(targets[:, 0], targets[:, 1], 'b-', label='True Trajectory', alpha=0.7)
    axes[0, 0].plot(predictions[:, 0], predictions[:, 1], 'r--', label='Predicted Trajectory', alpha=0.7)
    axes[0, 0].set_xlabel('East (m)')
    axes[0, 0].set_ylabel('North (m)')
    axes[0, 0].set_title('2D Trajectory Comparison')
    axes[0, 0].legend()
    axes[0, 0].grid(True)
    axes[0, 0].axis('equal')
    
    # 时间序列对比
    time_steps = range(len(predictions))
    
    # East方向
    axes[0, 1].plot(time_steps, targets[:, 0], 'b-', label='True', alpha=0.7)
    axes[0, 1].plot(time_steps, predictions[:, 0], 'r--', label='Predicted', alpha=0.7)
    axes[0, 1].set_xlabel('Time Step')
    axes[0, 1].set_ylabel('East Position (m)')
    axes[0, 1].set_title('East Direction Comparison')
    axes[0, 1].legend()
    axes[0, 1].grid(True)
    
    # North方向
    axes[1, 0].plot(time_steps, targets[:, 1], 'b-', label='True', alpha=0.7)
    axes[1, 0].plot(time_steps, predictions[:, 1], 'r--', label='Predicted', alpha=0.7)
    axes[1, 0].set_xlabel('Time Step')
    axes[1, 0].set_ylabel('North Position (m)')
    axes[1, 0].set_title('North Direction Comparison')
    axes[1, 0].legend()
    axes[1, 0].grid(True)
    
    # 误差分布
    if predictions.shape[1] >= 2 and targets.shape[1] >= 2:
        horizontal_error = np.sqrt((predictions[:, 0] - targets[:, 0])**2 + 
                                 (predictions[:, 1] - targets[:, 1])**2)
        axes[1, 1].hist(horizontal_error, bins=50, alpha=0.7, edgecolor='black')
        axes[1, 1].set_xlabel('Horizontal Error (m)')
        axes[1, 1].set_ylabel('Frequency')
        axes[1, 1].set_title('Horizontal Error Distribution')
        axes[1, 1].grid(True)
        
        # 添加统计信息
        mean_error = np.mean(horizontal_error)
        std_error = np.std(horizontal_error)
        axes[1, 1].axvline(mean_error, color='red', linestyle='--', 
                          label=f'Mean: {mean_error:.3f}m')
        axes[1, 1].axvline(mean_error + std_error, color='orange', linestyle='--', 
                          label=f'Mean+Std: {mean_error + std_error:.3f}m')
        axes[1, 1].legend()
    
    plt.suptitle(title, fontsize=16)
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    
    plt.show()

def plot_error_analysis(predictions, targets, save_path=None):
    """
    详细的误差分析图
    
    Args:
        predictions: 预测值 (N, 3)
        targets: 真实值 (N, 3)
        save_path: 保存路径
    """
    # 转换为numpy数组
    if torch.is_tensor(predictions):
        predictions = predictions.cpu().numpy()
    if torch.is_tensor(targets):
        targets = targets.cpu().numpy()
    
    # 计算误差
    errors = predictions - targets
    
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    
    dimensions = ['East', 'North', 'Up']
    
    # 误差时间序列
    for i, dim in enumerate(dimensions):
        if i < errors.shape[1]:
            axes[0, i].plot(errors[:, i], alpha=0.7)
            axes[0, i].set_xlabel('Time Step')
            axes[0, i].set_ylabel(f'{dim} Error (m)')
            axes[0, i].set_title(f'{dim} Direction Error Over Time')
            axes[0, i].grid(True)
            axes[0, i].axhline(y=0, color='red', linestyle='--', alpha=0.5)
    
    # 误差分布
    for i, dim in enumerate(dimensions):
        if i < errors.shape[1]:
            axes[1, i].hist(errors[:, i], bins=50, alpha=0.7, edgecolor='black')
            axes[1, i].set_xlabel(f'{dim} Error (m)')
            axes[1, i].set_ylabel('Frequency')
            axes[1, i].set_title(f'{dim} Error Distribution')
            axes[1, i].grid(True)
            axes[1, i].axvline(x=0, color='red', linestyle='--', alpha=0.5)
            
            # 添加统计信息
            mean_err = np.mean(errors[:, i])
            std_err = np.std(errors[:, i])
            axes[1, i].axvline(mean_err, color='orange', linestyle='--', 
                              label=f'Mean: {mean_err:.3f}m')
            axes[1, i].legend()
    
    plt.suptitle('LSTM DVL Prediction Error Analysis', fontsize=16)
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    
    plt.show()

def load_scaler(scaler_path):
    """加载数据标准化器"""
    with open(scaler_path, 'rb') as f:
        scaler = pickle.load(f)
    return scaler

def denormalize_data(data, scaler):
    """反标准化数据"""
    if torch.is_tensor(data):
        data_np = data.cpu().numpy()
        denorm_data = scaler.inverse_transform(data_np)
        return torch.from_numpy(denorm_data).to(data.device)
    else:
        return scaler.inverse_transform(data)

def save_results(results, save_path):
    """保存结果到文件"""
    import json
    
    # 转换numpy数组为列表以便JSON序列化
    def convert_numpy(obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, dict):
            return {key: convert_numpy(value) for key, value in obj.items()}
        elif isinstance(obj, list):
            return [convert_numpy(item) for item in obj]
        else:
            return obj
    
    results_serializable = convert_numpy(results)
    
    with open(save_path, 'w') as f:
        json.dump(results_serializable, f, indent=2)
    
    print(f"Results saved to {save_path}")

def create_summary_report(metrics, model_info, config, save_path=None):
    """创建训练总结报告"""
    report = {
        'model_info': model_info,
        'training_config': {
            'lstm_units': config.LSTM_UNITS,
            'learning_rate': config.INITIAL_LEARNING_RATE,
            'gradient_clip': config.GRADIENT_CLIP_VALUE,
            'batch_size': config.BATCH_SIZE,
            'sequence_length': config.SEQUENCE_LENGTH
        },
        'performance_metrics': metrics,
        'paper_reference': "An Underwater Multisource Fusion Anomaly Detection Navigation Algorithm Based on Factor Graph and LSTM"
    }
    
    if save_path:
        save_results(report, save_path)
    
    return report
