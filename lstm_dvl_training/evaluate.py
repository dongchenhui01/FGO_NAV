"""
LSTM DVL模型评估脚本
"""

import os
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset
import matplotlib.pyplot as plt
import pickle
import argparse
from datetime import datetime

from config import config
from lstm_model import create_model
import utils

class LSTMEvaluator:
    def __init__(self, config, model_path):
        self.config = config
        self.model_path = model_path
        self.device = torch.device('cuda' if torch.cuda.is_available() and config.DEVICE == 'cuda' else 'cpu')
        print(f"Using device: {self.device}")
        
        # 加载模型
        self.model = self.load_model()
        
        # 加载标准化器
        self.scaler_X = None
        self.scaler_y = None
        self.load_scalers()
    
    def load_model(self):
        """加载训练好的模型"""
        print(f"Loading model from {self.model_path}")
        
        # 创建模型
        model = create_model().to(self.device)
        
        # 加载权重
        checkpoint = torch.load(self.model_path, map_location=self.device)
        model.load_state_dict(checkpoint['model_state_dict'])
        
        model.eval()
        print("Model loaded successfully!")
        
        return model
    
    def load_scalers(self):
        """加载数据标准化器"""
        scaler_X_path = os.path.join(self.config.DATA_DIR, 'scaler_X.pkl')
        scaler_y_path = os.path.join(self.config.DATA_DIR, 'scaler_y.pkl')
        
        if os.path.exists(scaler_X_path):
            self.scaler_X = utils.load_scaler(scaler_X_path)
            print("Input scaler loaded")
        
        if os.path.exists(scaler_y_path):
            self.scaler_y = utils.load_scaler(scaler_y_path)
            print("Output scaler loaded")
    
    def load_test_data(self):
        """加载测试数据"""
        print("Loading test data...")
        
        X_test = np.load(os.path.join(self.config.DATA_DIR, 'X_test.npy'))
        y_test = np.load(os.path.join(self.config.DATA_DIR, 'y_test.npy'))
        
        # 转换为PyTorch张量
        X_test_tensor = torch.FloatTensor(X_test)
        y_test_tensor = torch.FloatTensor(y_test)
        
        # 创建数据加载器
        test_dataset = TensorDataset(X_test_tensor, y_test_tensor)
        test_loader = DataLoader(test_dataset, batch_size=self.config.BATCH_SIZE, shuffle=False)
        
        print(f"Test data loaded: {len(X_test)} samples")
        
        return test_loader, (X_test, y_test)
    
    def predict(self, test_loader):
        """进行预测"""
        print("Making predictions...")
        
        all_predictions = []
        all_targets = []
        
        self.model.eval()
        with torch.no_grad():
            for data, target in test_loader:
                data, target = data.to(self.device), target.to(self.device)
                
                # 预测
                output = self.model(data)
                
                all_predictions.append(output.cpu())
                all_targets.append(target.cpu())
        
        # 合并所有批次的结果
        predictions = torch.cat(all_predictions, dim=0)
        targets = torch.cat(all_targets, dim=0)
        
        print(f"Predictions completed: {len(predictions)} samples")
        
        return predictions, targets
    
    def denormalize_predictions(self, predictions, targets):
        """反标准化预测结果"""
        if self.scaler_y is not None:
            print("Denormalizing predictions...")
            predictions_denorm = utils.denormalize_data(predictions, self.scaler_y)
            targets_denorm = utils.denormalize_data(targets, self.scaler_y)
            return predictions_denorm, targets_denorm
        else:
            print("No output scaler found, using normalized data")
            return predictions, targets
    
    def evaluate_model(self):
        """完整的模型评估流程"""
        print("Starting model evaluation...")
        
        # 加载测试数据
        test_loader, (X_test, y_test) = self.load_test_data()
        
        # 进行预测
        predictions, targets = self.predict(test_loader)
        
        # 反标准化
        predictions_denorm, targets_denorm = self.denormalize_predictions(predictions, targets)
        
        # 计算评估指标
        print("\nCalculating metrics...")
        metrics = utils.calculate_metrics(predictions_denorm, targets_denorm)
        utils.print_metrics(metrics)
        
        # 可视化结果
        print("\nGenerating final visualizations...")

        # 预测结果对比
        print("Creating prediction comparison plots...")
        utils.plot_predictions(
            predictions_denorm, targets_denorm,
            save_path=os.path.join(self.config.RESULTS_DIR, 'prediction_comparison.png'),
            title="LSTM DVL Model Predictions vs Ground Truth"
        )

        # 误差分析
        print("Creating error analysis plots...")
        utils.plot_error_analysis(
            predictions_denorm, targets_denorm,
            save_path=os.path.join(self.config.RESULTS_DIR, 'error_analysis.png')
        )
        
        # 保存结果
        results = {
            'metrics': metrics,
            'model_info': self.model.get_model_info(),
            'test_samples': len(predictions),
            'evaluation_date': datetime.now().isoformat()
        }
        
        results_path = os.path.join(self.config.RESULTS_DIR, 'evaluation_results.json')
        utils.save_results(results, results_path)
        
        # 创建总结报告
        report_path = os.path.join(self.config.RESULTS_DIR, 'evaluation_report.json')
        report = utils.create_summary_report(metrics, self.model.get_model_info(), self.config, report_path)
        
        print("\nEvaluation completed!")
        print(f"Results saved to: {self.config.RESULTS_DIR}")
        
        return results
    
    def predict_sequence(self, input_sequence):
        """
        对单个输入序列进行预测
        
        Args:
            input_sequence: 输入序列 (sequence_length, input_dim)
        
        Returns:
            prediction: 预测的下一时刻位置 (output_dim,)
        """
        self.model.eval()
        
        # 转换为张量并添加batch维度
        if not torch.is_tensor(input_sequence):
            input_sequence = torch.FloatTensor(input_sequence)
        
        input_sequence = input_sequence.unsqueeze(0).to(self.device)  # (1, seq_len, input_dim)
        
        with torch.no_grad():
            prediction = self.model(input_sequence)
        
        prediction = prediction.squeeze(0).cpu()  # 移除batch维度
        
        # 反标准化
        if self.scaler_y is not None:
            prediction = utils.denormalize_data(prediction.unsqueeze(0), self.scaler_y).squeeze(0)
        
        return prediction.numpy()
    
    def analyze_feature_importance(self, test_loader, num_samples=100):
        """
        分析特征重要性（简单的扰动分析）
        
        Args:
            test_loader: 测试数据加载器
            num_samples: 分析的样本数量
        """
        print("Analyzing feature importance...")
        
        self.model.eval()
        feature_names = self.config.INPUT_FEATURES
        importance_scores = np.zeros(len(feature_names))
        
        sample_count = 0
        with torch.no_grad():
            for data, target in test_loader:
                if sample_count >= num_samples:
                    break
                
                data, target = data.to(self.device), target.to(self.device)
                batch_size = data.size(0)
                
                # 原始预测
                original_pred = self.model(data)
                original_loss = nn.MSELoss()(original_pred, target)
                
                # 对每个特征进行扰动
                for feat_idx in range(data.size(-1)):
                    # 创建扰动数据
                    perturbed_data = data.clone()
                    # 将该特征设为0（或添加噪声）
                    perturbed_data[:, :, feat_idx] = 0
                    
                    # 扰动后的预测
                    perturbed_pred = self.model(perturbed_data)
                    perturbed_loss = nn.MSELoss()(perturbed_pred, target)
                    
                    # 计算重要性分数（损失增加量）
                    importance_scores[feat_idx] += (perturbed_loss - original_loss).item()
                
                sample_count += batch_size
        
        # 归一化重要性分数
        importance_scores /= sample_count
        
        # 可视化特征重要性
        plt.figure(figsize=(12, 8))
        sorted_indices = np.argsort(importance_scores)[::-1]
        
        plt.barh(range(len(feature_names)), importance_scores[sorted_indices])
        plt.yticks(range(len(feature_names)), [feature_names[i] for i in sorted_indices])
        plt.xlabel('Importance Score (Loss Increase)')
        plt.title('Feature Importance Analysis')
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        
        importance_path = os.path.join(self.config.RESULTS_DIR, 'feature_importance.png')
        plt.savefig(importance_path, dpi=300, bbox_inches='tight')
        plt.show()
        
        # 保存重要性分数
        importance_results = {
            'feature_names': feature_names,
            'importance_scores': importance_scores.tolist(),
            'sorted_features': [(feature_names[i], importance_scores[i]) for i in sorted_indices]
        }
        
        importance_json_path = os.path.join(self.config.RESULTS_DIR, 'feature_importance.json')
        utils.save_results(importance_results, importance_json_path)
        
        print("Feature importance analysis completed!")
        
        return importance_results

def main():
    parser = argparse.ArgumentParser(description='Evaluate LSTM DVL model')
    parser.add_argument('--model_path', type=str, required=True,
                       help='Path to trained model checkpoint')
    parser.add_argument('--analyze_features', action='store_true',
                       help='Perform feature importance analysis')
    args = parser.parse_args()
    
    # 检查模型文件是否存在
    if not os.path.exists(args.model_path):
        print(f"Model file not found: {args.model_path}")
        return
    
    # 创建评估器
    evaluator = LSTMEvaluator(config, args.model_path)
    
    # 进行评估
    results = evaluator.evaluate_model()
    
    # 特征重要性分析（可选）
    if args.analyze_features:
        test_loader, _ = evaluator.load_test_data()
        importance_results = evaluator.analyze_feature_importance(test_loader)

if __name__ == "__main__":
    main()
