"""
LSTM DVL训练脚本
基于论文《An Underwater Multisource Fusion Anomaly Detection Navigation Algorithm Based on Factor Graph and LSTM》
"""

import os
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset
import matplotlib.pyplot as plt
import pickle
import argparse
from datetime import datetime
import json

from config import config
from lstm_model import create_model, create_loss_function
from data_processor import DataProcessor
import utils

class LSTMTrainer:
    def __init__(self, config):
        self.config = config
        self.device = torch.device('cuda' if torch.cuda.is_available() and config.DEVICE == 'cuda' else 'cpu')
        print(f"Using device: {self.device}")
        
        # 设置随机种子
        torch.manual_seed(config.RANDOM_SEED)
        np.random.seed(config.RANDOM_SEED)
        
        # 初始化模型、损失函数和优化器
        self.model = None
        self.criterion = None
        self.optimizer = None
        self.scheduler = None
        
        # 训练历史
        self.train_history = {
            'train_loss': [],
            'val_loss': [],
            'learning_rate': []
        }
        
        # 最佳模型状态
        self.best_val_loss = float('inf')
        self.best_model_state = None
        self.patience_counter = 0
    
    def setup_model(self):
        """设置模型、损失函数和优化器"""
        print("Setting up model...")
        
        # 创建模型
        self.model = create_model().to(self.device)
        self.model.print_model_info()
        
        # 创建损失函数
        self.criterion = create_loss_function()
        
        # 创建优化器 - 按照论文设置
        self.optimizer = optim.Adam(
            self.model.parameters(),
            lr=self.config.INITIAL_LEARNING_RATE,  # 0.01
            weight_decay=1e-5
        )
        
        # 创建学习率调度器
        if self.config.LR_SCHEDULER['type'] == 'ReduceLROnPlateau':
            self.scheduler = optim.lr_scheduler.ReduceLROnPlateau(
                self.optimizer,
                mode='min',
                factor=self.config.LR_SCHEDULER['factor'],
                patience=self.config.LR_SCHEDULER['patience'],
                min_lr=self.config.LR_SCHEDULER['min_lr'],
                verbose=True
            )
        
        # 梯度裁剪 - 按照论文设置梯度阈值0.75
        self.gradient_clip_value = self.config.GRADIENT_CLIP_VALUE
    
    def load_data(self):
        """加载训练数据"""
        print("Loading training data...")
        
        # 检查是否存在处理后的数据
        data_files = [
            'X_train.npy', 'y_train.npy',
            'X_val.npy', 'y_val.npy',
            'X_test.npy', 'y_test.npy'
        ]
        
        data_exist = all(os.path.exists(os.path.join(self.config.DATA_DIR, f)) for f in data_files)
        
        if not data_exist:
            print("Processed data not found. Processing raw data...")
            processor = DataProcessor(self.config)
            train_data, val_data, test_data = processor.process_all()
        else:
            print("Loading existing processed data...")
            # 加载数据
            X_train = np.load(os.path.join(self.config.DATA_DIR, 'X_train.npy'))
            y_train = np.load(os.path.join(self.config.DATA_DIR, 'y_train.npy'))
            X_val = np.load(os.path.join(self.config.DATA_DIR, 'X_val.npy'))
            y_val = np.load(os.path.join(self.config.DATA_DIR, 'y_val.npy'))
            X_test = np.load(os.path.join(self.config.DATA_DIR, 'X_test.npy'))
            y_test = np.load(os.path.join(self.config.DATA_DIR, 'y_test.npy'))
            
            train_data = (X_train, y_train)
            val_data = (X_val, y_val)
            test_data = (X_test, y_test)
        
        # 转换为PyTorch张量
        X_train_tensor = torch.FloatTensor(train_data[0])
        y_train_tensor = torch.FloatTensor(train_data[1])
        X_val_tensor = torch.FloatTensor(val_data[0])
        y_val_tensor = torch.FloatTensor(val_data[1])
        X_test_tensor = torch.FloatTensor(test_data[0])
        y_test_tensor = torch.FloatTensor(test_data[1])
        
        # 创建数据加载器
        train_dataset = TensorDataset(X_train_tensor, y_train_tensor)
        val_dataset = TensorDataset(X_val_tensor, y_val_tensor)
        test_dataset = TensorDataset(X_test_tensor, y_test_tensor)
        
        self.train_loader = DataLoader(
            train_dataset, batch_size=self.config.BATCH_SIZE, shuffle=True
        )
        self.val_loader = DataLoader(
            val_dataset, batch_size=self.config.BATCH_SIZE, shuffle=False
        )
        self.test_loader = DataLoader(
            test_dataset, batch_size=self.config.BATCH_SIZE, shuffle=False
        )
        
        print(f"Data loaded successfully!")
        print(f"Train batches: {len(self.train_loader)}")
        print(f"Validation batches: {len(self.val_loader)}")
        print(f"Test batches: {len(self.test_loader)}")
        
        return train_data, val_data, test_data
    
    def train_epoch(self):
        """训练一个epoch"""
        self.model.train()
        total_loss = 0.0
        num_batches = 0
        
        for batch_idx, (data, target) in enumerate(self.train_loader):
            data, target = data.to(self.device), target.to(self.device)
            
            # 前向传播
            self.optimizer.zero_grad()
            output = self.model(data)
            loss = self.criterion(output, target)
            
            # 反向传播
            loss.backward()
            
            # 梯度裁剪 - 论文设置的梯度阈值0.75
            torch.nn.utils.clip_grad_norm_(self.model.parameters(), self.gradient_clip_value)
            
            # 更新参数
            self.optimizer.step()
            
            total_loss += loss.item()
            num_batches += 1
            
            # 打印进度 - 每10个batch输出一次
            if batch_idx % 10 == 0:
                print(f'  Batch {batch_idx}/{len(self.train_loader)}, Loss: {loss.item():.6f}')
        
        avg_loss = total_loss / num_batches
        return avg_loss
    
    def validate_epoch(self):
        """验证一个epoch"""
        self.model.eval()
        total_loss = 0.0
        num_batches = 0
        
        with torch.no_grad():
            for data, target in self.val_loader:
                data, target = data.to(self.device), target.to(self.device)
                output = self.model(data)
                loss = self.criterion(output, target)
                total_loss += loss.item()
                num_batches += 1
        
        avg_loss = total_loss / num_batches
        return avg_loss
    
    def save_checkpoint(self, epoch, is_best=False):
        """保存模型检查点"""
        checkpoint = {
            'epoch': epoch,
            'model_state_dict': self.model.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'scheduler_state_dict': self.scheduler.state_dict() if self.scheduler else None,
            'train_history': self.train_history,
            'best_val_loss': self.best_val_loss,
            'config': self.config.__dict__
        }
        
        # 保存最新检查点
        checkpoint_path = os.path.join(self.config.MODEL_DIR, f'checkpoint_epoch_{epoch}.pth')
        torch.save(checkpoint, checkpoint_path)
        
        # 保存最佳模型
        if is_best:
            best_path = os.path.join(self.config.MODEL_DIR, 'best_model.pth')
            torch.save(checkpoint, best_path)
            print(f"New best model saved with validation loss: {self.best_val_loss:.6f}")
    
    def train(self):
        """完整的训练流程"""
        print("Starting LSTM DVL training...")
        print(f"Target iterations: 6,940 (as per paper)")
        
        # 设置模型
        self.setup_model()
        
        # 加载数据
        self.load_data()
        
        # 训练循环
        for epoch in range(self.config.MAX_EPOCHS):
            print(f"\n{'='*60}")
            print(f"EPOCH {epoch+1}/{self.config.MAX_EPOCHS}")
            print(f"{'='*60}")
            
            # 训练
            train_loss = self.train_epoch()
            
            # 验证
            val_loss = self.validate_epoch()
            
            # 更新学习率
            if self.scheduler:
                self.scheduler.step(val_loss)
            
            # 记录历史
            self.train_history['train_loss'].append(train_loss)
            self.train_history['val_loss'].append(val_loss)
            self.train_history['learning_rate'].append(self.optimizer.param_groups[0]['lr'])
            
            # 检查是否是最佳模型
            is_best = val_loss < self.best_val_loss

            # 打印详细结果
            print(f"  Train Loss: {train_loss:.6f}")
            print(f"  Val Loss: {val_loss:.6f}")
            print(f"  Learning Rate: {self.optimizer.param_groups[0]['lr']:.8f}")

            # 显示改进情况
            if is_best:
                print(f"  *** NEW BEST MODEL *** (Previous best: {self.best_val_loss:.6f})")
            else:
                print(f"  No improvement for {self.patience_counter} epochs")
            if is_best:
                self.best_val_loss = val_loss
                self.best_model_state = self.model.state_dict().copy()
                self.patience_counter = 0
            else:
                self.patience_counter += 1
            
            # 保存检查点
            if (epoch + 1) % self.config.SAVE_CHECKPOINT_EVERY == 0 or is_best:
                self.save_checkpoint(epoch + 1, is_best)
            
            # 早停检查
            if self.patience_counter >= self.config.EARLY_STOPPING_PATIENCE:
                print(f"Early stopping triggered after {epoch+1} epochs")
                break
        
        # 保存最终模型
        self.save_checkpoint(epoch + 1, False)
        
        # 训练完成后绘制训练曲线
        print("Generating training curves...")
        if self.config.PLOT_TRAINING_CURVES:
            self.plot_training_curves()
        
        print("Training completed!")
        print(f"Best validation loss: {self.best_val_loss:.6f}")
    
    def plot_training_curves(self):
        """绘制训练曲线"""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 5))
        
        # 损失曲线
        epochs = range(1, len(self.train_history['train_loss']) + 1)
        ax1.plot(epochs, self.train_history['train_loss'], 'b-', label='Training Loss')
        ax1.plot(epochs, self.train_history['val_loss'], 'r-', label='Validation Loss')
        ax1.set_xlabel('Epoch')
        ax1.set_ylabel('Loss')
        ax1.set_title('Training and Validation Loss')
        ax1.legend()
        ax1.grid(True)
        
        # 学习率曲线
        ax2.plot(epochs, self.train_history['learning_rate'], 'g-')
        ax2.set_xlabel('Epoch')
        ax2.set_ylabel('Learning Rate')
        ax2.set_title('Learning Rate Schedule')
        ax2.set_yscale('log')
        ax2.grid(True)
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.config.RESULTS_DIR, 'training_curves.png'), dpi=300, bbox_inches='tight')
        plt.show()

def main():
    parser = argparse.ArgumentParser(description='Train LSTM DVL model')
    parser.add_argument('--config', type=str, default='config.py',
                       help='Path to config file')
    parser.add_argument('--resume', type=str, default=None,
                       help='Path to checkpoint to resume from')
    args = parser.parse_args()
    
    # 打印配置
    config.print_config()
    
    # 创建训练器
    trainer = LSTMTrainer(config)
    
    # 开始训练
    trainer.train()

if __name__ == "__main__":
    main()
