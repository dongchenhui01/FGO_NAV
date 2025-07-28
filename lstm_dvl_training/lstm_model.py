"""
LSTM模型定义
基于论文《An Underwater Multisource Fusion Anomaly Detection Navigation Algorithm Based on Factor Graph and LSTM》
实现Figure 7的LSTM Train Model架构
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
from config import config

class LSTMDVLModel(nn.Module):
    """
    LSTM DVL模型
    按照论文Figure 7的架构：输入层 -> 中间层(3层LSTM) -> 全连接层 -> 输出层

    论文参数设置：
    - 3层LSTM网络
    - 每层单元数：128, 64, 32
    - 学习率：0.01
    - 梯度阈值：0.75
    """

    def __init__(self, input_dim, output_dim, lstm_units=None, dropout_rate=0.2):
        super(LSTMDVLModel, self).__init__()

        self.input_dim = input_dim
        self.output_dim = output_dim
        self.lstm_units = lstm_units or config.LSTM_UNITS  # [128, 64, 32]
        self.dropout_rate = dropout_rate

        # 输入层 (Input layer) - 论文Figure 7
        # 接收时间窗口内连续的状态特征
        self.input_projection = nn.Linear(input_dim, self.lstm_units[0])

        # 中间层 (Intermediate layer) - 3层LSTM堆叠
        self.lstm_layers = nn.ModuleList()

        # 第一层LSTM (128单元)
        self.lstm_layers.append(
            nn.LSTM(self.lstm_units[0], self.lstm_units[0],
                   batch_first=True, dropout=dropout_rate)
        )

        # 第二层LSTM (64单元)
        self.lstm_layers.append(
            nn.LSTM(self.lstm_units[0], self.lstm_units[1],
                   batch_first=True, dropout=dropout_rate)
        )

        # 第三层LSTM (32单元)
        self.lstm_layers.append(
            nn.LSTM(self.lstm_units[1], self.lstm_units[2],
                   batch_first=True)
        )

        # Dropout层
        self.dropout = nn.Dropout(dropout_rate)

        # 全连接层 (Fully connected layer) - 论文Figure 7
        # 整合LSTM学习到的高级特征，映射到输出维度
        self.fc_layers = nn.Sequential(
            nn.Linear(self.lstm_units[-1], self.lstm_units[-1]),
            nn.ReLU(),
            nn.Dropout(dropout_rate),
            nn.Linear(self.lstm_units[-1], output_dim)
        )

        # 初始化权重
        self._init_weights()
    
    def _init_weights(self):
        """初始化模型权重"""
        for name, param in self.named_parameters():
            if 'weight_ih' in name:
                # LSTM输入权重使用Xavier初始化
                nn.init.xavier_uniform_(param.data)
            elif 'weight_hh' in name:
                # LSTM隐藏权重使用正交初始化
                nn.init.orthogonal_(param.data)
            elif 'bias' in name:
                # 偏置初始化为0
                param.data.fill_(0.)
                # LSTM遗忘门偏置设为1
                if 'bias_ih' in name:
                    n = param.size(0)
                    param.data[n//4:n//2].fill_(1.)
            elif 'weight' in name and len(param.shape) == 2:
                # 全连接层权重使用Xavier初始化
                nn.init.xavier_uniform_(param.data)
    
    def forward(self, x):
        """
        前向传播
        
        Args:
            x: 输入张量，形状为 (batch_size, sequence_length, input_dim)
        
        Returns:
            output: 输出张量，形状为 (batch_size, output_dim)
        """
        batch_size, seq_len, _ = x.size()
        
        # 输入层处理 - 将每个时间步的特征投影到LSTM输入空间
        x_projected = []
        for t in range(seq_len):
            x_t = self.input_projection(x[:, t, :])  # (batch_size, lstm_units[0])
            x_projected.append(x_t.unsqueeze(1))

        x_processed = torch.cat(x_projected, dim=1)  # (batch_size, seq_len, lstm_units[0])
        x_processed = F.relu(x_processed)
        
        # 中间层 - 逐层通过LSTM
        lstm_out = x_processed
        
        for i, lstm_layer in enumerate(self.lstm_layers):
            # LSTM前向传播
            lstm_out, (hidden, cell) = lstm_layer(lstm_out)
            
            # 在非最后一层添加dropout
            if i < len(self.lstm_layers) - 1:
                lstm_out = self.dropout(lstm_out)
        
        # 取最后一个时间步的输出
        # lstm_out形状: (batch_size, seq_len, lstm_units[-1])
        last_output = lstm_out[:, -1, :]  # (batch_size, lstm_units[-1])
        
        # 全连接层
        output = self.fc_layers(last_output)
        
        return output
    
    def get_model_info(self):
        """获取模型信息"""
        total_params = sum(p.numel() for p in self.parameters())
        trainable_params = sum(p.numel() for p in self.parameters() if p.requires_grad)
        
        info = {
            'total_parameters': total_params,
            'trainable_parameters': trainable_params,
            'input_dim': self.input_dim,
            'output_dim': self.output_dim,
            'lstm_units': self.lstm_units,
            'dropout_rate': self.dropout_rate
        }
        
        return info
    
    def print_model_info(self):
        """打印模型信息"""
        info = self.get_model_info()
        print("=" * 50)
        print("LSTM DVL Model Information")
        print("=" * 50)
        print(f"Input dimension: {info['input_dim']}")
        print(f"Output dimension: {info['output_dim']}")
        print(f"LSTM units: {info['lstm_units']}")
        print(f"Dropout rate: {info['dropout_rate']}")
        print(f"Total parameters: {info['total_parameters']:,}")
        print(f"Trainable parameters: {info['trainable_parameters']:,}")
        print("=" * 50)

class LSTMDVLLoss(nn.Module):
    """
    自定义损失函数
    可以组合多种损失函数
    """
    
    def __init__(self, loss_type='mse', weights=None):
        super(LSTMDVLLoss, self).__init__()
        self.loss_type = loss_type
        self.weights = weights
        
        if loss_type == 'mse':
            self.criterion = nn.MSELoss()
        elif loss_type == 'mae':
            self.criterion = nn.L1Loss()
        elif loss_type == 'huber':
            self.criterion = nn.SmoothL1Loss()
        elif loss_type == 'combined':
            self.mse_loss = nn.MSELoss()
            self.mae_loss = nn.L1Loss()
        else:
            raise ValueError(f"Unsupported loss type: {loss_type}")
    
    def forward(self, predictions, targets):
        """
        计算损失
        
        Args:
            predictions: 模型预测，形状为 (batch_size, output_dim)
            targets: 真实标签，形状为 (batch_size, output_dim)
        
        Returns:
            loss: 损失值
        """
        if self.loss_type == 'combined':
            mse = self.mse_loss(predictions, targets)
            mae = self.mae_loss(predictions, targets)
            loss = 0.7 * mse + 0.3 * mae
        else:
            loss = self.criterion(predictions, targets)
        
        # 如果有权重，应用权重
        if self.weights is not None:
            if len(self.weights) == predictions.size(-1):
                weights = torch.tensor(self.weights, device=predictions.device)
                loss = loss * weights.unsqueeze(0)
                loss = loss.mean()
        
        return loss

def create_model(input_dim=None, output_dim=None):
    """
    创建LSTM模型的工厂函数
    
    Args:
        input_dim: 输入维度，如果为None则从config获取
        output_dim: 输出维度，如果为None则从config获取
    
    Returns:
        model: LSTM模型实例
    """
    if input_dim is None:
        input_dim = config.get_input_dim()
    if output_dim is None:
        output_dim = config.get_output_dim()
    
    model = LSTMDVLModel(
        input_dim=input_dim,
        output_dim=output_dim,
        lstm_units=config.LSTM_UNITS,
        dropout_rate=config.DROPOUT_RATE
    )
    
    return model

def create_loss_function(loss_type=None):
    """
    创建损失函数的工厂函数
    
    Args:
        loss_type: 损失函数类型，如果为None则从config获取
    
    Returns:
        loss_fn: 损失函数实例
    """
    if loss_type is None:
        loss_type = config.LOSS_FUNCTION
    
    return LSTMDVLLoss(loss_type=loss_type)

# 测试代码
if __name__ == "__main__":
    # 创建模型
    model = create_model()
    model.print_model_info()
    
    # 测试前向传播
    batch_size = 4
    seq_len = config.SEQUENCE_LENGTH
    input_dim = config.get_input_dim()
    
    # 创建随机输入
    x = torch.randn(batch_size, seq_len, input_dim)
    
    # 前向传播
    with torch.no_grad():
        output = model(x)
    
    print(f"Input shape: {x.shape}")
    print(f"Output shape: {output.shape}")
    print("Model test passed!")
