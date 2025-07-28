#!/bin/bash

echo "Installing LSTM DVL Training Dependencies..."
echo "============================================="

# 检查Python版本
python3 --version

# 安装基础依赖
echo "Installing basic dependencies..."
pip3 install numpy pandas matplotlib seaborn scikit-learn

# 安装PyTorch (CPU版本，如果需要GPU版本请手动安装)
echo "Installing PyTorch (CPU version)..."
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu

echo "Installation completed!"
echo "To verify installation, run: python3 lstm_model.py"
