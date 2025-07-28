# 贡献指南

感谢您对 LSTM_TimeFGO_nav 项目的关注！我们欢迎所有形式的贡献。

## 🚀 如何贡献

### 1. Fork 项目
1. 在 GitHub 上 Fork 本项目
2. 克隆您的 Fork 到本地：
```bash
git clone https://github.com/YOUR_USERNAME/LSTM_TimeFGO_nav.git
cd LSTM_TimeFGO_nav
```

### 2. 创建功能分支
```bash
git checkout -b feature/your-feature-name
```

### 3. 进行修改
- 编写代码
- 添加测试
- 更新文档
- 确保代码符合项目规范

### 4. 提交更改
```bash
git add .
git commit -m "feat: add your feature description"
```

### 5. 推送分支
```bash
git push origin feature/your-feature-name
```

### 6. 创建 Pull Request
在 GitHub 上创建 Pull Request，详细描述您的更改。

## 📋 贡献类型

### 🐛 Bug 修复
- 修复现有功能中的错误
- 提供详细的错误描述和复现步骤

### ✨ 新功能
- 添加新的算法或功能
- 改进现有功能
- 添加新的传感器支持

### 📚 文档改进
- 更新 README 文件
- 添加代码注释
- 编写使用教程

### 🧪 测试
- 添加单元测试
- 改进测试覆盖率
- 添加集成测试

## 🔧 开发环境设置

### 系统要求
- Ubuntu 20.04 LTS
- ROS2 Galactic
- GTSAM 4.1+
- Python 3.8+

### 安装步骤
```bash
# 1. 安装 ROS2 Galactic
sudo apt install ros-galactic-desktop

# 2. 安装 GTSAM
sudo apt install libgtsam-dev

# 3. 安装其他依赖
sudo apt install libeigen3-dev libboost-all-dev

# 4. 编译项目
cd underwater_nav_ws
colcon build --parallel-workers 1
```

## 📝 代码规范

### C++ 代码规范
- 使用 Google C++ Style Guide
- 函数和变量使用 snake_case
- 类名使用 PascalCase
- 添加适当的注释

### Python 代码规范
- 遵循 PEP 8 规范
- 使用有意义的变量名
- 添加文档字符串

### Git 提交规范
使用 Conventional Commits 格式：
- `feat:` 新功能
- `fix:` 修复 bug
- `docs:` 文档更新
- `style:` 代码格式调整
- `refactor:` 代码重构
- `test:` 测试相关
- `chore:` 构建过程或辅助工具的变动

## 🧪 测试

### 运行测试
```bash
# C++ 测试
cd underwater_nav_ws
colcon test

# Python 测试
cd lstm_dvl_training
python -m pytest tests/
```

## 📞 联系我们

- **邮箱**: 241314010016@hhu.edu.cn
- **项目地址**: https://github.com/dongchenhui/LSTM_TimeFGO_nav

## 📄 许可证

本项目采用 MIT 许可证。详见 [LICENSE](LICENSE) 文件。

---

感谢您的贡献！🎉 