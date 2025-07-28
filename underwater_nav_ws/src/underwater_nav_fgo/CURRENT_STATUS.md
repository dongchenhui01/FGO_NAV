# 当前系统状态和下一步操作

## ✅ 已完成的依赖安装

### 基础开发工具 (全部完成)
- ✅ GCC编译器: /usr/bin/gcc
- ✅ G++编译器: /usr/bin/g++
- ✅ CMake: /usr/bin/cmake (3.16.3)
- ✅ Make: /usr/bin/make
- ✅ Git: /usr/bin/git
- ✅ Python3: /usr/bin/python3
- ✅ Pip3: /usr/bin/pip3

### 系统包 (部分完成)
- ✅ 构建工具: build-essential 12.8ubuntu1.1
- ✅ Eigen3开发包: 3.3.7-2
- ✅ Boost开发包: 1.71.0.0ubuntu2
- ❌ YAML-CPP开发包: 未安装
- ❌ OpenCV开发包: 未安装

### Python模块 (全部完成)
- ✅ NumPy: 1.24.4
- ✅ SciPy: 1.10.1 (刚刚安装)
- ✅ Matplotlib: 3.7.5
- ✅ Pandas: 2.0.3
- ✅ PyYAML: 5.3.1

## ❌ 仍需安装的关键依赖

### 1. 系统包
```bash
sudo apt install -y libyaml-cpp-dev libopencv-dev libsuitesparse-dev
```

### 2. ROS2 Galactic
```bash
# 添加ROS2仓库
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装ROS2
sudo apt update
sudo apt install -y ros-galactic-desktop python3-colcon-common-extensions python3-rosdep
```

### 3. GTSAM库 (最复杂，需要编译)
```bash
# 下载和编译GTSAM
cd /tmp
git clone https://github.com/borglab/gtsam.git
cd gtsam && git checkout 4.1.1
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DGTSAM_BUILD_PYTHON=OFF -DGTSAM_BUILD_TESTS=OFF
make -j2 && sudo make install && sudo ldconfig
```

## 🚀 推荐的安装方式

### 方式1: 手动分步安装 (推荐)
按照上面的命令逐步执行，可以更好地控制进度和排查问题。

### 方式2: 使用交互式脚本
```bash
/home/dongchenhui/underwater_nav_fgo/scripts/install_step_by_step.sh
```

### 方式3: 一键安装 (如果网络稳定)
```bash
sudo /home/dongchenhui/underwater_nav_fgo/scripts/install_dependencies_ubuntu20.sh
```

## 📋 安装后的验证步骤

### 1. 检查依赖状态
```bash
/home/dongchenhui/underwater_nav_fgo/scripts/check_dependencies.sh
```

### 2. 设置环境变量
```bash
echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
mkdir -p ~/underwater_nav_ws/src
```

### 3. 复制项目到工作空间
```bash
cp -r /home/dongchenhui/underwater_nav_fgo ~/underwater_nav_ws/src/
```

### 4. 编译项目
```bash
cd ~/underwater_nav_ws
source /opt/ros/galactic/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 5. 测试运行
```bash
source install/setup.bash
python3 src/underwater_nav_fgo/scripts/generate_sample_data.py -o test.csv -d 30
ros2 launch underwater_nav_fgo navigation.launch.py data_file:=test.csv
```

## ⏱️ 预计安装时间

- 系统包: 2-3分钟
- ROS2: 10-15分钟  
- GTSAM: 15-25分钟
- **总计: 约30-45分钟**

## 💾 磁盘空间需求

- ROS2: ~2GB
- GTSAM编译: ~1GB (临时)
- 项目: ~500MB
- **总计: ~3-4GB**

## 🔧 故障排除

### 如果GTSAM编译失败
```bash
# 使用单线程编译
make -j1

# 或者安装更多依赖
sudo apt install -y libtbb-dev libmetis-dev
```

### 如果ROS2安装失败
```bash
# 清理apt缓存
sudo apt clean
sudo apt update

# 检查网络连接
ping packages.ros.org
```

### 如果权限问题
```bash
# 确保用户在sudo组
sudo usermod -aG sudo $USER
```

## 📞 获取帮助

1. **检查日志**: 安装过程中的错误信息
2. **运行检查脚本**: 确定具体缺少什么
3. **分步安装**: 避免一次性安装所有依赖
4. **查看详细文档**: `/home/dongchenhui/underwater_nav_fgo/INSTALL_GUIDE.md`

---

**当前进度**: 约60%完成 (基础工具和Python包已就绪)
**下一步**: 安装ROS2和GTSAM
**预计完成时间**: 30-45分钟
