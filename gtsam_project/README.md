# AUV因子图组合导航算法

基于GTSAM的AUV组合导航算法，使用ISAM2作为后端优化器，融合IMU、DVL、GPS和磁力计传感器数据。

## 特性

- **多传感器融合**: IMU + DVL + GPS + 磁力计
- **实时优化**: 使用ISAM2增量式平滑和建图
- **GPS初始化**: 使用第一个有效GPS位置作为先验
- **鲁棒性**: 处理传感器数据缺失和异常
- **可视化**: 完整的轨迹可视化和误差分析

## 系统要求

- Ubuntu 18.04/20.04/22.04
- GTSAM 4.0+
- CMake 3.10+
- C++14
- Python 3.6+ (用于可视化)

## 依赖安装

### GTSAM库
```bash
# 如果未安装GTSAM
sudo apt update
sudo apt install libgtsam-dev

# 或从源码编译
git clone https://github.com/borglab/gtsam.git
cd gtsam
mkdir build && cd build
cmake .. -DGTSAM_BUILD_PYTHON=ON
make -j$(nproc)
sudo make install
```

### Python依赖
```bash
pip3 install matplotlib pandas numpy
```

## 编译和运行

### 快速开始
```bash
# 给脚本执行权限
chmod +x build_and_run.sh

# 编译并运行（使用您的数据文件）
./build_and_run.sh ../underwater_nav_fgo/data/sensor_fusion_log_20250712_102606.csv
```

### 分步执行
```bash
# 仅编译
./build_and_run.sh data.csv --build-only

# 仅运行
./build_and_run.sh data.csv --run-only

# 运行但不生成可视化
./build_and_run.sh data.csv --no-viz
```

### 手动编译
```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### 手动运行
```bash
# 运行算法
./build/bin/auv_navigation path/to/your/data.csv

# 可视化结果
python3 src/visualize_trajectory.py \
    --trajectory results/trajectory.csv \
    --gps_ref path/to/your/data.csv \
    --output_dir results
```

## 数据格式

输入CSV文件应包含以下列（按顺序）：
```
utc_timestamp,acc_x,acc_y,acc_z,gyr_x,gyr_y,gyr_z,mag_x,mag_y,mag_z,dvl_vx,dvl_vy,dvl_vz,gps_lon,gps_lat,gps_alt,...,gps_east,gps_north,gps_up
```

### 重要说明
- **GPS初始化**: 算法会自动找到第一个有效GPS数据点作为初始位置
- **坐标系**: 使用ENU（东-北-上）坐标系
- **单位**: 位置(m), 速度(m/s), 加速度(m/s²), 角速度(rad/s)

## 算法原理

### 因子图结构
```
X(t) ---- IMU ---- X(t+1) ---- IMU ---- X(t+2)
 |                   |                    |
 V(t)               V(t+1)              V(t+2)
 |                   |                    |
 B(t) ---- Bias ---- B(t+1) ---- Bias ---- B(t+2)
 |                   |                    |
GPS                 DVL                  GPS
```

### 状态变量
- **X(t)**: 6DOF位姿 (位置 + 姿态)
- **V(t)**: 3D速度
- **B(t)**: IMU偏置 (加速度计 + 陀螺仪)

### 因子类型
1. **IMU因子**: 预积分IMU测量
2. **GPS因子**: 位置约束
3. **DVL因子**: 速度约束
4. **偏置因子**: 偏置随机游走
5. **先验因子**: 初始状态约束

## 输出结果

### 轨迹文件
- `results/trajectory.csv`: 估计的3D轨迹点

### 可视化图像
- `results/trajectory_2d.png`: 2D轨迹对比图
- `results/trajectory_3d.png`: 3D轨迹可视化
- `results/position_error.png`: 位置误差分析

## 参数调优

在`src/auv_navigation.cpp`中可以调整以下参数：

### 噪声模型
```cpp
// IMU噪声
imu_params->accelerometerCovariance = Matrix33::Identity() * 0.1;
imu_params->gyroscopeCovariance = Matrix33::Identity() * 0.01;

// GPS噪声
Vector(3) << 2.0, 2.0, 5.0  // East, North, Up

// DVL噪声
Vector(3) << 0.1, 0.1, 0.1  // Vx, Vy, Vz
```

### ISAM2参数
```cpp
isam_params.relinearizeThreshold = 0.1;
isam_params.relinearizeSkip = 1;
```

## 性能分析

### 计算复杂度
- **时间复杂度**: O(n) 增量式优化
- **空间复杂度**: O(n) 滑动窗口

### 精度评估
算法会自动计算与GPS参考的位置误差：
- 平均误差 (Mean Error)
- 标准差 (Std Error)
- 最大误差 (Max Error)
- RMS误差 (RMS Error)

## 故障排除

### 编译错误
```bash
# 检查GTSAM安装
pkg-config --modversion gtsam

# 检查依赖
ldd build/bin/auv_navigation
```

### 运行时错误
```bash
# 检查数据格式
head -5 your_data.csv

# 检查GPS数据
grep -v "0,0,0" your_data.csv | head -5
```

### 可视化问题
```bash
# 安装matplotlib后端
sudo apt install python3-tk

# 检查Python依赖
python3 -c "import matplotlib; print(matplotlib.__version__)"
```

## 扩展功能

### 添加新传感器
1. 在`SensorData`结构中添加新字段
2. 在`readCSV`函数中读取数据
3. 创建对应的因子类型
4. 在主循环中添加因子

### 自定义因子
```cpp
// 示例：磁力计航向因子
class MagnetometerFactor : public NoiseModelFactor1<Pose3> {
    // 实现evaluate函数
};
```

## 贡献

欢迎提交Issue和Pull Request来改进算法！

## 许可证

MIT License

## 作者

dongchenhui - AUV导航算法开发
