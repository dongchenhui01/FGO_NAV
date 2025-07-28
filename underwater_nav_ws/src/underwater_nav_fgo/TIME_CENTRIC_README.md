# 时间中心因子图优化 (Time-Centric Factor Graph Optimization)

## 概述

本项目基于IEEE论文 **"gnssFGO: an online and time-centric factor graph optimization for GNSS/Multi-sensor vehicle localization"** 的方法，实现了时间中心的水下导航系统。与传统的基于关键帧的方法不同，时间中心方法使用连续时间轨迹表示和高斯过程回归，能够更好地处理多传感器数据的时间同步和插值问题。

## 核心特点

### 🕒 时间中心设计
- **连续时间轨迹**: 使用高斯过程回归表示连续时间轨迹
- **时间戳驱动**: 所有传感器数据按时间戳排序和处理
- **任意时间查询**: 支持查询任意时间点的导航状态
- **时间窗口管理**: 自动管理时间窗口，保持计算效率

### 🔄 多传感器融合
- **IMU预积分**: 连续时间的IMU预积分处理
- **DVL速度约束**: 时间插值的DVL速度因子
- **磁力计航向**: 时间同步的磁力计约束
- **自适应噪声**: 基于时间一致性的噪声模型

### 📊 高级插值方法
- **高斯过程回归**: 基于核函数的非参数插值
- **线性插值**: 快速的分段线性插值
- **样条插值**: 平滑的样条曲线插值
- **置信度评估**: 插值结果的不确定性量化

## 与传统方法的对比

| 特性 | 传统关键帧方法 | 时间中心方法 |
|------|----------------|--------------|
| 轨迹表示 | 离散关键帧 | 连续时间轨迹 |
| 时间处理 | 固定时间间隔 | 任意时间查询 |
| 传感器同步 | 最近邻匹配 | 精确时间插值 |
| 内存使用 | 固定窗口大小 | 自适应时间窗口 |
| 查询延迟 | 需要优化后 | 实时插值查询 |
| 精度 | 关键帧间线性 | 高斯过程平滑 |

## 使用方法

### 1. 基本使用

```bash
# 启动时间中心导航系统
ros2 launch underwater_nav_fgo time_centric_navigation.launch.py \
    data_file:=your_data.csv \
    time_window_size:=2.0 \
    interpolation_method:=gp
```

### 2. 参数配置

主要参数在 `config/time_centric_params.yaml` 中配置：

```yaml
time_centric:
  time_window_size: 2.0          # 时间窗口大小
  interpolation:
    method: "gp"                 # 插值方法: linear, gp, spline
    gaussian_process:
      length_scale: 0.1          # GP长度尺度
      signal_variance: 1.0       # GP信号方差
      noise_variance: 0.01       # GP噪声方差
```

### 3. 编程接口

```cpp
// 启用时间中心模式
fgo->enableTimeCentricMode(true, 2.0, "gp");

// 查询任意时间的轨迹状态
double query_time = current_time - 0.5;  // 查询0.5秒前的状态
auto trajectory_point = fgo->queryTrajectoryAtTime(query_time);

// 获取插值置信度
double confidence = trajectory_point.confidence;
```

## 插值方法详解

### 1. 高斯过程回归 (GP)
- **优点**: 提供不确定性量化，平滑插值
- **缺点**: 计算复杂度较高
- **适用**: 高精度要求，有足够计算资源

```yaml
interpolation:
  method: "gp"
  gaussian_process:
    length_scale: 0.1      # 控制平滑程度
    signal_variance: 1.0   # 信号强度
    noise_variance: 0.01   # 噪声水平
```

### 2. 线性插值 (Linear)
- **优点**: 计算快速，实现简单
- **缺点**: 不够平滑，无不确定性信息
- **适用**: 实时性要求高，计算资源有限

```yaml
interpolation:
  method: "linear"
  linear:
    extrapolation_limit: 0.1  # 外推限制
```

### 3. 样条插值 (Spline)
- **优点**: 平滑连续，计算适中
- **缺点**: 需要足够的控制点
- **适用**: 平衡精度和效率

```yaml
interpolation:
  method: "spline"
  spline:
    degree: 3              # 样条次数
    smoothing_factor: 0.1  # 平滑因子
```

## 时间窗口管理

时间中心方法使用滑动时间窗口来管理数据：

```yaml
time_centric:
  time_window_size: 2.0        # 窗口大小 (秒)
  sliding_window_step: 0.1     # 滑动步长 (秒)
  max_time_gap: 0.5            # 最大时间间隔 (秒)
```

### 窗口管理策略
1. **固定窗口**: 保持固定的时间窗口大小
2. **自适应窗口**: 根据数据密度调整窗口大小
3. **重叠窗口**: 使用重叠窗口保证连续性

## 性能优化

### 1. 计算优化
```yaml
performance:
  max_cpu_usage: 80.0          # CPU使用限制
  memory_limit: 2048           # 内存限制 (MB)
  max_processing_delay: 0.05   # 最大延迟 (秒)
```

### 2. 缓存策略
```yaml
performance:
  trajectory_cache_size: 1000    # 轨迹缓存大小
  measurement_buffer_size: 10000 # 测量缓冲区大小
```

### 3. 实时性保证
- **优先级调度**: 关键计算优先处理
- **异步处理**: 插值查询与优化分离
- **预计算**: 提前计算常用时间点

## 应用场景

### 1. 水下机器人导航
- **AUV路径跟踪**: 精确的轨迹跟踪和控制
- **ROV操作**: 实时位置反馈和稳定控制
- **水下测量**: 高精度的位置标定

### 2. 数据后处理
- **轨迹重建**: 从稀疏数据重建完整轨迹
- **数据同步**: 多传感器数据的时间对齐
- **质量评估**: 导航精度的定量分析

### 3. 仿真和测试
- **算法验证**: 不同插值方法的性能对比
- **参数调优**: 时间窗口和插值参数优化
- **基准测试**: 与传统方法的性能对比

## 调试和监控

### 1. 可视化工具
```bash
# 启动带可视化的系统
ros2 launch underwater_nav_fgo time_centric_navigation.launch.py \
    use_rviz:=true \
    enable_continuous_query:=true
```

### 2. 性能监控
```bash
# 查看优化统计
ros2 service call /time_centric_navigation/get_stats std_srvs/srv/Empty

# 查询特定时间的轨迹
ros2 service call /time_centric_navigation/query_trajectory std_srvs/srv/Empty
```

### 3. 调试输出
```yaml
debug:
  time_centric_debug:
    log_interpolation: true      # 记录插值过程
    log_time_sync: true          # 记录时间同步
    save_trajectory_cache: true  # 保存轨迹缓存
```

## 常见问题

### Q1: 如何选择合适的时间窗口大小？
**A**: 时间窗口大小需要平衡精度和效率：
- 太小：插值精度不足，缺乏足够的控制点
- 太大：计算负担重，内存占用高
- 建议：根据传感器频率设置为2-5秒

### Q2: 高斯过程插值计算太慢怎么办？
**A**: 可以采用以下优化策略：
- 减少控制点数量 (`max_control_points`)
- 使用线性插值作为备选
- 启用缓存机制
- 调整长度尺度参数

### Q3: 如何处理传感器时间同步问题？
**A**: 系统提供多种时间同步方法：
- 设置传感器时间偏移 (`time_offset`)
- 启用自动时间同步 (`time_synchronization.enable`)
- 使用时间一致性检测

## 参考文献

1. **gnssFGO论文**: "gnssFGO: an online and time-centric factor graph optimization for GNSS/Multi-sensor vehicle localization"
2. **GTSAM库**: Georgia Tech Smoothing and Mapping library
3. **高斯过程**: Gaussian Processes for Machine Learning (Rasmussen & Williams)

---

**注意**: 时间中心方法相比传统方法有更高的计算复杂度，建议在有足够计算资源的系统上使用。对于资源受限的应用，可以选择线性插值或调整参数以平衡性能和精度。
