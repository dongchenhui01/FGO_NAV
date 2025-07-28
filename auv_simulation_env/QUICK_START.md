# AUV仿真环境快速开始指南

## 🚀 快速启动

### 第1步：自动安装环境
```bash
cd ~/LSTM_TimeFGO_nav/auv_simulation_env
./setup_environment.sh
```

### 第2步：重新加载环境变量
```bash
source ~/.bashrc
# 或者重新打开终端
```

### 第3步：启动仿真环境
```bash
./launch_simulation.sh
```

## 📁 项目结构

```
auv_simulation_env/
├── README.md                    # 项目说明
├── INSTALL_GUIDE.md            # 详细安装指南
├── QUICK_START.md              # 本文件
├── setup_environment.sh        # 自动安装脚本
├── launch_simulation.sh        # 启动仿真脚本
├── ardupilot/                  # ArduPilot SITL源码
├── config/                     # 配置文件
├── data/                       # 数据文件夹
├── scripts/                    # 工具脚本
└── ros_packages/               # 自定义ROS包
```

## 🔧 使用您的真实数据

### 1. 准备数据
将您采集的CSV数据文件放入 `data/real_data/` 目录：
```bash
cp your_data.csv auv_simulation_env/data/real_data/
```

### 2. 启动数据回放
```bash
# 在新终端中运行
cd ~/LSTM_TimeFGO_nav/auv_simulation_env
python3 scripts/data_replay.py data/real_data/your_data.csv
```

### 3. 监控数据
```bash
# 查看发布的话题
ros2 topic list

# 监控IMU数据
ros2 topic echo /auv/imu/data

# 监控GPS数据
ros2 topic echo /auv/gps/fix

# 监控DVL数据
ros2 topic echo /auv/dvl/twist
```

## 🧪 测试您的算法

### 1. 修改算法测试脚本
编辑 `scripts/algorithm_test.py` 中的 `run_custom_algorithm()` 函数，替换为您的实际算法。

### 2. 运行算法测试
```bash
python3 scripts/algorithm_test.py
```

### 3. 查看结果
算法测试会自动：
- 订阅传感器数据
- 运行您的算法
- 与Ground Truth对比
- 计算误差统计
- 保存结果到JSON文件

## 📊 数据话题说明

### 输入数据话题
- `/auv/imu/data` - IMU数据 (sensor_msgs/Imu)
- `/auv/gps/fix` - GPS数据 (sensor_msgs/NavSatFix)
- `/auv/dvl/twist` - DVL速度数据 (geometry_msgs/TwistStamped)

### Ground Truth话题
- `/gazebo/model_states` - Gazebo仿真的真实状态
- `/mavros/local_position/pose` - MAVROS本地位置

### 算法输出话题
- `/auv/algorithm/pose` - 算法估计的位姿
- `/auv/algorithm/odometry` - 算法估计的里程计

## 🎮 控制AUV

### 基本控制命令
```bash
# 解锁AUV
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

# 设置模式为GUIDED
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"

# 设置目标位置
ros2 topic pub /mavros/setpoint_position/local geometry_msgs/PoseStamped "
header:
  frame_id: 'map'
pose:
  position:
    x: 10.0
    y: 5.0
    z: -2.0
  orientation:
    w: 1.0"
```

## 🔍 故障排除

### 常见问题

1. **SITL启动失败**
   ```bash
   # 检查ArduPilot路径
   echo $PATH | grep ardupilot
   
   # 重新安装依赖
   cd ardupilot
   Tools/environment_install/install-prereqs-ubuntu.sh -y
   ```

2. **Gazebo无法启动**
   ```bash
   # 检查显卡驱动
   nvidia-smi
   
   # 重新安装Gazebo
   sudo apt install --reinstall gazebo11
   ```

3. **MAVROS连接失败**
   ```bash
   # 检查端口占用
   netstat -tulpn | grep 14550
   
   # 重启MAVROS
   ros2 launch mavros apm.launch fcu_url:=udp://:14550@127.0.0.1:14551
   ```

4. **ROS话题无数据**
   ```bash
   # 检查节点状态
   ros2 node list
   
   # 检查话题连接
   ros2 topic info /auv/imu/data
   ```

### 性能优化

1. **减少Gazebo渲染负载**
   ```bash
   # 启动无GUI的Gazebo
   gazebo --verbose -s libgazebo_ros_factory.so world.world
   ```

2. **调整数据回放频率**
   ```bash
   # 降低回放速率
   python3 scripts/data_replay.py data.csv --rate 0.5
   ```

## 📈 评估指标

算法测试会自动计算以下指标：

### 精度指标
- **位置误差**: RMSE, MAE, 最大误差
- **速度误差**: RMSE, MAE, 最大误差
- **轨迹误差**: 累积误差, 相对误差

### 性能指标
- **计算时间**: 平均, 最大, 最小
- **实时性**: 处理频率, 延迟
- **资源占用**: CPU, 内存使用率

## 🎯 下一步

1. **集成LSTM模型**: 将您训练好的LSTM模型集成到算法测试脚本中
2. **参数调优**: 根据测试结果调整算法参数
3. **闭环测试**: 将算法输出反馈给控制器进行闭环测试
4. **性能优化**: 优化算法以满足实时性要求
5. **鲁棒性测试**: 在不同环境条件下测试算法

## 📞 获取帮助

如果遇到问题：
1. 查看详细的安装指南 `INSTALL_GUIDE.md`
2. 检查日志输出中的错误信息
3. 确认所有依赖都已正确安装
4. 验证ROS环境变量设置正确
