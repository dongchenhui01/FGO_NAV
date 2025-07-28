# DVL数据消息定义
# 时间戳
builtin_interfaces/Time stamp

# DVL测量的三轴速度 (m/s)
geometry_msgs/Vector3 velocity

# 速度协方差矩阵 (3x3，行优先存储)
float64[9] velocity_covariance

# DVL工作状态
bool is_valid
bool bottom_track_valid
bool water_track_valid

# 测量质量指标
float32 altitude          # 距离海底高度 (m)
float32[4] beam_quality   # 四个波束的信号质量
int32 num_good_beams      # 有效波束数量

# DVL配置信息
float32 frequency         # 工作频率 (Hz)
float32 beam_angle        # 波束角度 (度)
