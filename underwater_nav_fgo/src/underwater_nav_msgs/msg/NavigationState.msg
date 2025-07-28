# 导航状态消息定义
# 时间戳
builtin_interfaces/Time stamp

# 位置 (ENU坐标系，米)
geometry_msgs/Point position

# 速度 (ENU坐标系，m/s)
geometry_msgs/Vector3 velocity

# 姿态 (四元数表示)
geometry_msgs/Quaternion orientation

# 角速度 (载体坐标系，rad/s)
geometry_msgs/Vector3 angular_velocity

# 协方差矩阵 (15x15状态协方差，行优先存储)
# 状态顺序: [px, py, pz, vx, vy, vz, qw, qx, qy, qz, bax, bay, baz, bgx, bgy, bgz]
float64[225] covariance

# 估计质量指标
float64 position_accuracy    # 位置精度 (m)
float64 velocity_accuracy    # 速度精度 (m/s)
float64 heading_accuracy     # 航向精度 (度)

# 系统状态
bool is_initialized
bool is_converged
int32 optimization_iterations
