# IMU数据消息定义
# 时间戳
builtin_interfaces/Time stamp

# 线性加速度 (m/s²)
geometry_msgs/Vector3 linear_acceleration

# 角速度 (rad/s)  
geometry_msgs/Vector3 angular_velocity

# 磁场强度 (任意单位)
geometry_msgs/Vector3 magnetic_field

# 协方差矩阵 (可选)
float64[9] linear_acceleration_covariance  # 3x3矩阵，行优先存储
float64[9] angular_velocity_covariance     # 3x3矩阵，行优先存储
float64[9] magnetic_field_covariance       # 3x3矩阵，行优先存储

# 数据质量标志
bool is_valid
