"""
三维导航系统仿真：因子图优化(iSAM2)与EKF比较

本程序实现了使用里程计和位置观测的三维导航系统仿真，并对比了两种后端优化方法：
1. 扩展卡尔曼滤波(EKF)：经典的递归滤波方法
2. 因子图优化(iSAM2)：现代化的增量式图优化方法

仿真时间为3000秒，系统生成了复杂的三维轨迹并添加噪声和外点干扰。
"""
import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # 使用适合您系统的后端
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation
import gtsam
from gtsam.symbol_shorthand import X, V

# -----------------------------------------------------------------------------
# 1. 工具函数
# -----------------------------------------------------------------------------

def normalize_euler_angles(angles):
    """归一化欧拉角，确保它们在合理范围内"""
    return Rotation.from_euler('xyz', angles).as_euler('xyz')

def pose_to_vector(position, rotation):
    """将位置和旋转转换为9维状态向量: [x,y,z,rx,ry,rz,vx,vy,vz]"""
    return np.concatenate([position, rotation])

def pose_distance(pose1, pose2):
    """计算两个位姿之间的距离"""
    pos_dist = np.linalg.norm(pose1[:3] - pose2[:3])
    # 旋转距离使用四元数计算
    r1 = Rotation.from_euler('xyz', pose1[3:6])
    r2 = Rotation.from_euler('xyz', pose2[3:6])
    rot_dist = np.arccos(np.clip(np.abs(np.sum(r1.as_quat() * r2.as_quat())), -1.0, 1.0)) * 2.0
    return pos_dist, rot_dist

# -----------------------------------------------------------------------------
# 2. 扩展卡尔曼滤波器(EKF)实现
# -----------------------------------------------------------------------------

class EKF:
    def __init__(self, initial_state, initial_covariance):
        """
        初始化EKF
        
        参数:
        - initial_state: 初始状态向量 [x,y,z,rx,ry,rz]
        - initial_covariance: 初始协方差矩阵
        """
        self.state = initial_state
        self.covariance = initial_covariance
        
    def predict(self, odom, odom_cov, dt):
        """
        预测步骤
        
        参数:
        - odom: 里程计测量 [dx,dy,dz,drx,dry,drz]
        - odom_cov: 里程计测量噪声协方差
        - dt: 时间步长
        """
        # 状态转移函数
        def f(x, u):
            # 从当前状态和里程计读数计算新状态
            R = Rotation.from_euler('xyz', x[3:6])
            new_pos = x[:3] + R.apply(u[:3])
            new_rot = normalize_euler_angles(x[3:6] + u[3:6])
            return np.concatenate([new_pos, new_rot])
        
        # 计算雅可比矩阵（数值近似）
        def compute_jacobian(func, x, u, eps=1e-6):
            n = len(x)
            m = len(u)
            F = np.zeros((n, n))
            G = np.zeros((n, m))
            
            # 计算状态转移对状态的雅可比
            for i in range(n):
                x_plus = x.copy()
                x_plus[i] += eps
                F[:, i] = (func(x_plus, u) - func(x, u)) / eps
                
            # 计算状态转移对输入的雅可比
            for i in range(m):
                u_plus = u.copy()
                u_plus[i] += eps
                G[:, i] = (func(x, u_plus) - func(x, u)) / eps
                
            return F, G
        
        # 计算雅可比矩阵
        F, G = compute_jacobian(f, self.state, odom)
        
        # 预测新状态
        self.state = f(self.state, odom)
        
        # 预测新协方差
        self.covariance = F @ self.covariance @ F.T + G @ odom_cov @ G.T
        
    def update(self, measurement, meas_cov, measurement_function, measurement_jacobian=None):
        """
        更新步骤
        
        参数:
        - measurement: 测量值
        - meas_cov: 测量噪声协方差
        - measurement_function: 测量函数，从状态到测量
        - measurement_jacobian: 测量雅可比，可选
        """
        # 计算测量雅可比（如果未提供）
        if measurement_jacobian is None:
            def compute_measurement_jacobian(func, x, eps=1e-6):
                n = len(x)
                m = len(func(x))
                H = np.zeros((m, n))
                
                for i in range(n):
                    x_plus = x.copy()
                    x_plus[i] += eps
                    H[:, i] = (func(x_plus) - func(x)) / eps
                    
                return H
            
            H = compute_measurement_jacobian(measurement_function, self.state)
        else:
            H = measurement_jacobian(self.state)
        
        # 计算预测测量
        predicted_measurement = measurement_function(self.state)
        
        # 计算测量残差
        residual = measurement - predicted_measurement
        
        # 特殊处理：如果包含旋转，需要正确处理角度差异
        if len(residual) >= 6:  # 假设位姿测量包含位置和姿态
            euler_diff = (Rotation.from_euler('xyz', measurement[3:6]) * 
                         Rotation.from_euler('xyz', predicted_measurement[3:6]).inv()).as_euler('xyz')
            residual[3:6] = euler_diff
        
        # 计算卡尔曼增益
        S = H @ self.covariance @ H.T + meas_cov
        K = self.covariance @ H.T @ np.linalg.inv(S)
        
        # 更新状态
        self.state = self.state + K @ residual
        
        # 确保欧拉角在正确范围内
        self.state[3:6] = normalize_euler_angles(self.state[3:6])
        
        # 更新协方差
        I = np.eye(len(self.state))
        self.covariance = (I - K @ H) @ self.covariance
        
# -----------------------------------------------------------------------------
# 3. 主程序：生成数据、优化和比较
# -----------------------------------------------------------------------------

def main():
    """主函数：运行仿真、优化和比较"""
    print("开始三维导航系统仿真...")
    
    # 仿真参数
    duration = 3000.0  # 仿真时间（秒）
    dt = 1.0         # 时间步长（秒）
    num_steps = int(duration / dt)
    
    # 噪声参数
    odom_pos_noise = 0.05  # 里程计位置噪声标准差（米）
    odom_rot_noise = np.deg2rad(2.0)  # 里程计旋转噪声标准差（弧度）
    gps_pos_noise = 0.5   # GPS位置噪声标准差（米）
    gps_rot_noise = np.deg2rad(5.0)  # GPS旋转噪声标准差（弧度）
    
    # 外点参数
    outlier_prob = 0.02  # 外点概率
    gps_frequency = 10   # GPS测量频率（每10步一次）
    
    # 1. 生成真实轨迹
    print("生成真实轨迹...")
    
    # 初始位姿
    true_poses = np.zeros((num_steps + 1, 6))  # [x,y,z,rx,ry,rz]
    true_velocities = np.zeros((num_steps, 3))  # [vx,vy,vz]
    
    # 生成有趣的三维轨迹
    for i in range(num_steps):
        t = i * dt
        
        # 当前位姿
        current_pose = true_poses[i]
        current_rotation = Rotation.from_euler('xyz', current_pose[3:6])
        
        # 计算速度（随时间变化的复杂模式）
        body_vx = 1.0 + 0.5 * np.sin(0.01 * t)
        body_vy = 0.1 * np.sin(0.05 * t)
        body_vz = 0.2 * np.cos(0.02 * t)
        body_velocity = np.array([body_vx, body_vy, body_vz])
        
        # 计算角速度
        roll_rate = 0.02 * np.sin(0.01 * t)
        pitch_rate = 0.01 * np.cos(0.02 * t)
        yaw_rate = 0.1 * (0.5 + 0.5 * np.sin(0.005 * t))
        angular_velocity = np.array([roll_rate, pitch_rate, yaw_rate])
        
        # 转换到世界坐标系
        world_velocity = current_rotation.apply(body_velocity)
        true_velocities[i] = world_velocity
        
        # 更新位置
        new_position = current_pose[:3] + world_velocity * dt
        
        # 更新旋转
        new_rotation = normalize_euler_angles(current_pose[3:6] + angular_velocity * dt)
        
        # 保存新位姿
        true_poses[i+1] = np.concatenate([new_position, new_rotation])
    
    # 2. 生成带噪声的测量
    print("生成带噪声的测量...")
    
    # 里程计噪声协方差
    odom_cov = np.diag([odom_pos_noise**2] * 3 + [odom_rot_noise**2] * 3)
    
    # GPS噪声协方差
    gps_cov = np.diag([gps_pos_noise**2] * 3 + [gps_rot_noise**2] * 3)
    
    # 生成里程计测量
    odom_measurements = []
    for i in range(num_steps):
        # 计算真实里程计
        p1, p2 = true_poses[i], true_poses[i+1]
        r1 = Rotation.from_euler('xyz', p1[3:6])
        r2 = Rotation.from_euler('xyz', p2[3:6])
        
        # 从p1到p2的相对运动，在p1坐标系下
        rel_pos = r1.inv().apply(p2[:3] - p1[:3])
        rel_rot = (r1.inv() * r2).as_euler('xyz')
        true_odom = np.concatenate([rel_pos, rel_rot])
        
        # 添加噪声
        if np.random.rand() < outlier_prob:
            # 外点 - 添加更大的噪声
            print(f"里程计外点 @ 步骤 {i}")
            odom_noise = np.random.multivariate_normal(
                np.zeros(6), 
                np.diag([1.0**2] * 3 + [np.deg2rad(20.0)**2] * 3)
            )
        else:
            # 正常噪声
            odom_noise = np.random.multivariate_normal(np.zeros(6), odom_cov)
        
        odom_measurements.append(true_odom + odom_noise)
    
    # 生成GPS测量（稀疏）
    gps_measurements = {}
    for i in range(0, num_steps + 1, gps_frequency):
        if np.random.rand() < outlier_prob:
            # 外点 - 添加更大的噪声
            print(f"GPS外点 @ 步骤 {i}")
            gps_noise = np.random.multivariate_normal(
                np.zeros(6), 
                np.diag([10.0**2] * 3 + [np.deg2rad(45.0)**2] * 3)
            )
        else:
            # 正常噪声
            gps_noise = np.random.multivariate_normal(np.zeros(6), gps_cov)
        
        gps_measurements[i] = true_poses[i] + gps_noise
    
    # 3. 运行EKF
    print("\n运行扩展卡尔曼滤波(EKF)...")
    
    # 初始化EKF
    initial_cov = np.diag([0.1**2] * 3 + [np.deg2rad(5.0)**2] * 3)
    ekf = EKF(true_poses[0], initial_cov)
    
    # 存储EKF轨迹
    ekf_poses = np.zeros_like(true_poses)
    ekf_poses[0] = true_poses[0]
    
    # 运行EKF
    for i in range(num_steps):
        # 预测步骤 - 使用里程计
        ekf.predict(odom_measurements[i], odom_cov, dt)
        
        # 更新步骤 - 如果有GPS测量
        if i+1 in gps_measurements:
            # GPS更新
            ekf.update(
                gps_measurements[i+1],
                gps_cov,
                lambda x: x,  # 测量函数 - 位姿是直接测量的
                lambda x: np.eye(6)  # 雅可比矩阵 - 恒等映射
            )
        
        # 保存当前估计
        ekf_poses[i+1] = ekf.state
        
        # 每100步打印一次状态
        if (i+1) % 100 == 0:
            print(f"EKF步骤 {i+1}/{num_steps} | 位置: [{ekf.state[0]:.2f}, {ekf.state[1]:.2f}, {ekf.state[2]:.2f}]")
    
    # 4. 运行iSAM2（因子图优化）
    print("\n运行iSAM2（因子图优化）...")
    
    # 创建iSAM2优化器
    isam_params = gtsam.ISAM2Params()
    isam_params.setRelinearizeThreshold(0.1)
    isam_params.relinearizeSkip = 10
    isam = gtsam.ISAM2(isam_params)
    
    # 噪声模型
    isam_odom_noise = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([odom_rot_noise, odom_rot_noise, odom_rot_noise, 
                 odom_pos_noise, odom_pos_noise, odom_pos_noise])
    )
    isam_gps_noise = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([gps_rot_noise, gps_rot_noise, gps_rot_noise,
                 gps_pos_noise, gps_pos_noise, gps_pos_noise])
    )
    
    # 存储因子图和当前估计
    graph = gtsam.NonlinearFactorGraph()
    initial_values = gtsam.Values()
    
    # 添加先验因子
    prior_noise = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01])
    )
    initial_pose = gtsam.Pose3(
        gtsam.Rot3.RzRyRx(true_poses[0, 3], true_poses[0, 4], true_poses[0, 5]),
        true_poses[0, :3]
    )
    graph.add(gtsam.PriorFactorPose3(X(0), initial_pose, prior_noise))
    initial_values.insert(X(0), initial_pose)
    
    # 存储iSAM2轨迹
    isam_poses = np.zeros_like(true_poses)
    isam_poses[0] = true_poses[0]
    
    # 首次更新
    isam.update(graph, initial_values)
    current_estimate = isam.calculateEstimate()
    
    # 清空图和初始值，为增量更新做准备
    graph = gtsam.NonlinearFactorGraph()
    initial_values = gtsam.Values()
    
    # 运行iSAM2
    for i in range(num_steps):
        # 从里程计创建相对位姿测量
        odom = odom_measurements[i]
        rel_rot = gtsam.Rot3.RzRyRx(odom[3], odom[4], odom[5])
        rel_pos = odom[:3]
        between_pose = gtsam.Pose3(rel_rot, rel_pos)
        
        # 添加里程计因子
        graph.add(gtsam.BetweenFactorPose3(X(i), X(i+1), between_pose, isam_odom_noise))
        
        # 获取上一个位姿估计作为初始值
        try:
            prev_pose_estimate = current_estimate.atPose3(X(i))
            # 计算新位姿的初始估计
            new_pose_estimate = prev_pose_estimate.compose(between_pose)
            initial_values.insert(X(i+1), new_pose_estimate)
        except RuntimeError:
            # 如果没有之前的估计，使用真实位姿作为初始值
            print(f"警告: 未找到X({i})的估计，使用先验初始值")
            initial_pose = gtsam.Pose3(
                gtsam.Rot3.RzRyRx(true_poses[i+1, 3], true_poses[i+1, 4], true_poses[i+1, 5]),
                true_poses[i+1, :3]
            )
            initial_values.insert(X(i+1), initial_pose)
        
        # 添加GPS因子（如果有）
        if i+1 in gps_measurements:
            gps = gps_measurements[i+1]
            gps_pose = gtsam.Pose3(
                gtsam.Rot3.RzRyRx(gps[3], gps[4], gps[5]),
                gps[:3]
            )
            graph.add(gtsam.PriorFactorPose3(X(i+1), gps_pose, isam_gps_noise))
        
        # 执行iSAM2增量更新
        isam.update(graph, initial_values)
        current_estimate = isam.calculateEstimate()
        
        # 提取当前位姿估计
        current_pose = current_estimate.atPose3(X(i+1))
        rot = current_pose.rotation().rpy()  # 获取roll-pitch-yaw
        pos = current_pose.translation()
        isam_poses[i+1] = np.array([pos[0], pos[1], pos[2], rot[0], rot[1], rot[2]])
        
        # 清空图和初始值，为下一次迭代做准备
        graph = gtsam.NonlinearFactorGraph()
        initial_values = gtsam.Values()
        
        # 每100步打印一次状态
        if (i+1) % 100 == 0:
            print(f"iSAM2步骤 {i+1}/{num_steps} | 位置: [{isam_poses[i+1,0]:.2f}, {isam_poses[i+1,1]:.2f}, {isam_poses[i+1,2]:.2f}]")
    
    # 5. 结果分析与可视化
    print("\n结果分析与可视化...")
    
    # 计算均方根误差
    def calculate_rmse(est_traj, true_traj):
        """计算位置的RMSE"""
        return np.sqrt(np.mean(np.sum((est_traj[:, :3] - true_traj[:, :3])**2, axis=1)))
    
    ekf_rmse = calculate_rmse(ekf_poses, true_poses)
    isam_rmse = calculate_rmse(isam_poses, true_poses)
    
    print(f"\n位置RMSE (米):")
    print(f"  EKF:   {ekf_rmse:.4f}")
    print(f"  iSAM2: {isam_rmse:.4f}")
    
    # 绘制3D轨迹
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # 真实轨迹
    ax.plot(true_poses[:, 0], true_poses[:, 1], true_poses[:, 2], 
            'g-', linewidth=2, label='真实轨迹')
    
    # EKF轨迹
    ax.plot(ekf_poses[:, 0], ekf_poses[:, 1], ekf_poses[:, 2], 
            'r--', linewidth=1.5, label='EKF估计')
    
    # iSAM2轨迹
    ax.plot(isam_poses[:, 0], isam_poses[:, 1], isam_poses[:, 2], 
            'b-', linewidth=1.5, label='iSAM2估计')
    
    # GPS测量
    gps_points = np.array([gps_measurements[i] for i in gps_measurements.keys()])
    ax.scatter(gps_points[:, 0], gps_points[:, 1], gps_points[:, 2], 
               c='m', marker='^', s=30, label='GPS测量')
    
    # 设置图形属性
    ax.set_xlabel('X (米)')
    ax.set_ylabel('Y (米)')
    ax.set_zlabel('Z (米)')
    ax.set_title('三维导航轨迹比较：EKF vs iSAM2')
    ax.legend()
    
    # 保存图形
    plt.savefig('3d_trajectory_comparison.png', dpi=300, bbox_inches='tight')
    
    # 位置误差随时间变化
    fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
    t = np.arange(0, num_steps + 1) * dt
    
    for i, axis_name in enumerate(['X', 'Y', 'Z']):
        # 计算误差
        ekf_error = ekf_poses[:, i] - true_poses[:, i]
        isam_error = isam_poses[:, i] - true_poses[:, i]
        
        # 绘制误差
        axs[i].plot(t, ekf_error, 'r-', alpha=0.7, label='EKF误差')
        axs[i].plot(t, isam_error, 'b-', alpha=0.7, label='iSAM2误差')
        
        # 绘制GPS测量误差
        gps_times = np.array(list(gps_measurements.keys())) * dt
        gps_errors = np.array([gps_measurements[j][i] - true_poses[j, i] for j in gps_measurements.keys()])
        axs[i].scatter(gps_times, gps_errors, c='m', marker='x', s=20, label='GPS误差')
        
        # 设置坐标轴属性
        axs[i].set_ylabel(f'{axis_name}轴误差 (米)')
        axs[i].grid(True)
        axs[i].legend()
    
    axs[2].set_xlabel('时间 (秒)')
    fig.suptitle('位置估计误差随时间变化', fontsize=16)
    
    # 保存图形
    plt.savefig('position_error_vs_time.png', dpi=300, bbox_inches='tight')
    
    # 尝试显示图形（如果支持的话）
    try:
        plt.show()
    except Exception as e:
        print(f"无法显示图形：{e}")
        print("图形已保存到文件：3d_trajectory_comparison.png 和 position_error_vs_time.png")

if __name__ == "__main__":
    main() 