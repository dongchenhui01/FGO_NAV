"""
Comparison of Full Batch Factor Graph Optimization (FGO) and Extended Kalman Filter (EKF)
for 3D Trajectory Estimation with Velocity Measurements.

This script implements and compares two major backend optimization approaches
from scratch, without relying on external libraries like gtsam.

- EKF (Extended Kalman Filter): An online, recursive filtering method. It processes
  measurements sequentially and maintains a single belief (state and covariance)
  of the current pose.

- iSAM2 (Incremental Smoothing and Mapping): An incremental version of FGO.
  It efficiently updates the solution as new measurements arrive, avoiding
  full re-optimization, making it suitable for online applications.
"""
import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # 或者 'GTK3Agg', 'Qt5Agg' 等
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import least_squares, approx_fprime
from scipy.spatial.transform import Rotation
import gtsam
from gtsam.symbol_shorthand import X

#
# ==============================================================================
#  1. gtsam 辅助函数
# ==============================================================================
#

def normalize_euler_angles(angles: np.ndarray) -> np.ndarray:
    return Rotation.from_euler('xyz', angles).as_euler('xyz')

def gtsam_pose_to_numpy(pose: gtsam.Pose3) -> np.ndarray:
    """Converts a gtsam.Pose3 to a 6D numpy array (x, y, z, roll, pitch, yaw)."""
    p = pose.translation()
    r = pose.rotation().rpy() # roll, pitch, yaw
    return np.array([p[0], p[1], p[2], r[0], r[1], r[2]])

def numpy_pose_to_gtsam(pose_vec: np.ndarray) -> gtsam.Pose3:
    """Converts a 6D numpy array to a gtsam.Pose3."""
    return gtsam.Pose3(gtsam.Rot3.Ypr(pose_vec[5], pose_vec[4], pose_vec[3]), pose_vec[:3])

def add_velocity_constraint(graph, values, pose1_key, pose2_key, velocity, dt, noise_model):
    """添加速度约束到因子图中。"""
    try:
        # 获取当前位姿估计（如果存在）
        pose1 = values.atPose3(pose1_key)
        pose1_vec = gtsam_pose_to_numpy(pose1)
    except RuntimeError:
        # 如果键不存在，则无法添加基于当前估计的速度约束
        # 我们可以跳过这个约束，或者使用其他方法，比如通过里程计初始化
        print(f"警告: 键 {pose1_key} 在当前估计中不存在，使用零速度约束。")
        # 创建一个单位位姿（零位置，单位旋转）作为约束
        graph.add(gtsam.BetweenFactorPose3(
            pose1_key, pose2_key, 
            gtsam.Pose3(), 
            noise_model))
        return
    
    # 根据速度计算预期位置
    predicted_pos = pose1_vec[:3] + velocity * dt
    
    # 创建一个新位姿，其位置是预测的，方向与原来相同
    target_pose_vec = np.copy(pose1_vec)
    target_pose_vec[:3] = predicted_pos
    vel_constraint_pose = numpy_pose_to_gtsam(target_pose_vec)
    
    # 创建一个位置不确定性小、方向不确定性大的噪声模型
    vel_constraint_cov = gtsam.noiseModel.Diagonal.Sigmas(
        np.concatenate([noise_model.sigmas(), (np.pi*np.ones(3))]))
    
    # 添加一个"软"的位置约束
    graph.add(gtsam.BetweenFactorPose3(
        pose1_key, pose2_key, 
        vel_constraint_pose,
        vel_constraint_cov))

#
# ==============================================================================
#  2. 扩展卡尔曼滤波 (EKF)
# ==============================================================================
#
class EKF:
    def __init__(self, initial_pose: np.ndarray, initial_covariance: np.ndarray):
        self.x = initial_pose
        self.P = initial_covariance

    def predict(self, odom, odom_cov):
        def f(x, u):
            R = Rotation.from_euler('xyz', x[3:])
            new_pos = x[:3] + R.apply(u[:3])
            new_rot = (R * Rotation.from_rotvec(u[3:])).as_euler('xyz')
            return np.concatenate([new_pos, new_rot])
        F = approx_fprime(self.x, lambda x: f(x, odom), 1e-8)
        G = approx_fprime(odom, lambda u: f(self.x, u), 1e-8)
        self.x = f(self.x, odom)
        self.P = F @ self.P @ F.T + G @ odom_cov @ G.T

    def update(self, z, R, h, H_func=None):
        H = approx_fprime(self.x, h, 1e-8) if H_func is None else H_func(self.x)
        y = z - h(self.x)
        if len(y) == 6: y[3:] = (Rotation.from_euler('xyz', h(self.x)[3:]).inv() * Rotation.from_euler('xyz', z[3:])).as_rotvec()
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x += K @ y
        # 归一化欧拉角防止越界
        self.x[3:] = normalize_euler_angles(self.x[3:])
        self.P = (np.eye(len(self.x)) - K @ H) @ self.P

#
# ==============================================================================
#  3. 主程序：仿真、运行、对比
# ==============================================================================
#
if __name__ == '__main__':
    # --- 1. 生成3D仿真数据 (变速运动, 长时间, 大干扰) ---
    print("Generating long-term 3D simulation data with high disturbances...")
    # 将仿真时间增加到300秒 (3000步)。注意：这对于全批量优化来说计算量巨大，可能会很慢。
    num_steps = 3000
    dt = 0.1
    true_poses = np.zeros((num_steps + 1, 6))
    true_velocities = np.zeros((num_steps, 3))
    time_vector = np.arange(num_steps + 1) * dt
    for i in range(num_steps):
        t = time_vector[i]
        body_vx = 1.0 + 0.5 * np.sin(0.5 * t)
        body_vz = 0.2 * np.cos(0.8 * t)
        yaw_rate = 0.5 + 0.3 * np.cos(0.4 * t)
        prev_pose = true_poses[i,:]
        R_prev = Rotation.from_euler('xyz', prev_pose[3:])
        v_world = R_prev.apply([body_vx, 0, body_vz])
        new_pos = prev_pose[:3] + v_world * dt
        new_angles = prev_pose[3:] + np.array([0, 0, yaw_rate * dt])
        true_poses[i+1,:] = np.concatenate([new_pos, normalize_euler_angles(new_angles)])
        true_velocities[i,:] = v_world
    
    # 增加噪声和干扰
    odom_pos_std, odom_rot_std = 0.1, np.deg2rad(4)
    odom_cov = np.diag([odom_pos_std**2]*3 + [odom_rot_std**2]*3)
    
    vel_std = 0.2
    vel_cov = np.diag([vel_std**2]*3)
    
    gps_pos_std, gps_rot_std = 1.0, np.deg2rad(15)
    gps_cov = np.diag([gps_pos_std**2]*3 + [gps_rot_std**2]*3)
    
    outlier_prob = 0.02 # 2% 的概率出现外点
    
    odom_measurements, vel_measurements = [], []
    dead_reckoning_poses = np.zeros_like(true_poses)
    dead_reckoning_poses[0,:] = true_poses[0,:]
    for i in range(num_steps):
        p1, p2 = true_poses[i], true_poses[i+1]
        R1, R2 = Rotation.from_euler('xyz', p1[3:]), Rotation.from_euler('xyz', p2[3:])
        
        # 里程计测量
        true_odom = np.concatenate([R1.inv().apply(p2[:3] - p1[:3]), (R1.inv() * R2).as_rotvec()])
        odom_noise_sample = np.random.multivariate_normal(np.zeros(6), odom_cov)
        
        # 引入外点干扰
        if np.random.rand() < outlier_prob:
            print(f"--- Injecting Odometry outlier at step {i} ---")
            # 引入一个巨大的随机噪声作为外点
            odom_noise_sample = np.random.multivariate_normal(np.zeros(6), np.diag([5**2]*3 + [np.pi**2]*3))
        
        odom_measurements.append(true_odom + odom_noise_sample)
        
        # 速度测量
        vel_measurements.append(true_velocities[i,:] + np.random.multivariate_normal(np.zeros(3), vel_cov))
        
        # 航位推算
        dr_p1 = dead_reckoning_poses[i]
        dr_R1 = Rotation.from_euler('xyz', dr_p1[3:])
        meas_pos, meas_rot = odom_measurements[i][:3], Rotation.from_rotvec(odom_measurements[i][3:])
        dead_reckoning_poses[i+1,:] = np.concatenate([dr_p1[:3] + dr_R1.apply(meas_pos), (dr_R1 * meas_rot).as_euler('xyz')])
        
    # GPS测量变得更稀疏
    gps_indices = list(range(0, num_steps + 1, 60))
    gps_measurements = {}
    for i in gps_indices:
        gps_noise_sample = np.random.multivariate_normal(np.zeros(6), gps_cov)
        # 引入外点干扰
        if np.random.rand() < outlier_prob:
             print(f"--- Injecting GPS outlier at step {i} ---")
             gps_noise_sample = np.random.multivariate_normal(np.zeros(6), np.diag([50**2]*3 + [np.pi**2]*3))
        gps_measurements[i] = true_poses[i] + gps_noise_sample

    # --- 2. 运行 EKF ---
    print("\n--- Running Extended Kalman Filter (EKF) ---")
    ekf = EKF(true_poses[0], np.diag([1e-9]*6))
    ekf_poses = np.zeros_like(true_poses)
    ekf_poses[0,:] = true_poses[0]
    for i in range(num_steps):
        ekf.predict(odom_measurements[i], odom_cov)
        h_vel = lambda x: (x[:3] - ekf_poses[i, :3]) / dt
        ekf.update(vel_measurements[i], vel_cov, h_vel)
        if (i+1) in gps_measurements: ekf.update(gps_measurements[i+1], gps_cov, lambda x: x)
        ekf_poses[i+1,:] = ekf.x
        print(f"EKF Step {i+1:3d} | Est. Pose: x={ekf.x[0]:.2f}, y={ekf.x[1]:.2f}, z={ekf.x[2]:.2f}")

    # --- 3. 运行 iSAM2 增量式平滑 ---
    print("\n--- Running Incremental Smoothing and Mapping (iSAM2) ---")
    
    # iSAM2 参数
    parameters = gtsam.ISAM2Params()
    parameters.setRelinearizeThreshold(0.1)
    parameters.relinearizeSkip = 1
    isam = gtsam.ISAM2(parameters)
    
    # 存储图和初始估计值
    graph = gtsam.NonlinearFactorGraph()
    initial_estimate = gtsam.Values()
    
    # 噪声模型
    gtsam_odom_cov = gtsam.noiseModel.Diagonal.Sigmas(np.concatenate([odom_rot_std*np.ones(3), odom_pos_std*np.ones(3)]))
    gtsam_vel_cov = gtsam.noiseModel.Isotropic.Sigma(3, vel_std)
    gtsam_gps_cov = gtsam.noiseModel.Diagonal.Sigmas(np.concatenate([gps_rot_std*np.ones(3), gps_pos_std*np.ones(3)]))
    gtsam_prior_cov = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-9] * 6))

    # 添加先验因子
    initial_estimate.insert(X(0), numpy_pose_to_gtsam(true_poses[0]))
    graph.add(gtsam.PriorFactorPose3(X(0), numpy_pose_to_gtsam(true_poses[0]), gtsam_prior_cov))
    
    # 第一次更新
    isam.update(graph, initial_estimate)
    current_estimate = isam.calculateEstimate()
    
    # 清空图和初始估计值，准备下一步增量更新
    graph = gtsam.NonlinearFactorGraph()
    initial_estimate = gtsam.Values()
    
    for i in range(num_steps):
        # 添加初始估计
        # 使用航位推算的结果作为新位姿的初始值
        initial_estimate.insert(X(i + 1), numpy_pose_to_gtsam(dead_reckoning_poses[i+1]))
        
        # 添加里程计因子
        odom_pose = numpy_pose_to_gtsam(odom_measurements[i])
        graph.add(gtsam.BetweenFactorPose3(X(i), X(i+1), odom_pose, gtsam_odom_cov))
        
        # 添加速度约束因子（使用当前估计）
        add_velocity_constraint(
            graph, current_estimate, X(i), X(i+1), 
            vel_measurements[i], dt, gtsam_vel_cov)
        
        # 添加GPS因子 (如果存在)
        if (i+1) in gps_measurements:
            gps_pose = numpy_pose_to_gtsam(gps_measurements[i+1])
            graph.add(gtsam.PriorFactorPose3(X(i+1), gps_pose, gtsam_gps_cov))
            
        # iSAM2 更新
        isam.update(graph, initial_estimate)
        current_estimate = isam.calculateEstimate()
        
        # 清空图和值，为下一次迭代做准备
        graph = gtsam.NonlinearFactorGraph()
        initial_estimate = gtsam.Values()
        
        current_pose = gtsam_pose_to_numpy(current_estimate.atPose3(X(i+1)))
        print(f"iSAM2 Step {i+1:3d} | Est. Pose: x={current_pose[0]:.2f}, y={current_pose[1]:.2f}, z={current_pose[2]:.2f}")

    # 提取iSAM2最终结果
    isam2_result = isam.calculateEstimate()
    isam2_poses = np.zeros_like(true_poses)
    for i in range(num_steps + 1):
        isam2_poses[i,:] = gtsam_pose_to_numpy(isam2_result.atPose3(X(i)))

    # --- 4. 结果分析与可视化 ---
    print("\n--- Results Analysis ---")
    def calculate_rmse(est_poses, true_poses):
        return np.sqrt(np.mean(np.linalg.norm(est_poses[:,:3] - true_poses[:,:3], axis=1)**2))
    
    print("\nPosition RMSE (m):")
    print(f"  Dead Reckoning: {calculate_rmse(dead_reckoning_poses, true_poses):.4f}")
    print(f"  EKF:            {calculate_rmse(ekf_poses, true_poses):.4f}")
    print(f"  iSAM2:          {calculate_rmse(isam2_poses, true_poses):.4f}")

    # 3D轨迹图
    fig = plt.figure(figsize=(12, 12))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(true_poses[:, 0], true_poses[:, 1], true_poses[:, 2], 'g-', linewidth=3, label='Ground Truth')
    ax.plot(dead_reckoning_poses[:, 0], dead_reckoning_poses[:, 1], dead_reckoning_poses[:, 2], 'r:', label='Dead Reckoning')
    ax.plot(ekf_poses[:, 0], ekf_poses[:, 1], ekf_poses[:, 2], 'm--', label='EKF Path')
    ax.plot(isam2_poses[:, 0], isam2_poses[:, 1], isam2_poses[:, 2], 'b-', linewidth=2, label='iSAM2 Path')
    ax.set_title('3D Trajectory Comparison: EKF vs. iSAM2'), ax.legend()
    plt.savefig('3d_trajectory.png', dpi=300)
    print("保存了3D轨迹图像到 3d_trajectory.png")

    # 位置对比图
    fig2, axes = plt.subplots(3, 1, figsize=(15, 10), sharex=True)
    time_axis = np.arange(num_steps + 1) * dt
    for i, label in enumerate(['x', 'y', 'z']):
        axes[i].plot(time_axis, true_poses[:, i], 'g-', label='Ground Truth')
        axes[i].plot(time_axis, ekf_poses[:, i], 'm--', label='EKF')
        axes[i].plot(time_axis, isam2_poses[:, i], 'b-', label='iSAM2')
        axes[i].set_ylabel(f'{label} (m)'), axes[i].legend(), axes[i].grid(True)
    axes[2].set_xlabel('Time (s)'), fig2.suptitle('Position vs. Time', fontsize=16)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.savefig('position_vs_time.png', dpi=300)
    print("保存了位置对比图像到 position_vs_time.png")
    
    # 尝试显示图像，但如果失败也不影响程序运行
    try:
        plt.show()
    except Exception as e:
        print(f"无法显示图像，但已保存到文件: {e}") 