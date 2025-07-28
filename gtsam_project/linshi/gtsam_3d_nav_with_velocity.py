"""
Comparison of Full Batch Factor Graph Optimization (FGO) and Extended Kalman Filter (EKF)
for 3D Trajectory Estimation with Velocity Measurements.

This script implements and compares two major backend optimization approaches
from scratch, without relying on external libraries like gtsam.

- EKF (Extended Kalman Filter): An online, recursive filtering method. It processes
  measurements sequentially and maintains a single belief (state and covariance)
  of the current pose.

- Full Batch FGO: A full smoothing method that considers all measurements
  simultaneously to find the globally optimal solution. It is generally more
  accurate but requires all data to be available and is computationally more expensive.
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import least_squares, approx_fprime
from scipy.spatial.transform import Rotation

#
# ==============================================================================
#  1. 核心因子图类 (用于批量优化)
# ==============================================================================
#

def normalize_euler_angles(angles: np.ndarray) -> np.ndarray:
    return Rotation.from_euler('xyz', angles).as_euler('xyz')

class StateFGO:
    def __init__(self, state_id: int, pose: np.ndarray):
        self.id = state_id
        self.pose = pose

class Factor:
    def __init__(self, state_ids: list, measurement: np.ndarray, covariance: np.ndarray):
        self.state_ids = state_ids
        self.measurement = measurement
        self.information = np.linalg.inv(covariance)
        self.sqrt_information = np.linalg.cholesky(self.information)

    def compute_error(self, states: dict) -> np.ndarray:
        raise NotImplementedError

class PriorFactorFGO(Factor):
    def __init__(self, state_id: int, measurement: np.ndarray, covariance: np.ndarray):
        super().__init__([state_id], measurement, covariance)

    def compute_error(self, states: dict) -> np.ndarray:
        error = np.zeros(6)
        error[:3] = states[self.state_ids[0]].pose[:3] - self.measurement[:3]
        error[3:] = (Rotation.from_euler('xyz', self.measurement[3:]).inv() * Rotation.from_euler('xyz', states[self.state_ids[0]].pose[3:])).as_rotvec()
        return error

class OdometryFactorFGO(Factor):
    def __init__(self, state_id1: int, state_id2: int, measurement: np.ndarray, covariance: np.ndarray):
        super().__init__([state_id1, state_id2], measurement, covariance)

    def compute_error(self, states: dict) -> np.ndarray:
        s1, s2 = states[self.state_ids[0]], states[self.state_ids[1]]
        R1, R2 = Rotation.from_euler('xyz', s1.pose[3:]), Rotation.from_euler('xyz', s2.pose[3:])
        pred_pos_diff = R1.inv().apply(s2.pose[:3] - s1.pose[:3])
        pred_rot_diff = R1.inv() * R2
        pos_error = pred_pos_diff - self.measurement[:3]
        rot_error = (Rotation.from_rotvec(self.measurement[3:]).inv() * pred_rot_diff).as_rotvec()
        return np.concatenate([pos_error, rot_error])

class VelocityFactorFGO(Factor):
    def __init__(self, state_id1: int, state_id2: int, measurement: np.ndarray, covariance: np.ndarray, dt: float):
        super().__init__([state_id1, state_id2], measurement, covariance)
        self.dt = dt

    def compute_error(self, states: dict) -> np.ndarray:
        s1, s2 = states[self.state_ids[0]], states[self.state_ids[1]]
        pred_vel = (s2.pose[:3] - s1.pose[:3]) / self.dt
        return pred_vel - self.measurement

class FactorGraph:
    def __init__(self):
        self.states = {}
        self.factors = []

    def add_factor(self, factor: Factor):
        self.factors.append(factor)

    def optimize(self, max_iterations=20, verbose=0):
        print("\n--- Starting Full Batch Factor Graph Optimization ---")
        sorted_states = sorted(self.states.values(), key=lambda s: s.id)
        initial_poses = np.concatenate([s.pose for s in sorted_states])
        
        def residuals_func(poses_vec):
            for i, state in enumerate(sorted_states):
                state.pose = poses_vec[i*6:(i+1)*6]
            all_residuals = []
            for factor in self.factors:
                error = factor.compute_error(self.states)
                weighted_error = factor.sqrt_information @ error
                all_residuals.extend(weighted_error)
            return np.array(all_residuals)
        
        result = least_squares(residuals_func, initial_poses, method='lm', max_nfev=max_iterations, verbose=verbose)
        
        optimized_poses = result.x.reshape(-1, 6)
        final_poses = {}
        for i, state in enumerate(sorted_states):
            state.pose = optimized_poses[i, :]
            final_poses[state.id] = state.pose
        
        print("FGO Finished!")
        return final_poses

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
        self.P = (np.eye(len(self.x)) - K @ H) @ self.P

#
# ==============================================================================
#  3. 主程序：仿真、运行、对比
# ==============================================================================
#
if __name__ == '__main__':
    # --- 1. 生成3D仿真数据 (变速运动, 长时间, 大干扰) ---
    print("Generating long-term 3D simulation data with high disturbances...")
    # 将仿真时间设置为50秒 (500步)
    num_steps = 300
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

    # --- 3. 运行全批量FGO ---
    full_graph = FactorGraph()
    for i in range(num_steps + 1):
        full_graph.states[i] = StateFGO(i, dead_reckoning_poses[i])
    full_graph.add_factor(PriorFactorFGO(0, true_poses[0], np.diag([1e-9]*6)))
    for i in range(num_steps):
        full_graph.add_factor(OdometryFactorFGO(i, i + 1, odom_measurements[i], odom_cov))
        full_graph.add_factor(VelocityFactorFGO(i, i + 1, vel_measurements[i], vel_cov, dt))
        if (i+1) in gps_measurements: full_graph.add_factor(PriorFactorFGO(i+1, gps_measurements[i+1], gps_cov))
    
    fgo_poses_dict = full_graph.optimize(verbose=1)
    fgo_poses = np.array([fgo_poses_dict[i] for i in sorted(fgo_poses_dict.keys())])

    # --- 4. 结果分析与可视化 ---
    print("\n--- Results Analysis ---")
    def calculate_rmse(est_poses, true_poses):
        return np.sqrt(np.mean(np.linalg.norm(est_poses[:,:3] - true_poses[:,:3], axis=1)**2))
    
    print("\nPosition RMSE (m):")
    print(f"  Dead Reckoning: {calculate_rmse(dead_reckoning_poses, true_poses):.4f}")
    print(f"  EKF:            {calculate_rmse(ekf_poses, true_poses):.4f}")
    print(f"  Full Batch FGO: {calculate_rmse(fgo_poses, true_poses):.4f}")

    # 3D轨迹图
    fig = plt.figure(figsize=(12, 12))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(true_poses[:, 0], true_poses[:, 1], true_poses[:, 2], 'g-', linewidth=3, label='Ground Truth')
    ax.plot(dead_reckoning_poses[:, 0], dead_reckoning_poses[:, 1], dead_reckoning_poses[:, 2], 'r:', label='Dead Reckoning')
    ax.plot(ekf_poses[:, 0], ekf_poses[:, 1], ekf_poses[:, 2], 'm--', label='EKF Path')
    ax.plot(fgo_poses[:, 0], fgo_poses[:, 1], fgo_poses[:, 2], 'b-', linewidth=2, label='Full Batch FGO Path')
    ax.set_title('3D Trajectory Comparison: EKF vs. Full Batch FGO'), ax.legend()

    # 位置对比图
    fig2, axes = plt.subplots(3, 1, figsize=(15, 10), sharex=True)
    time_axis = np.arange(num_steps + 1) * dt
    for i, label in enumerate(['x', 'y', 'z']):
        axes[i].plot(time_axis, true_poses[:, i], 'g-', label='Ground Truth')
        axes[i].plot(time_axis, ekf_poses[:, i], 'm--', label='EKF')
        axes[i].plot(time_axis, fgo_poses[:, i], 'b-', label='FGO')
        axes[i].set_ylabel(f'{label} (m)'), axes[i].legend(), axes[i].grid(True)
    axes[2].set_xlabel('Time (s)'), fig2.suptitle('Position vs. Time', fontsize=16)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    # 保存图像到文件而不是尝试显示
    print("保存轨迹图像到 trajectory_comparison.png")
    plt.savefig('trajectory_comparison.png')
    print("保存位置对比图到 position_comparison.png")
    plt.figure(2)  # 切换到第二个图形
    plt.savefig('position_comparison.png')
    
    # 如果有GUI环境，尝试显示图像
    try:
        plt.show()
    except Exception as e:
        print(f"无法显示图像，请查看保存的PNG文件。错误: {e}")