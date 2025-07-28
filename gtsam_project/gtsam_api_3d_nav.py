"""
使用GTSAM API实现3D导航与速度测量的因子图优化和扩展卡尔曼滤波

本脚本包含两种导航算法的实现：
- 扩展卡尔曼滤波 (Extended Kalman Filter, EKF)，使用Numpy从零实现。
- 增量平滑与映射 (iSAM2)，使用GTSAM库实现。
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation
from scipy.optimize import approx_fprime
import gtsam
from gtsam import (Pose3, Rot3, Point3, NonlinearFactorGraph, Values,
                   PriorFactorPose3, BetweenFactorPose3, ISAM2, ISAM2Params, symbol, CustomFactor)
from gtsam.noiseModel import Diagonal, Gaussian, Robust, mEstimator

# 在某些GTSAM版本中，FixedLagSmoother在顶层
try:
    from gtsam import FixedLagSmootherKeyTimestampMap
except ImportError:
    # 在其他版本中，它可能在不同的位置，或者需要从BatchFixedLagSmoother中获取
    # 这里我们假设它不存在，并在下面使用 BatchFixedLagSmoother
    pass

try:
    from gtsam import BatchFixedLagSmoother
    FIXED_LAG_SMOOTHER_CLASS = BatchFixedLagSmoother
    print("Using BatchFixedLagSmoother.")
except ImportError:
    try:
        from gtsam import ISAM2FixedLagSmoother
        FIXED_LAG_SMOOTHER_CLASS = ISAM2FixedLagSmoother
        print("Using ISAM2FixedLagSmoother.")
    except ImportError:
        print("Could not find a FixedLagSmoother implementation. Falling back to standard ISAM2.")
        FIXED_LAG_SMOOTHER_CLASS = None

# ==============================================================================
#  0. GTSAM 符号定义
# ==============================================================================

def X(i):
    """为位姿变量创建符号"""
    return symbol('x', i)

# ==============================================================================
#  1. 数据生成函数 (与原始版本保持一致)
# ==============================================================================

def normalize_euler_angles(angles: np.ndarray) -> np.ndarray:
    return Rotation.from_euler('xyz', angles).as_euler('xyz')

def generate_simulation_data(num_steps=300, dt=0.1, outlier_prob=0.02):
    """生成3D仿真数据，包括真实轨迹、里程计、速度和GPS测量"""
    print("Generating 3D simulation data...")
    
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
    
    odom_pos_std, odom_rot_std = 0.1, np.deg2rad(4)
    odom_cov = np.diag([odom_pos_std**2]*3 + [odom_rot_std**2]*3)
    vel_std = 0.2
    vel_cov = np.diag([vel_std**2]*3)
    gps_pos_std, gps_rot_std = 1.0, np.deg2rad(15)
    gps_cov = np.diag([gps_pos_std**2]*3 + [gps_rot_std**2]*3)
    
    odom_measurements, vel_measurements = [], []
    dead_reckoning_poses = np.zeros_like(true_poses)
    dead_reckoning_poses[0,:] = true_poses[0,:]
    
    for i in range(num_steps):
        p1, p2 = true_poses[i], true_poses[i+1]
        R1, R2 = Rotation.from_euler('xyz', p1[3:]), Rotation.from_euler('xyz', p2[3:])
        
        true_odom = np.concatenate([R1.inv().apply(p2[:3] - p1[:3]), (R1.inv() * R2).as_rotvec()])
        odom_noise = np.random.multivariate_normal(np.zeros(6), odom_cov)
        
        if np.random.rand() < outlier_prob:
            print(f"--- Adding odometry outlier at step {i} ---")
            odom_noise = np.random.multivariate_normal(np.zeros(6), np.diag([5**2]*3 + [np.pi**2]*3))
        
        odom_measurements.append(true_odom + odom_noise)
        vel_measurements.append(true_velocities[i,:] + np.random.multivariate_normal(np.zeros(3), vel_cov))
        
        dr_p1 = dead_reckoning_poses[i]
        dr_R1 = Rotation.from_euler('xyz', dr_p1[3:])
        meas_pos, meas_rot = odom_measurements[i][:3], Rotation.from_rotvec(odom_measurements[i][3:])
        dead_reckoning_poses[i+1,:] = np.concatenate([
            dr_p1[:3] + dr_R1.apply(meas_pos), 
            (dr_R1 * meas_rot).as_euler('xyz')
        ])
    
    # 根据dt自动计算GPS的更新间隔，以达到1Hz的更新频率
    gps_update_interval_steps = max(1, int(round(1.0 / dt)))
    gps_indices = list(range(0, num_steps + 1, gps_update_interval_steps))
    gps_measurements = {}
    for i in gps_indices:
        gps_noise = np.random.multivariate_normal(np.zeros(6), gps_cov)
        
        if np.random.rand() < outlier_prob:
            print(f"--- Adding GPS outlier at step {i} ---")
            gps_noise = np.random.multivariate_normal(np.zeros(6), np.diag([50**2]*3 + [np.pi**2]*3))
        
        gps_measurements[i] = true_poses[i] + gps_noise
    
    return {
        'true_poses': true_poses, 'true_velocities': true_velocities,
        'odom_measurements': odom_measurements, 'vel_measurements': vel_measurements,
        'gps_measurements': gps_measurements, 'dead_reckoning_poses': dead_reckoning_poses,
        'dt': dt, 'odom_cov': odom_cov, 'vel_cov': vel_cov, 'gps_cov': gps_cov
    }

# ==============================================================================
#  2. 扩展卡尔曼滤波 (EKF)
# ==============================================================================

class EKF:
    def __init__(self, initial_pose: np.ndarray, initial_covariance: np.ndarray):
        self.x = initial_pose
        self.P = initial_covariance

    def predict(self, odom, odom_cov):
        def f(x, u):
            R = Rotation.from_euler('xyz', x[3:])
            new_pos = x[:3] + R.apply(u[:3])
            new_rot_vec = (R * Rotation.from_rotvec(u[3:])).as_euler('xyz')
            return np.concatenate([new_pos, new_rot_vec])

        F = approx_fprime(self.x, lambda x: f(x, odom), 1e-8)
        self.x = f(self.x, odom)
        self.P = F @ self.P @ F.T + odom_cov

    def update(self, z, R, h):
        H = approx_fprime(self.x, h, 1e-8)
        y = z - h(self.x)
        
        is_pose_update = len(y) == 6
        if is_pose_update:
            h_rot = Rotation.from_euler('xyz', h(self.x)[3:])
            z_rot = Rotation.from_euler('xyz', z[3:])
            rot_error = (h_rot.inv() * z_rot).as_rotvec()
            y[3:] = rot_error
            
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x += K @ y
        if is_pose_update:
            self.x[3:] = normalize_euler_angles(self.x[3:])
        self.P = (np.eye(len(self.x)) - K @ H) @ self.P

def run_ekf(data):
    """使用纯Numpy运行EKF"""
    print("\n--- Running Pure EKF ---")
    num_steps = len(data['odom_measurements'])
    dt = data['dt']
    
    ekf = EKF(data['true_poses'][0], np.diag([1e-9]*6))
    ekf_poses = np.zeros_like(data['true_poses'])
    ekf_poses[0,:] = data['true_poses'][0]

    for i in range(num_steps):
        ekf.predict(data['odom_measurements'][i], data['odom_cov'])
        
        h_vel = lambda x: (x[:3] - ekf_poses[i, :3]) / dt
        ekf.update(data['vel_measurements'][i], data['vel_cov'], h_vel)
        
        if (i+1) in data['gps_measurements']:
            ekf.update(data['gps_measurements'][i+1], data['gps_cov'], lambda x: x)
            
        ekf_poses[i+1,:] = ekf.x
        if (i+1) % 50 == 0:
            print(f"EKF step {i+1} | Est. Pose: x={ekf.x[0]:.2f}, y={ekf.x[1]:.2f}, z={ekf.x[2]:.2f}")
    
    print("EKF completed!")
    return ekf_poses

# ==============================================================================
#  3. GTSAM iSAM2 增量平滑
# ==============================================================================

def pose3_from_array(arr):
    return Pose3(Rot3.RzRyRx(arr[3], arr[4], arr[5]), Point3(arr[0], arr[1], arr[2]))

def array_from_pose3(pose):
    return np.array([pose.x(), pose.y(), pose.z(), 
                     pose.rotation().roll(), pose.rotation().pitch(), pose.rotation().yaw()])

def run_gtsam_isam2(data):
    """使用GTSAM运行iSAM2增量平滑"""
    print("\n--- Running GTSAM iSAM2 Incremental Smoothing ---")
    
    try:
        num_steps = len(data['odom_measurements'])
        dt = data['dt']
        
        isam_params = ISAM2Params()
        isam_params.setFactorization("QR")
        isam_params.setRelinearizeThreshold(0.1)
        isam = ISAM2(isam_params)
        
        graph = NonlinearFactorGraph()
        values = Values()
        
        prior_noise = Diagonal.Sigmas(np.array([1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6]))
        initial_pose = pose3_from_array(data['true_poses'][0])
        graph.add(PriorFactorPose3(X(0), initial_pose, prior_noise))
        values.insert(X(0), initial_pose)
        
        # 使用Huber鲁棒噪声模型来处理外点
        huber_k = 1.345
        odom_noise_gaussian = Gaussian.Covariance(data['odom_cov'])
        odom_noise = Robust.Create(mEstimator.Huber.Create(huber_k), odom_noise_gaussian)
        
        gps_noise_gaussian = Gaussian.Covariance(data['gps_cov'])
        gps_noise = Robust.Create(mEstimator.Huber.Create(huber_k), gps_noise_gaussian)
        
        isam.update(graph, values)
        result = isam.calculateEstimate()
        
        isam2_poses = np.zeros_like(data['true_poses'])
        isam2_poses[0] = array_from_pose3(result.atPose3(X(0)))
        
        for i in range(num_steps):
            graph = NonlinearFactorGraph()
            values = Values()
            
            prev_pose = result.atPose3(X(i))
            
            odom = data['odom_measurements'][i]
            relative_pose = Pose3(Rot3.Rodrigues(odom[3:]), Point3(odom[:3]))
            
            next_pose_est = prev_pose.compose(relative_pose)
            values.insert(X(i+1), next_pose_est)
            
            graph.add(BetweenFactorPose3(X(i), X(i+1), relative_pose, odom_noise))
            
            # --- 自定义速度因子 ---
            def make_velocity_error_func(measured_vel, dt):
                def velocity_error_func(this, values, H=None):
                    key1 = this.keys()[0]
                    key2 = this.keys()[1]
                    pose1 = values.atPose3(key1)
                    pose2 = values.atPose3(key2)
                    
                    predicted_vel = (pose2.translation() - pose1.translation()) / dt
                    error = predicted_vel - measured_vel
                    
                    if H is not None:
                        # 雅可比矩阵 wrt pose1, tangent space is (omega, v)
                        J_p1 = np.zeros((3, 6))
                        J_p1[:, 3:6] = -pose1.rotation().matrix() / dt
                        H[0] = J_p1
                        
                        # 雅可比矩阵 wrt pose2
                        J_p2 = np.zeros((3, 6))
                        J_p2[:, 3:6] = pose2.rotation().matrix() / dt
                        H[1] = J_p2
                        
                    return error
                return velocity_error_func

            vel_noise = Diagonal.Sigmas(np.sqrt(np.diag(data['vel_cov'])))
            measured_vel = data['vel_measurements'][i]
            graph.add(CustomFactor(vel_noise, [X(i), X(i+1)], make_velocity_error_func(measured_vel, dt)))
            
            if (i+1) in data['gps_measurements']:
                gps_pose = pose3_from_array(data['gps_measurements'][i+1])
                graph.add(PriorFactorPose3(X(i+1), gps_pose, gps_noise))
            
            isam.update(graph, values)
            result = isam.calculateEstimate()
            isam2_poses[i+1] = array_from_pose3(result.atPose3(X(i+1)))
            
            if (i+1) % 50 == 0:
                print(f"iSAM2 step {i+1} | Est. Pose: x={isam2_poses[i+1,0]:.2f}, y={isam2_poses[i+1,1]:.2f}, z={isam2_poses[i+1,2]:.2f}")
        
        print("GTSAM iSAM2 incremental smoothing completed!")
        return isam2_poses
    except Exception as e:
        print(f"GTSAM iSAM2 error: {e}")
        return data['dead_reckoning_poses']

# ==============================================================================
#  4. 结果分析与可视化
# ==============================================================================

def calculate_errors(est_poses, true_poses):
    """计算位置和旋转误差"""
    pos_errors = np.linalg.norm(est_poses[:, :3] - true_poses[:, :3], axis=1)
    pos_rmse = np.sqrt(np.mean(pos_errors**2))
    
    rot_errors = []
    for i in range(len(est_poses)):
        R_est = Rotation.from_euler('xyz', est_poses[i, 3:])
        R_true = Rotation.from_euler('xyz', true_poses[i, 3:])
        error_rot = R_true.inv() * R_est
        # as_rotvec()返回旋转向量，其范数即为旋转角
        rot_error_angle = np.linalg.norm(error_rot.as_rotvec())
        rot_errors.append(rot_error_angle)
    
    rot_errors = np.array(rot_errors)
    rot_rmse = np.sqrt(np.mean(rot_errors**2))
    
    return pos_errors, pos_rmse, rot_errors, rot_rmse

def visualize_results(data, ekf_poses, isam2_poses):
    """可视化轨迹并对误差进行详细分析"""
    true_poses = data['true_poses']
    
    # --- 计算所有轨迹的误差 ---
    ekf_pos_err, ekf_pos_rmse, ekf_rot_err, ekf_rot_rmse = calculate_errors(ekf_poses, true_poses)
    isam2_pos_err, isam2_pos_rmse, isam2_rot_err, isam2_rot_rmse = calculate_errors(isam2_poses, true_poses)

    print("\n--- Detailed Error Analysis ---")
    print("Position RMSE (m):")
    print(f"  - Pure EKF:       {ekf_pos_rmse:.4f}")
    print(f"  - GTSAM iSAM2:    {isam2_pos_rmse:.4f}")

    print("\nRotation RMSE (deg):")
    print(f"  - Pure EKF:       {np.rad2deg(ekf_rot_rmse):.4f}")
    print(f"  - GTSAM iSAM2:    {np.rad2deg(isam2_rot_rmse):.4f}")
    
    # --- 绘制3D轨迹对比图 ---
    fig = plt.figure(figsize=(12, 12))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(true_poses[:, 0], true_poses[:, 1], true_poses[:, 2], 'g-', linewidth=3, label='Ground Truth')
    ax.plot(ekf_poses[:, 0], ekf_poses[:, 1], ekf_poses[:, 2], 'm--', label='Pure EKF')
    ax.plot(isam2_poses[:, 0], isam2_poses[:, 1], isam2_poses[:, 2], 'c-', linewidth=2, label='GTSAM iSAM2')
    ax.set_title('3D Trajectory Comparison: EKF vs. iSAM2')
    ax.legend()
    
    # --- 绘制XYZ坐标随时间变化图 ---
    fig2, axes = plt.subplots(3, 1, figsize=(15, 10), sharex=True)
    time_axis = np.arange(len(true_poses)) * data['dt']
    for i, label in enumerate(['x', 'y', 'z']):
        axes[i].plot(time_axis, true_poses[:, i], 'g-', label='Ground Truth')
        axes[i].plot(time_axis, ekf_poses[:, i], 'm--', label='EKF')
        axes[i].plot(time_axis, isam2_poses[:, i], 'c-', label='iSAM2')
        axes[i].set_ylabel(f'{label} (m)')
        axes[i].legend()
        axes[i].grid(True)
    axes[2].set_xlabel('Time (s)')
    fig2.suptitle('Position vs. Time', fontsize=16)
    plt.tight_layout(rect=[0, 0, 1, 0.96])

    # --- 绘制误差随时间变化图 ---
    _, dr_pos_err, _, dr_rot_err = calculate_errors(data['dead_reckoning_poses'], true_poses) # Keep for context if needed elsewhere, but don't plot
    fig_err, axes_err = plt.subplots(2, 1, figsize=(15, 8), sharex=True)
    axes_err[0].plot(time_axis, ekf_pos_err, 'm--', label='EKF')
    axes_err[0].plot(time_axis, isam2_pos_err, 'c-', label='iSAM2')
    axes_err[0].set_ylabel('Position Error (m)')
    axes_err[0].legend()
    axes_err[0].grid(True)

    axes_err[1].plot(time_axis, np.rad2deg(ekf_rot_err), 'm--', label='EKF')
    axes_err[1].plot(time_axis, np.rad2deg(isam2_rot_err), 'c-', label='iSAM2')
    axes_err[1].set_ylabel('Rotation Error (deg)')
    axes_err[1].set_xlabel('Time (s)')
    axes_err[1].legend()
    axes_err[1].grid(True)
    fig_err.suptitle('Error vs. Time', fontsize=16)
    plt.tight_layout(rect=[0, 0, 1, 0.96])

    # --- 绘制RMSE对比柱状图 ---
    labels = ['Pure EKF', 'GTSAM iSAM2']
    pos_rmses = [ekf_pos_rmse, isam2_pos_rmse]
    _, dr_pos_rmse, _, dr_rot_rmse = calculate_errors(data['dead_reckoning_poses'], true_poses) # Recalculate for context if needed, but don't plot
    rot_rmses_deg = [np.rad2deg(ekf_rot_rmse), np.rad2deg(isam2_rot_rmse)]

    fig_rmse, axes_rmse = plt.subplots(1, 2, figsize=(14, 6))
    colors = ['m', 'c']

    # Position RMSE
    bars_pos = axes_rmse[0].bar(labels, pos_rmses, color=colors)
    axes_rmse[0].set_title('Position RMSE Comparison')
    axes_rmse[0].set_ylabel('RMSE (m)')
    axes_rmse[0].bar_label(bars_pos, fmt='%.4f')
    
    # Rotation RMSE
    bars_rot = axes_rmse[1].bar(labels, rot_rmses_deg, color=colors)
    axes_rmse[1].set_title('Rotation RMSE Comparison')
    axes_rmse[1].set_ylabel('RMSE (deg)')
    axes_rmse[1].bar_label(bars_rot, fmt='%.4f')

    fig_rmse.suptitle('Overall RMSE Comparison', fontsize=16)
    plt.tight_layout(rect=[0, 0, 1, 0.94])
    
    print("\nSaving result images...")
    fig.savefig('gtsam_trajectory_comparison.png')
    print("- Trajectory plot saved: gtsam_trajectory_comparison.png")
    fig2.savefig('gtsam_position_comparison.png')
    print("- Position plot saved: gtsam_position_comparison.png")
    fig_err.savefig('gtsam_error_comparison.png')
    print("- Error plot saved: gtsam_error_comparison.png")
    fig_rmse.savefig('gtsam_rmse_comparison.png')
    print("- RMSE plot saved: gtsam_rmse_comparison.png")
    
    try:
        plt.show()
    except:
        print("\nCannot display images, please check the saved PNG files.")

# ==============================================================================
#  5. 主函数
# ==============================================================================

if __name__ == '__main__':
    # 将里程计更新频率设为100Hz (dt=0.01)，GPS更新频率设为1Hz (在函数内自动计算)
    # 仿真总时长为30秒
    data = generate_simulation_data(num_steps=3000, dt=0.01)
    
    ekf_poses = run_ekf(data)
    isam2_poses = run_gtsam_isam2(data)
    
    visualize_results(data, ekf_poses, isam2_poses) 