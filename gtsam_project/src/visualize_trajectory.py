#!/usr/bin/env python3
"""
AUV轨迹可视化脚本
对比因子图估计轨迹与GPS参考轨迹
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse
import os

def load_trajectory(filename):
    """加载轨迹数据"""
    try:
        trajectory = pd.read_csv(filename)
        print(f"Loaded trajectory with {len(trajectory)} points from {filename}")
        return trajectory
    except Exception as e:
        print(f"Error loading trajectory: {e}")
        return None

def load_gps_reference(filename):
    """加载GPS参考数据"""
    try:
        data = pd.read_csv(filename)
        # 过滤有效GPS数据
        valid_gps = data[(data['gps_lon'] != 0) & (data['gps_lat'] != 0)]
        print(f"Loaded {len(valid_gps)} valid GPS points from {len(data)} total points")
        return valid_gps
    except Exception as e:
        print(f"Error loading GPS reference: {e}")
        return None

def plot_2d_trajectory(trajectory, gps_ref=None, save_path=None):
    """绘制2D轨迹图"""
    plt.figure(figsize=(12, 8))
    
    # 绘制因子图估计轨迹
    plt.plot(trajectory['x'], trajectory['y'], 'b-', linewidth=2, label='Factor Graph Estimate', alpha=0.8)
    plt.scatter(trajectory['x'].iloc[0], trajectory['y'].iloc[0], c='green', s=100, marker='o', label='Start', zorder=5)
    plt.scatter(trajectory['x'].iloc[-1], trajectory['y'].iloc[-1], c='red', s=100, marker='x', label='End', zorder=5)
    
    # 如果有GPS参考数据，绘制GPS轨迹
    if gps_ref is not None:
        plt.plot(gps_ref['gps_east'], gps_ref['gps_north'], 'r--', linewidth=1, label='GPS Reference', alpha=0.6)
        plt.scatter(gps_ref['gps_east'].iloc[0], gps_ref['gps_north'].iloc[0], c='orange', s=80, marker='s', label='GPS Start', zorder=4)
    
    plt.xlabel('East (m)')
    plt.ylabel('North (m)')
    plt.title('AUV Trajectory - Factor Graph vs GPS Reference')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"2D trajectory plot saved to {save_path}")
    
    plt.show()

def plot_3d_trajectory(trajectory, gps_ref=None, save_path=None):
    """绘制3D轨迹图"""
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    
    # 绘制因子图估计轨迹
    ax.plot(trajectory['x'], trajectory['y'], trajectory['z'], 'b-', linewidth=2, label='Factor Graph Estimate')
    ax.scatter(trajectory['x'].iloc[0], trajectory['y'].iloc[0], trajectory['z'].iloc[0], 
               c='green', s=100, marker='o', label='Start')
    ax.scatter(trajectory['x'].iloc[-1], trajectory['y'].iloc[-1], trajectory['z'].iloc[-1], 
               c='red', s=100, marker='x', label='End')
    
    # 如果有GPS参考数据，绘制GPS轨迹
    if gps_ref is not None:
        ax.plot(gps_ref['gps_east'], gps_ref['gps_north'], gps_ref['gps_up'], 
                'r--', linewidth=1, label='GPS Reference', alpha=0.6)
    
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_zlabel('Up (m)')
    ax.set_title('AUV 3D Trajectory - Factor Graph vs GPS Reference')
    ax.legend()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"3D trajectory plot saved to {save_path}")
    
    plt.show()

def plot_position_error(trajectory, gps_ref, save_path=None):
    """绘制位置误差"""
    if gps_ref is None or len(gps_ref) == 0:
        print("No GPS reference data for error analysis")
        return
    
    # 简单的时间对齐（假设轨迹点数相近）
    min_len = min(len(trajectory), len(gps_ref))
    
    if min_len < 10:
        print("Not enough data points for error analysis")
        return
    
    # 计算位置误差
    dx = trajectory['x'][:min_len].values - gps_ref['gps_east'][:min_len].values
    dy = trajectory['y'][:min_len].values - gps_ref['gps_north'][:min_len].values
    dz = trajectory['z'][:min_len].values - gps_ref['gps_up'][:min_len].values
    
    # 计算欧几里得距离误差
    position_error = np.sqrt(dx**2 + dy**2 + dz**2)
    
    # 统计信息
    mean_error = np.mean(position_error)
    std_error = np.std(position_error)
    max_error = np.max(position_error)
    
    print(f"\nPosition Error Statistics:")
    print(f"Mean Error: {mean_error:.2f} m")
    print(f"Std Error: {std_error:.2f} m")
    print(f"Max Error: {max_error:.2f} m")
    print(f"RMS Error: {np.sqrt(np.mean(position_error**2)):.2f} m")
    
    # 绘制误差图
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    
    # 各轴误差
    axes[0, 0].plot(dx, 'r-', alpha=0.7)
    axes[0, 0].set_title('East Error')
    axes[0, 0].set_ylabel('Error (m)')
    axes[0, 0].grid(True, alpha=0.3)
    
    axes[0, 1].plot(dy, 'g-', alpha=0.7)
    axes[0, 1].set_title('North Error')
    axes[0, 1].set_ylabel('Error (m)')
    axes[0, 1].grid(True, alpha=0.3)
    
    axes[1, 0].plot(dz, 'b-', alpha=0.7)
    axes[1, 0].set_title('Up Error')
    axes[1, 0].set_xlabel('Sample')
    axes[1, 0].set_ylabel('Error (m)')
    axes[1, 0].grid(True, alpha=0.3)
    
    # 总位置误差
    axes[1, 1].plot(position_error, 'k-', linewidth=2)
    axes[1, 1].axhline(y=mean_error, color='r', linestyle='--', label=f'Mean: {mean_error:.2f}m')
    axes[1, 1].set_title('Position Error Magnitude')
    axes[1, 1].set_xlabel('Sample')
    axes[1, 1].set_ylabel('Error (m)')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Error analysis plot saved to {save_path}")
    
    plt.show()

def main():
    parser = argparse.ArgumentParser(description='Visualize AUV trajectory from factor graph')
    parser.add_argument('--trajectory', required=True, help='Path to trajectory CSV file')
    parser.add_argument('--gps_ref', help='Path to GPS reference CSV file')
    parser.add_argument('--output_dir', default='../results', help='Output directory for plots')
    
    args = parser.parse_args()
    
    # 创建输出目录
    os.makedirs(args.output_dir, exist_ok=True)
    
    # 加载轨迹数据
    trajectory = load_trajectory(args.trajectory)
    if trajectory is None:
        return
    
    # 加载GPS参考数据
    gps_ref = None
    if args.gps_ref:
        gps_ref = load_gps_reference(args.gps_ref)
    
    # 绘制2D轨迹
    plot_2d_trajectory(trajectory, gps_ref, 
                      os.path.join(args.output_dir, 'trajectory_2d.png'))
    
    # 绘制3D轨迹
    plot_3d_trajectory(trajectory, gps_ref, 
                      os.path.join(args.output_dir, 'trajectory_3d.png'))
    
    # 如果有GPS参考数据，进行误差分析
    if gps_ref is not None:
        plot_position_error(trajectory, gps_ref, 
                           os.path.join(args.output_dir, 'position_error.png'))
    
    print(f"\nTrajectory visualization completed!")
    print(f"Results saved to: {args.output_dir}")

if __name__ == '__main__':
    main()
