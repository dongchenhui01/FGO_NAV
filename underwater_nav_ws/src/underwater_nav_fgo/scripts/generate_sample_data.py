#!/usr/bin/env python3
"""
生成示例水下导航数据的脚本
用于测试和演示系统功能
"""

import numpy as np
import pandas as pd
import argparse
from datetime import datetime, timedelta
import math

class UnderwaterDataGenerator:
    def __init__(self):
        # 仿真参数
        self.dt_imu = 0.01  # IMU采样间隔 (100Hz)
        self.dt_dvl = 0.1   # DVL采样间隔 (10Hz)
        self.duration = 300  # 仿真时长 (秒)
        
        # 噪声参数
        self.acc_noise_std = 0.01    # 加速度计噪声 (m/s²)
        self.gyr_noise_std = 0.0017  # 陀螺仪噪声 (rad/s)
        self.mag_noise_std = 0.1     # 磁力计噪声
        self.dvl_noise_std = 0.02    # DVL噪声 (m/s)
        
        # 偏差参数
        self.acc_bias = np.array([0.05, -0.03, 0.02])  # 加速度计偏差
        self.gyr_bias = np.array([0.001, -0.002, 0.0005])  # 陀螺仪偏差
        
        # 环境参数
        self.gravity = 9.81
        self.magnetic_field = np.array([1.0, 0.0, 0.5])  # 本地磁场
        
    def generate_trajectory(self):
        """生成参考轨迹"""
        t = np.arange(0, self.duration, self.dt_imu)
        
        # 生成螺旋下降轨迹 (模拟AUV下潜过程)
        radius = 10.0
        descent_rate = 0.1  # m/s
        angular_velocity = 0.1  # rad/s
        
        # 位置
        x = radius * np.cos(angular_velocity * t)
        y = radius * np.sin(angular_velocity * t)
        z = -descent_rate * t  # 向下为负
        
        # 速度
        vx = -radius * angular_velocity * np.sin(angular_velocity * t)
        vy = radius * angular_velocity * np.cos(angular_velocity * t)
        vz = -descent_rate * np.ones_like(t)
        
        # 加速度
        ax = -radius * angular_velocity**2 * np.cos(angular_velocity * t)
        ay = -radius * angular_velocity**2 * np.sin(angular_velocity * t)
        az = np.zeros_like(t)
        
        # 姿态 (航向角随轨迹变化)
        roll = 0.1 * np.sin(0.5 * t)  # 小幅横滚
        pitch = 0.05 * np.cos(0.3 * t)  # 小幅俯仰
        yaw = angular_velocity * t + 0.2 * np.sin(0.2 * t)  # 主要航向变化
        
        return {
            'time': t,
            'position': np.column_stack([x, y, z]),
            'velocity': np.column_stack([vx, vy, vz]),
            'acceleration': np.column_stack([ax, ay, az]),
            'attitude': np.column_stack([roll, pitch, yaw])
        }
    
    def rotation_matrix(self, roll, pitch, yaw):
        """计算旋转矩阵 (ZYX欧拉角)"""
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        
        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp, cp*sr, cp*cr]
        ])
        return R
    
    def generate_imu_data(self, trajectory):
        """生成IMU数据"""
        t = trajectory['time']
        acc_true = trajectory['acceleration']
        att = trajectory['attitude']
        
        imu_data = []
        
        for i in range(len(t)):
            # 真实加速度 (导航坐标系)
            acc_nav = acc_true[i] + np.array([0, 0, self.gravity])
            
            # 转换到载体坐标系
            R = self.rotation_matrix(att[i, 0], att[i, 1], att[i, 2])
            acc_body = R.T @ acc_nav
            
            # 添加偏差和噪声
            acc_measured = acc_body + self.acc_bias + np.random.normal(0, self.acc_noise_std, 3)
            
            # 角速度 (简化处理，假设匀速转动)
            if i > 0:
                dt = t[i] - t[i-1]
                datt = att[i] - att[i-1]
                gyr_true = datt / dt
            else:
                gyr_true = np.zeros(3)
            
            gyr_measured = gyr_true + self.gyr_bias + np.random.normal(0, self.gyr_noise_std, 3)
            
            # 磁场测量 (载体坐标系)
            mag_nav = self.magnetic_field
            mag_body = R.T @ mag_nav
            mag_measured = mag_body + np.random.normal(0, self.mag_noise_std, 3)
            
            imu_data.append({
                'time': t[i],
                'acc_x': acc_measured[0],
                'acc_y': acc_measured[1],
                'acc_z': acc_measured[2],
                'gyr_x': gyr_measured[0],
                'gyr_y': gyr_measured[1],
                'gyr_z': gyr_measured[2],
                'mag_x': mag_measured[0],
                'mag_y': mag_measured[1],
                'mag_z': mag_measured[2]
            })
        
        return imu_data
    
    def generate_dvl_data(self, trajectory):
        """生成DVL数据"""
        # DVL数据频率较低，需要重采样
        t_dvl = np.arange(0, self.duration, self.dt_dvl)
        
        # 插值获取DVL时刻的速度
        vel_interp = np.zeros((len(t_dvl), 3))
        att_interp = np.zeros((len(t_dvl), 3))
        
        for i in range(3):
            vel_interp[:, i] = np.interp(t_dvl, trajectory['time'], trajectory['velocity'][:, i])
            att_interp[:, i] = np.interp(t_dvl, trajectory['time'], trajectory['attitude'][:, i])
        
        dvl_data = []
        
        for i in range(len(t_dvl)):
            # 真实速度 (导航坐标系)
            vel_nav = vel_interp[i]
            
            # 转换到载体坐标系
            R = self.rotation_matrix(att_interp[i, 0], att_interp[i, 1], att_interp[i, 2])
            vel_body = R.T @ vel_nav
            
            # 添加噪声
            vel_measured = vel_body + np.random.normal(0, self.dvl_noise_std, 3)
            
            dvl_data.append({
                'time': t_dvl[i],
                'dvl_vx': vel_measured[0],
                'dvl_vy': vel_measured[1],
                'dvl_vz': vel_measured[2]
            })
        
        return dvl_data
    
    def generate_gps_reference(self, trajectory):
        """生成GPS参考数据 (用于精度评估)"""
        # GPS数据频率为1Hz
        t_gps = np.arange(0, self.duration, 1.0)
        
        # 插值获取GPS时刻的位置和姿态
        pos_interp = np.zeros((len(t_gps), 3))
        att_interp = np.zeros((len(t_gps), 3))
        vel_interp = np.zeros((len(t_gps), 3))
        
        for i in range(3):
            pos_interp[:, i] = np.interp(t_gps, trajectory['time'], trajectory['position'][:, i])
            att_interp[:, i] = np.interp(t_gps, trajectory['time'], trajectory['attitude'][:, i])
            vel_interp[:, i] = np.interp(t_gps, trajectory['time'], trajectory['velocity'][:, i])
        
        gps_data = []
        
        # 参考点 (假设为起始位置)
        ref_lat, ref_lon = 40.0, -74.0  # 纽约附近
        
        for i in range(len(t_gps)):
            # 转换为经纬度 (简化处理)
            lat = ref_lat + pos_interp[i, 1] / 111320.0  # 1度纬度约111320米
            lon = ref_lon + pos_interp[i, 0] / (111320.0 * np.cos(np.radians(ref_lat)))
            alt = -pos_interp[i, 2]  # GPS高度为正
            
            gps_data.append({
                'time': t_gps[i],
                'gps_lat': lat,
                'gps_lon': lon,
                'gps_alt': alt,
                'gps_heading': np.degrees(att_interp[i, 2]),
                'gps_pitch': np.degrees(att_interp[i, 1]),
                'gps_roll': np.degrees(att_interp[i, 0]),
                'gps_track': np.degrees(att_interp[i, 2]),
                'gps_vel': np.linalg.norm(vel_interp[i]),
                'gps_pos_qual': 4,  # 固定解
                'gps_head_qual': 4,
                'gps_h_svs': 8,
                'gps_m_svs': 8,
                'gps_east': pos_interp[i, 0],
                'gps_north': pos_interp[i, 1],
                'gps_up': -pos_interp[i, 2],
                'gps_east_vel': vel_interp[i, 0],
                'gps_north_vel': vel_interp[i, 1],
                'gps_up_vel': -vel_interp[i, 2]
            })
        
        return gps_data
    
    def merge_and_save(self, imu_data, dvl_data, gps_data, filename):
        """合并数据并保存为CSV"""
        # 创建完整的时间序列
        all_times = set()
        
        # 添加所有时间点
        for data in imu_data:
            all_times.add(data['time'])
        for data in dvl_data:
            all_times.add(data['time'])
        for data in gps_data:
            all_times.add(data['time'])
        
        all_times = sorted(all_times)
        
        # 创建数据字典
        merged_data = []
        
        # 为每个时间点创建数据行
        imu_idx = 0
        dvl_idx = 0
        gps_idx = 0
        
        for t in all_times:
            row = {'utc_timestamp': datetime.utcfromtimestamp(t).isoformat() + 'Z'}
            
            # 查找最近的IMU数据
            while imu_idx < len(imu_data) - 1 and abs(imu_data[imu_idx + 1]['time'] - t) < abs(imu_data[imu_idx]['time'] - t):
                imu_idx += 1
            
            if imu_idx < len(imu_data):
                imu = imu_data[imu_idx]
                row.update({
                    'acc_x': imu['acc_x'],
                    'acc_y': imu['acc_y'],
                    'acc_z': imu['acc_z'],
                    'gyr_x': imu['gyr_x'],
                    'gyr_y': imu['gyr_y'],
                    'gyr_z': imu['gyr_z'],
                    'mag_x': imu['mag_x'],
                    'mag_y': imu['mag_y'],
                    'mag_z': imu['mag_z']
                })
            
            # 查找最近的DVL数据
            while dvl_idx < len(dvl_data) - 1 and abs(dvl_data[dvl_idx + 1]['time'] - t) < abs(dvl_data[dvl_idx]['time'] - t):
                dvl_idx += 1
            
            if dvl_idx < len(dvl_data):
                dvl = dvl_data[dvl_idx]
                row.update({
                    'dvl_vx': dvl['dvl_vx'],
                    'dvl_vy': dvl['dvl_vy'],
                    'dvl_vz': dvl['dvl_vz']
                })
            
            # 查找最近的GPS数据
            while gps_idx < len(gps_data) - 1 and abs(gps_data[gps_idx + 1]['time'] - t) < abs(gps_data[gps_idx]['time'] - t):
                gps_idx += 1
            
            if gps_idx < len(gps_data):
                gps = gps_data[gps_idx]
                row.update(gps)
                del row['time']  # 移除time字段，使用utc_timestamp
            
            merged_data.append(row)
        
        # 转换为DataFrame并保存
        df = pd.DataFrame(merged_data)
        
        # 填充缺失值
        df = df.fillna(method='ffill').fillna(0)
        
        # 保存CSV
        df.to_csv(filename, index=False)
        print(f"示例数据已保存到: {filename}")
        print(f"数据点数量: {len(df)}")
        print(f"时间范围: {self.duration}秒")

def main():
    parser = argparse.ArgumentParser(description='生成水下导航示例数据')
    parser.add_argument('--output', '-o', default='sample_underwater_data.csv',
                       help='输出CSV文件名')
    parser.add_argument('--duration', '-d', type=float, default=300,
                       help='仿真时长 (秒)')
    
    args = parser.parse_args()
    
    # 创建数据生成器
    generator = UnderwaterDataGenerator()
    generator.duration = args.duration
    
    print("生成水下导航示例数据...")
    print(f"仿真时长: {args.duration}秒")
    
    # 生成轨迹
    print("生成参考轨迹...")
    trajectory = generator.generate_trajectory()
    
    # 生成传感器数据
    print("生成IMU数据...")
    imu_data = generator.generate_imu_data(trajectory)
    
    print("生成DVL数据...")
    dvl_data = generator.generate_dvl_data(trajectory)
    
    print("生成GPS参考数据...")
    gps_data = generator.generate_gps_reference(trajectory)
    
    # 合并并保存
    print("合并数据并保存...")
    generator.merge_and_save(imu_data, dvl_data, gps_data, args.output)
    
    print("完成！")

if __name__ == '__main__':
    main()
