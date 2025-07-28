#!/usr/bin/env python3
"""
AUV数据实时可视化脚本
支持多种可视化模式：实时图表、轨迹图、传感器数据图
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TwistStamped
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np
import threading
import time
import argparse

class AUVDataVisualizer(Node):
    def __init__(self, mode='realtime'):
        super().__init__('auv_data_visualizer')
        
        self.mode = mode
        self.max_points = 1000  # 最大显示点数
        
        # 数据缓存
        self.timestamps = deque(maxlen=self.max_points)
        self.imu_data = {
            'acc_x': deque(maxlen=self.max_points),
            'acc_y': deque(maxlen=self.max_points),
            'acc_z': deque(maxlen=self.max_points),
            'gyr_x': deque(maxlen=self.max_points),
            'gyr_y': deque(maxlen=self.max_points),
            'gyr_z': deque(maxlen=self.max_points)
        }
        self.gps_data = {
            'lat': deque(maxlen=self.max_points),
            'lon': deque(maxlen=self.max_points),
            'alt': deque(maxlen=self.max_points)
        }
        self.dvl_data = {
            'vx': deque(maxlen=self.max_points),
            'vy': deque(maxlen=self.max_points),
            'vz': deque(maxlen=self.max_points)
        }
        
        # ROS订阅
        self.imu_subscription = self.create_subscription(
            Imu, '/auv/imu/data', self.imu_callback, 10)
        self.gps_subscription = self.create_subscription(
            NavSatFix, '/auv/gps/fix', self.gps_callback, 10)
        self.dvl_subscription = self.create_subscription(
            TwistStamped, '/auv/dvl/twist', self.dvl_callback, 10)
        
        self.start_time = time.time()
        self.data_count = 0
        
        self.get_logger().info(f'AUV数据可视化器已启动 - 模式: {mode}')
        
        # 启动可视化
        if mode == 'realtime':
            self.setup_realtime_plot()
        elif mode == 'trajectory':
            self.setup_trajectory_plot()
        elif mode == 'sensors':
            self.setup_sensors_plot()
    
    def imu_callback(self, msg):
        """IMU数据回调"""
        current_time = time.time() - self.start_time
        self.timestamps.append(current_time)
        
        self.imu_data['acc_x'].append(msg.linear_acceleration.x)
        self.imu_data['acc_y'].append(msg.linear_acceleration.y)
        self.imu_data['acc_z'].append(msg.linear_acceleration.z)
        self.imu_data['gyr_x'].append(msg.angular_velocity.x)
        self.imu_data['gyr_y'].append(msg.angular_velocity.y)
        self.imu_data['gyr_z'].append(msg.angular_velocity.z)
        
        self.data_count += 1
        if self.data_count % 100 == 0:
            self.get_logger().info(f'已接收 {self.data_count} 个IMU数据点')
    
    def gps_callback(self, msg):
        """GPS数据回调"""
        if msg.latitude != 0 and msg.longitude != 0:
            self.gps_data['lat'].append(msg.latitude)
            self.gps_data['lon'].append(msg.longitude)
            self.gps_data['alt'].append(msg.altitude)
            self.get_logger().info(f'GPS数据: {msg.latitude:.6f}, {msg.longitude:.6f}')
    
    def dvl_callback(self, msg):
        """DVL数据回调"""
        self.dvl_data['vx'].append(msg.twist.linear.x)
        self.dvl_data['vy'].append(msg.twist.linear.y)
        self.dvl_data['vz'].append(msg.twist.linear.z)
    
    def setup_realtime_plot(self):
        """设置实时图表"""
        self.fig, self.axes = plt.subplots(2, 2, figsize=(12, 8))
        self.fig.suptitle('AUV传感器数据实时监控', fontsize=16)
        
        # 加速度图
        self.axes[0, 0].set_title('IMU加速度 (m/s²)')
        self.axes[0, 0].set_xlabel('时间 (s)')
        self.axes[0, 0].set_ylabel('加速度')
        self.axes[0, 0].grid(True)
        
        # 角速度图
        self.axes[0, 1].set_title('IMU角速度 (rad/s)')
        self.axes[0, 1].set_xlabel('时间 (s)')
        self.axes[0, 1].set_ylabel('角速度')
        self.axes[0, 1].grid(True)
        
        # DVL速度图
        self.axes[1, 0].set_title('DVL速度 (m/s)')
        self.axes[1, 0].set_xlabel('时间 (s)')
        self.axes[1, 0].set_ylabel('速度')
        self.axes[1, 0].grid(True)
        
        # 数据统计图
        self.axes[1, 1].set_title('数据接收统计')
        self.axes[1, 1].set_xlabel('时间 (s)')
        self.axes[1, 1].set_ylabel('数据点数')
        self.axes[1, 1].grid(True)
        
        # 启动动画
        self.ani = animation.FuncAnimation(
            self.fig, self.update_realtime_plot, interval=100, blit=False)
        
        plt.tight_layout()
        plt.show()
    
    def update_realtime_plot(self, frame):
        """更新实时图表"""
        if len(self.timestamps) == 0:
            return
        
        times = list(self.timestamps)
        
        # 清除所有子图
        for ax in self.axes.flat:
            ax.clear()
        
        # 加速度图
        if len(times) > 0:
            self.axes[0, 0].plot(times, list(self.imu_data['acc_x']), 'r-', label='X', alpha=0.7)
            self.axes[0, 0].plot(times, list(self.imu_data['acc_y']), 'g-', label='Y', alpha=0.7)
            self.axes[0, 0].plot(times, list(self.imu_data['acc_z']), 'b-', label='Z', alpha=0.7)
            self.axes[0, 0].set_title('IMU加速度 (m/s²)')
            self.axes[0, 0].set_xlabel('时间 (s)')
            self.axes[0, 0].set_ylabel('加速度')
            self.axes[0, 0].legend()
            self.axes[0, 0].grid(True)
        
        # 角速度图
        if len(times) > 0:
            self.axes[0, 1].plot(times, list(self.imu_data['gyr_x']), 'r-', label='X', alpha=0.7)
            self.axes[0, 1].plot(times, list(self.imu_data['gyr_y']), 'g-', label='Y', alpha=0.7)
            self.axes[0, 1].plot(times, list(self.imu_data['gyr_z']), 'b-', label='Z', alpha=0.7)
            self.axes[0, 1].set_title('IMU角速度 (rad/s)')
            self.axes[0, 1].set_xlabel('时间 (s)')
            self.axes[0, 1].set_ylabel('角速度')
            self.axes[0, 1].legend()
            self.axes[0, 1].grid(True)
        
        # DVL速度图
        if len(times) > 0 and len(self.dvl_data['vx']) > 0:
            dvl_times = times[-len(self.dvl_data['vx']):]
            self.axes[1, 0].plot(dvl_times, list(self.dvl_data['vx']), 'r-', label='Vx', alpha=0.7)
            self.axes[1, 0].plot(dvl_times, list(self.dvl_data['vy']), 'g-', label='Vy', alpha=0.7)
            self.axes[1, 0].plot(dvl_times, list(self.dvl_data['vz']), 'b-', label='Vz', alpha=0.7)
            self.axes[1, 0].set_title('DVL速度 (m/s)')
            self.axes[1, 0].set_xlabel('时间 (s)')
            self.axes[1, 0].set_ylabel('速度')
            self.axes[1, 0].legend()
            self.axes[1, 0].grid(True)
        
        # 数据统计
        self.axes[1, 1].text(0.1, 0.8, f'IMU数据点: {len(self.timestamps)}', transform=self.axes[1, 1].transAxes)
        self.axes[1, 1].text(0.1, 0.6, f'GPS数据点: {len(self.gps_data["lat"])}', transform=self.axes[1, 1].transAxes)
        self.axes[1, 1].text(0.1, 0.4, f'DVL数据点: {len(self.dvl_data["vx"])}', transform=self.axes[1, 1].transAxes)
        self.axes[1, 1].text(0.1, 0.2, f'运行时间: {times[-1]:.1f}s' if times else '运行时间: 0s', transform=self.axes[1, 1].transAxes)
        self.axes[1, 1].set_title('数据接收统计')
        self.axes[1, 1].set_xlim(0, 1)
        self.axes[1, 1].set_ylim(0, 1)
    
    def setup_trajectory_plot(self):
        """设置轨迹图"""
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title('AUV轨迹图')
        self.ax.set_xlabel('经度')
        self.ax.set_ylabel('纬度')
        self.ax.grid(True)
        
        # 启动动画
        self.ani = animation.FuncAnimation(
            self.fig, self.update_trajectory_plot, interval=500, blit=False)
        
        plt.show()
    
    def update_trajectory_plot(self, frame):
        """更新轨迹图"""
        self.ax.clear()
        
        if len(self.gps_data['lat']) > 1:
            lats = list(self.gps_data['lat'])
            lons = list(self.gps_data['lon'])
            
            self.ax.plot(lons, lats, 'b-', alpha=0.7, linewidth=2, label='轨迹')
            self.ax.scatter(lons[0], lats[0], c='green', s=100, marker='o', label='起点')
            self.ax.scatter(lons[-1], lats[-1], c='red', s=100, marker='x', label='当前位置')
            
            self.ax.set_title(f'AUV轨迹图 ({len(lats)} GPS点)')
            self.ax.set_xlabel('经度')
            self.ax.set_ylabel('纬度')
            self.ax.legend()
            self.ax.grid(True)
        else:
            self.ax.text(0.5, 0.5, '等待GPS数据...', ha='center', va='center', transform=self.ax.transAxes)
            self.ax.set_title('AUV轨迹图 (等待GPS数据)')
    
    def setup_sensors_plot(self):
        """设置传感器详细图"""
        self.fig, self.axes = plt.subplots(3, 1, figsize=(12, 10))
        self.fig.suptitle('AUV传感器详细数据', fontsize=16)
        
        # 启动动画
        self.ani = animation.FuncAnimation(
            self.fig, self.update_sensors_plot, interval=200, blit=False)
        
        plt.tight_layout()
        plt.show()
    
    def update_sensors_plot(self, frame):
        """更新传感器详细图"""
        if len(self.timestamps) == 0:
            return
        
        times = list(self.timestamps)
        
        # 清除所有子图
        for ax in self.axes:
            ax.clear()
        
        # IMU加速度
        self.axes[0].plot(times, list(self.imu_data['acc_x']), 'r-', label='Acc X')
        self.axes[0].plot(times, list(self.imu_data['acc_y']), 'g-', label='Acc Y')
        self.axes[0].plot(times, list(self.imu_data['acc_z']), 'b-', label='Acc Z')
        self.axes[0].set_title('IMU加速度')
        self.axes[0].set_ylabel('m/s²')
        self.axes[0].legend()
        self.axes[0].grid(True)
        
        # IMU角速度
        self.axes[1].plot(times, list(self.imu_data['gyr_x']), 'r-', label='Gyr X')
        self.axes[1].plot(times, list(self.imu_data['gyr_y']), 'g-', label='Gyr Y')
        self.axes[1].plot(times, list(self.imu_data['gyr_z']), 'b-', label='Gyr Z')
        self.axes[1].set_title('IMU角速度')
        self.axes[1].set_ylabel('rad/s')
        self.axes[1].legend()
        self.axes[1].grid(True)
        
        # DVL速度
        if len(self.dvl_data['vx']) > 0:
            dvl_times = times[-len(self.dvl_data['vx']):]
            self.axes[2].plot(dvl_times, list(self.dvl_data['vx']), 'r-', label='Vel X')
            self.axes[2].plot(dvl_times, list(self.dvl_data['vy']), 'g-', label='Vel Y')
            self.axes[2].plot(dvl_times, list(self.dvl_data['vz']), 'b-', label='Vel Z')
        self.axes[2].set_title('DVL速度')
        self.axes[2].set_xlabel('时间 (s)')
        self.axes[2].set_ylabel('m/s')
        self.axes[2].legend()
        self.axes[2].grid(True)

def main():
    parser = argparse.ArgumentParser(description='AUV数据可视化器')
    parser.add_argument('--mode', choices=['realtime', 'trajectory', 'sensors'], 
                       default='realtime', help='可视化模式')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        visualizer = AUVDataVisualizer(args.mode)
        
        # 在单独线程中运行ROS
        ros_thread = threading.Thread(target=lambda: rclpy.spin(visualizer))
        ros_thread.daemon = True
        ros_thread.start()
        
        # 主线程运行matplotlib
        plt.show()
        
    except KeyboardInterrupt:
        print('\n可视化器被用户中断')
    finally:
        if 'visualizer' in locals():
            visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
