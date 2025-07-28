#!/usr/bin/env python3
"""
AUV数据回放脚本
用于回放您采集的真实DVL、IMU、GPS数据到ROS话题中
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TwistStamped, Vector3Stamped
from std_msgs.msg import Header
import pandas as pd
import numpy as np
from datetime import datetime
import argparse
import os
import time

class AUVDataReplayNode(Node):
    def __init__(self, csv_file, replay_rate=1.0, loop=False):
        super().__init__('auv_data_replay_node')
        
        self.csv_file = csv_file
        self.replay_rate = replay_rate
        self.loop = loop
        
        # 创建发布者
        self.imu_publisher = self.create_publisher(Imu, '/auv/imu/data', 10)
        self.gps_publisher = self.create_publisher(NavSatFix, '/auv/gps/fix', 10)
        self.dvl_publisher = self.create_publisher(TwistStamped, '/auv/dvl/twist', 10)
        
        # 加载数据
        self.load_data()
        
        # 创建定时器
        self.timer_period = 0.1 / self.replay_rate  # 10Hz基础频率
        self.timer = self.create_timer(self.timer_period, self.replay_callback)
        
        self.current_index = 0
        self.start_time = time.time()
        
        self.get_logger().info(f'数据回放节点已启动')
        self.get_logger().info(f'数据文件: {csv_file}')
        self.get_logger().info(f'数据点数: {len(self.data)}')
        self.get_logger().info(f'回放速率: {replay_rate}x')
        self.get_logger().info(f'循环播放: {loop}')
    
    def load_data(self):
        """加载CSV数据"""
        try:
            self.data = pd.read_csv(self.csv_file)
            self.get_logger().info(f'成功加载 {len(self.data)} 行数据')
            
            # 显示数据列信息
            self.get_logger().info(f'数据列: {list(self.data.columns)}')
            
            # 检查必要的列是否存在
            required_columns = ['acc_x', 'acc_y', 'acc_z', 'gyr_x', 'gyr_y', 'gyr_z']
            missing_columns = [col for col in required_columns if col not in self.data.columns]
            if missing_columns:
                self.get_logger().warn(f'缺少列: {missing_columns}')
            
        except Exception as e:
            self.get_logger().error(f'加载数据失败: {e}')
            raise
    
    def replay_callback(self):
        """回放回调函数"""
        if self.current_index >= len(self.data):
            if self.loop:
                self.current_index = 0
                self.get_logger().info('数据回放循环重启')
            else:
                self.get_logger().info('数据回放完成')
                self.timer.cancel()
                return
        
        # 获取当前数据行
        row = self.data.iloc[self.current_index]
        
        # 创建时间戳
        current_time = self.get_clock().now().to_msg()
        
        # 发布IMU数据
        self.publish_imu_data(row, current_time)
        
        # 发布GPS数据（如果有效）
        if 'gps_lon' in self.data.columns and 'gps_lat' in self.data.columns:
            if row['gps_lon'] != 0 and row['gps_lat'] != 0:
                self.publish_gps_data(row, current_time)
        
        # 发布DVL数据
        if 'dvl_vx' in self.data.columns:
            self.publish_dvl_data(row, current_time)
        
        self.current_index += 1
        
        # 每1000个数据点打印一次进度
        if self.current_index % 1000 == 0:
            progress = (self.current_index / len(self.data)) * 100
            self.get_logger().info(f'回放进度: {progress:.1f}% ({self.current_index}/{len(self.data)})')
    
    def publish_imu_data(self, row, timestamp):
        """发布IMU数据"""
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = timestamp
        imu_msg.header.frame_id = 'imu_link'
        
        # 线性加速度 (m/s²)
        if all(col in row for col in ['acc_x', 'acc_y', 'acc_z']):
            imu_msg.linear_acceleration.x = float(row['acc_x'])
            imu_msg.linear_acceleration.y = float(row['acc_y'])
            imu_msg.linear_acceleration.z = float(row['acc_z'])
        
        # 角速度 (rad/s)
        if all(col in row for col in ['gyr_x', 'gyr_y', 'gyr_z']):
            imu_msg.angular_velocity.x = float(row['gyr_x'])
            imu_msg.angular_velocity.y = float(row['gyr_y'])
            imu_msg.angular_velocity.z = float(row['gyr_z'])
        
        # 方向（如果有四元数数据）
        # 这里可以根据您的数据格式添加四元数或欧拉角转换
        imu_msg.orientation.w = 1.0  # 默认值
        
        # 协方差矩阵（可以根据传感器规格设置）
        imu_msg.linear_acceleration_covariance[0] = 0.01
        imu_msg.linear_acceleration_covariance[4] = 0.01
        imu_msg.linear_acceleration_covariance[8] = 0.01
        
        imu_msg.angular_velocity_covariance[0] = 0.001
        imu_msg.angular_velocity_covariance[4] = 0.001
        imu_msg.angular_velocity_covariance[8] = 0.001
        
        self.imu_publisher.publish(imu_msg)
    
    def publish_gps_data(self, row, timestamp):
        """发布GPS数据"""
        gps_msg = NavSatFix()
        gps_msg.header = Header()
        gps_msg.header.stamp = timestamp
        gps_msg.header.frame_id = 'gps_link'
        
        # GPS坐标
        gps_msg.latitude = float(row['gps_lat'])
        gps_msg.longitude = float(row['gps_lon'])
        
        if 'gps_alt' in row:
            gps_msg.altitude = float(row['gps_alt'])
        
        # 定位状态
        gps_msg.status.status = 0  # STATUS_FIX
        gps_msg.status.service = 1  # SERVICE_GPS
        
        # 位置协方差
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        gps_msg.position_covariance[0] = 1.0  # 经度方差
        gps_msg.position_covariance[4] = 1.0  # 纬度方差
        gps_msg.position_covariance[8] = 4.0  # 高度方差
        
        self.gps_publisher.publish(gps_msg)
    
    def publish_dvl_data(self, row, timestamp):
        """发布DVL数据"""
        dvl_msg = TwistStamped()
        dvl_msg.header = Header()
        dvl_msg.header.stamp = timestamp
        dvl_msg.header.frame_id = 'dvl_link'
        
        # DVL速度
        if all(col in row for col in ['dvl_vx', 'dvl_vy', 'dvl_vz']):
            dvl_msg.twist.linear.x = float(row['dvl_vx'])
            dvl_msg.twist.linear.y = float(row['dvl_vy'])
            dvl_msg.twist.linear.z = float(row['dvl_vz'])
        
        self.dvl_publisher.publish(dvl_msg)

def main():
    parser = argparse.ArgumentParser(description='AUV数据回放节点')
    parser.add_argument('csv_file', help='CSV数据文件路径')
    parser.add_argument('--rate', type=float, default=1.0, help='回放速率倍数 (默认: 1.0)')
    parser.add_argument('--loop', action='store_true', help='循环播放')
    
    args = parser.parse_args()
    
    # 检查文件是否存在
    if not os.path.exists(args.csv_file):
        print(f'错误: 文件不存在 {args.csv_file}')
        return
    
    # 初始化ROS
    rclpy.init()
    
    try:
        # 创建节点
        node = AUVDataReplayNode(args.csv_file, args.rate, args.loop)
        
        # 运行节点
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print('\n数据回放被用户中断')
    except Exception as e:
        print(f'错误: {e}')
    finally:
        # 清理
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
