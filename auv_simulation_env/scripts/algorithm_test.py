#!/usr/bin/env python3
"""
AUV算法测试脚本
用于测试您的自定义算法并与Ground Truth进行对比
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import pandas as pd
from datetime import datetime
import json
import os
import time
from collections import deque

class AlgorithmTestNode(Node):
    def __init__(self, algorithm_name="custom_algorithm"):
        super().__init__('algorithm_test_node')
        
        self.algorithm_name = algorithm_name
        
        # 数据缓存
        self.imu_buffer = deque(maxlen=100)
        self.gps_buffer = deque(maxlen=100)
        self.dvl_buffer = deque(maxlen=100)
        self.ground_truth_buffer = deque(maxlen=100)
        
        # 算法输出缓存
        self.algorithm_output = deque(maxlen=1000)
        
        # 订阅传感器数据
        self.imu_subscription = self.create_subscription(
            Imu, '/auv/imu/data', self.imu_callback, 10)
        self.gps_subscription = self.create_subscription(
            NavSatFix, '/auv/gps/fix', self.gps_callback, 10)
        self.dvl_subscription = self.create_subscription(
            TwistStamped, '/auv/dvl/twist', self.dvl_callback, 10)
        
        # 订阅Ground Truth（来自Gazebo仿真）
        self.ground_truth_subscription = self.create_subscription(
            Odometry, '/gazebo/model_states', self.ground_truth_callback, 10)
        
        # 发布算法结果
        self.algorithm_pose_publisher = self.create_publisher(
            PoseStamped, '/auv/algorithm/pose', 10)
        self.algorithm_odom_publisher = self.create_publisher(
            Odometry, '/auv/algorithm/odometry', 10)
        
        # 创建定时器进行算法计算
        self.timer = self.create_timer(0.1, self.algorithm_callback)  # 10Hz
        
        # 性能统计
        self.start_time = time.time()
        self.algorithm_call_count = 0
        self.total_computation_time = 0.0
        
        # 误差统计
        self.position_errors = []
        self.velocity_errors = []
        
        self.get_logger().info(f'算法测试节点已启动: {algorithm_name}')
        self.get_logger().info('等待传感器数据...')
    
    def imu_callback(self, msg):
        """IMU数据回调"""
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        imu_data = {
            'timestamp': timestamp,
            'acc_x': msg.linear_acceleration.x,
            'acc_y': msg.linear_acceleration.y,
            'acc_z': msg.linear_acceleration.z,
            'gyr_x': msg.angular_velocity.x,
            'gyr_y': msg.angular_velocity.y,
            'gyr_z': msg.angular_velocity.z
        }
        self.imu_buffer.append(imu_data)
    
    def gps_callback(self, msg):
        """GPS数据回调"""
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        gps_data = {
            'timestamp': timestamp,
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude
        }
        self.gps_buffer.append(gps_data)
    
    def dvl_callback(self, msg):
        """DVL数据回调"""
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dvl_data = {
            'timestamp': timestamp,
            'vx': msg.twist.linear.x,
            'vy': msg.twist.linear.y,
            'vz': msg.twist.linear.z
        }
        self.dvl_buffer.append(dvl_data)
    
    def ground_truth_callback(self, msg):
        """Ground Truth数据回调（来自Gazebo）"""
        # 这里需要根据实际的Gazebo话题格式进行调整
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # 假设我们关心的是第一个模型（AUV）
        if len(msg.pose) > 0:
            pose = msg.pose[0]
            twist = msg.twist[0] if len(msg.twist) > 0 else None
            
            ground_truth_data = {
                'timestamp': timestamp,
                'position': {
                    'x': pose.position.x,
                    'y': pose.position.y,
                    'z': pose.position.z
                },
                'orientation': {
                    'x': pose.orientation.x,
                    'y': pose.orientation.y,
                    'z': pose.orientation.z,
                    'w': pose.orientation.w
                }
            }
            
            if twist:
                ground_truth_data['velocity'] = {
                    'x': twist.linear.x,
                    'y': twist.linear.y,
                    'z': twist.linear.z
                }
            
            self.ground_truth_buffer.append(ground_truth_data)
    
    def algorithm_callback(self):
        """算法计算回调"""
        # 检查是否有足够的数据
        if len(self.imu_buffer) < 10 or len(self.dvl_buffer) < 5:
            return
        
        start_time = time.time()
        
        # 运行您的算法
        result = self.run_custom_algorithm()
        
        end_time = time.time()
        computation_time = end_time - start_time
        
        # 更新性能统计
        self.algorithm_call_count += 1
        self.total_computation_time += computation_time
        
        if result is not None:
            # 发布算法结果
            self.publish_algorithm_result(result)
            
            # 与Ground Truth进行对比
            self.compare_with_ground_truth(result)
            
            # 保存结果
            self.algorithm_output.append({
                'timestamp': time.time(),
                'result': result,
                'computation_time': computation_time
            })
        
        # 每100次调用打印一次性能统计
        if self.algorithm_call_count % 100 == 0:
            avg_time = self.total_computation_time / self.algorithm_call_count
            self.get_logger().info(
                f'算法性能: 平均计算时间 {avg_time*1000:.2f}ms, '
                f'频率 {1.0/avg_time:.1f}Hz'
            )
    
    def run_custom_algorithm(self):
        """
        运行您的自定义算法
        这里是一个示例实现，您需要替换为您的实际算法
        """
        # 获取最新的传感器数据
        latest_imu = list(self.imu_buffer)[-10:]  # 最近10个IMU数据
        latest_dvl = list(self.dvl_buffer)[-5:]   # 最近5个DVL数据
        latest_gps = list(self.gps_buffer)[-1:] if self.gps_buffer else []
        
        # 示例算法：简单的积分定位
        if len(latest_dvl) >= 2:
            # 使用DVL速度进行积分
            dt = latest_dvl[-1]['timestamp'] - latest_dvl[-2]['timestamp']
            
            # 简单的位置估计（这里需要替换为您的实际算法）
            estimated_position = {
                'x': latest_dvl[-1]['vx'] * dt,
                'y': latest_dvl[-1]['vy'] * dt,
                'z': latest_dvl[-1]['vz'] * dt
            }
            
            # 简单的速度估计
            estimated_velocity = {
                'x': latest_dvl[-1]['vx'],
                'y': latest_dvl[-1]['vy'],
                'z': latest_dvl[-1]['vz']
            }
            
            return {
                'position': estimated_position,
                'velocity': estimated_velocity,
                'timestamp': latest_dvl[-1]['timestamp']
            }
        
        return None
    
    def publish_algorithm_result(self, result):
        """发布算法结果"""
        current_time = self.get_clock().now().to_msg()
        
        # 发布位姿
        pose_msg = PoseStamped()
        pose_msg.header.stamp = current_time
        pose_msg.header.frame_id = 'map'
        
        pose_msg.pose.position.x = result['position']['x']
        pose_msg.pose.position.y = result['position']['y']
        pose_msg.pose.position.z = result['position']['z']
        
        # 默认方向
        pose_msg.pose.orientation.w = 1.0
        
        self.algorithm_pose_publisher.publish(pose_msg)
        
        # 发布里程计
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'
        
        odom_msg.pose.pose = pose_msg.pose
        odom_msg.twist.twist.linear.x = result['velocity']['x']
        odom_msg.twist.twist.linear.y = result['velocity']['y']
        odom_msg.twist.twist.linear.z = result['velocity']['z']
        
        self.algorithm_odom_publisher.publish(odom_msg)
    
    def compare_with_ground_truth(self, result):
        """与Ground Truth进行对比"""
        if not self.ground_truth_buffer:
            return
        
        # 找到时间最接近的Ground Truth
        result_time = result['timestamp']
        closest_gt = min(self.ground_truth_buffer, 
                        key=lambda x: abs(x['timestamp'] - result_time))
        
        # 计算位置误差
        pos_error = np.sqrt(
            (result['position']['x'] - closest_gt['position']['x'])**2 +
            (result['position']['y'] - closest_gt['position']['y'])**2 +
            (result['position']['z'] - closest_gt['position']['z'])**2
        )
        
        self.position_errors.append(pos_error)
        
        # 计算速度误差（如果Ground Truth有速度信息）
        if 'velocity' in closest_gt:
            vel_error = np.sqrt(
                (result['velocity']['x'] - closest_gt['velocity']['x'])**2 +
                (result['velocity']['y'] - closest_gt['velocity']['y'])**2 +
                (result['velocity']['z'] - closest_gt['velocity']['z'])**2
            )
            self.velocity_errors.append(vel_error)
        
        # 每50次对比打印一次统计
        if len(self.position_errors) % 50 == 0:
            avg_pos_error = np.mean(self.position_errors[-50:])
            self.get_logger().info(f'平均位置误差: {avg_pos_error:.3f}m')
    
    def save_results(self, filename=None):
        """保存测试结果"""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"algorithm_test_results_{timestamp}.json"
        
        results = {
            'algorithm_name': self.algorithm_name,
            'test_duration': time.time() - self.start_time,
            'algorithm_calls': self.algorithm_call_count,
            'average_computation_time': self.total_computation_time / max(1, self.algorithm_call_count),
            'position_errors': {
                'mean': float(np.mean(self.position_errors)) if self.position_errors else 0,
                'std': float(np.std(self.position_errors)) if self.position_errors else 0,
                'max': float(np.max(self.position_errors)) if self.position_errors else 0,
                'min': float(np.min(self.position_errors)) if self.position_errors else 0
            },
            'velocity_errors': {
                'mean': float(np.mean(self.velocity_errors)) if self.velocity_errors else 0,
                'std': float(np.std(self.velocity_errors)) if self.velocity_errors else 0,
                'max': float(np.max(self.velocity_errors)) if self.velocity_errors else 0,
                'min': float(np.min(self.velocity_errors)) if self.velocity_errors else 0
            }
        }
        
        with open(filename, 'w') as f:
            json.dump(results, f, indent=2)
        
        self.get_logger().info(f'测试结果已保存到: {filename}')
        return filename

def main():
    rclpy.init()
    
    try:
        node = AlgorithmTestNode("LSTM_DVL_Algorithm")
        
        def signal_handler():
            node.get_logger().info('保存测试结果...')
            node.save_results()
        
        # 注册信号处理
        import signal
        signal.signal(signal.SIGINT, lambda s, f: signal_handler())
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print('\n算法测试被用户中断')
    finally:
        if 'node' in locals():
            node.save_results()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
