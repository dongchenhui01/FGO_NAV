#!/usr/bin/env python3
"""
AUV数据静态分析和可视化脚本
分析CSV数据文件，生成各种图表和统计信息
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse
import os
from datetime import datetime

class AUVDataAnalyzer:
    def __init__(self, csv_file):
        self.csv_file = csv_file
        self.data = None
        self.load_data()
    
    def load_data(self):
        """加载CSV数据"""
        try:
            self.data = pd.read_csv(self.csv_file)
            print(f"成功加载数据: {len(self.data)} 行 × {len(self.data.columns)} 列")
            print(f"数据列: {list(self.data.columns)}")
            
            # 数据基本信息
            print(f"\n数据时间范围:")
            if 'utc_timestamp' in self.data.columns:
                print(f"  起始时间: {self.data['utc_timestamp'].iloc[0]}")
                print(f"  结束时间: {self.data['utc_timestamp'].iloc[-1]}")
                duration = len(self.data) * 0.1  # 假设10Hz采样
                print(f"  估计持续时间: {duration:.1f} 秒")
            
        except Exception as e:
            print(f"加载数据失败: {e}")
            raise
    
    def analyze_imu_data(self):
        """分析IMU数据"""
        print("\n=== IMU数据分析 ===")
        
        imu_cols = ['acc_x', 'acc_y', 'acc_z', 'gyr_x', 'gyr_y', 'gyr_z']
        available_cols = [col for col in imu_cols if col in self.data.columns]
        
        if not available_cols:
            print("未找到IMU数据列")
            return
        
        # 统计信息
        print("IMU数据统计:")
        for col in available_cols:
            data_col = self.data[col]
            print(f"  {col}: 均值={data_col.mean():.4f}, 标准差={data_col.std():.4f}, "
                  f"最小值={data_col.min():.4f}, 最大值={data_col.max():.4f}")
        
        # 绘制IMU数据图
        fig, axes = plt.subplots(2, 1, figsize=(12, 8))
        
        # 加速度
        acc_cols = [col for col in ['acc_x', 'acc_y', 'acc_z'] if col in self.data.columns]
        if acc_cols:
            for col in acc_cols:
                axes[0].plot(self.data.index, self.data[col], label=col, alpha=0.7)
            axes[0].set_title('IMU加速度数据')
            axes[0].set_ylabel('加速度 (m/s²)')
            axes[0].legend()
            axes[0].grid(True)
        
        # 角速度
        gyr_cols = [col for col in ['gyr_x', 'gyr_y', 'gyr_z'] if col in self.data.columns]
        if gyr_cols:
            for col in gyr_cols:
                axes[1].plot(self.data.index, self.data[col], label=col, alpha=0.7)
            axes[1].set_title('IMU角速度数据')
            axes[1].set_xlabel('数据点')
            axes[1].set_ylabel('角速度 (rad/s)')
            axes[1].legend()
            axes[1].grid(True)
        
        plt.tight_layout()
        plt.savefig('results/imu_analysis.png', dpi=300, bbox_inches='tight')
        print("IMU分析图已保存: results/imu_analysis.png")
        plt.show()
    
    def analyze_gps_data(self):
        """分析GPS数据"""
        print("\n=== GPS数据分析 ===")
        
        gps_cols = ['gps_lat', 'gps_lon', 'gps_alt']
        available_cols = [col for col in gps_cols if col in self.data.columns]
        
        if not available_cols:
            print("未找到GPS数据列")
            return
        
        # 找到有效GPS数据
        valid_gps = self.data[(self.data['gps_lat'] != 0) & (self.data['gps_lon'] != 0)]
        
        if len(valid_gps) == 0:
            print("未找到有效GPS数据（非零值）")
            return
        
        print(f"有效GPS数据点: {len(valid_gps)} / {len(self.data)} ({len(valid_gps)/len(self.data)*100:.1f}%)")
        print(f"GPS数据范围:")
        print(f"  纬度: {valid_gps['gps_lat'].min():.6f} ~ {valid_gps['gps_lat'].max():.6f}")
        print(f"  经度: {valid_gps['gps_lon'].min():.6f} ~ {valid_gps['gps_lon'].max():.6f}")
        if 'gps_alt' in valid_gps.columns:
            print(f"  高度: {valid_gps['gps_alt'].min():.2f} ~ {valid_gps['gps_alt'].max():.2f} m")
        
        # 绘制GPS轨迹
        fig, axes = plt.subplots(1, 2, figsize=(15, 6))
        
        # 轨迹图
        axes[0].plot(valid_gps['gps_lon'], valid_gps['gps_lat'], 'b-', alpha=0.7, linewidth=2)
        axes[0].scatter(valid_gps['gps_lon'].iloc[0], valid_gps['gps_lat'].iloc[0], 
                       c='green', s=100, marker='o', label='起点')
        axes[0].scatter(valid_gps['gps_lon'].iloc[-1], valid_gps['gps_lat'].iloc[-1], 
                       c='red', s=100, marker='x', label='终点')
        axes[0].set_title('GPS轨迹图')
        axes[0].set_xlabel('经度')
        axes[0].set_ylabel('纬度')
        axes[0].legend()
        axes[0].grid(True)
        
        # 高度变化
        if 'gps_alt' in valid_gps.columns:
            axes[1].plot(range(len(valid_gps)), valid_gps['gps_alt'], 'g-', linewidth=2)
            axes[1].set_title('GPS高度变化')
            axes[1].set_xlabel('GPS数据点')
            axes[1].set_ylabel('高度 (m)')
            axes[1].grid(True)
        
        plt.tight_layout()
        plt.savefig('results/gps_analysis.png', dpi=300, bbox_inches='tight')
        print("GPS分析图已保存: results/gps_analysis.png")
        plt.show()
    
    def analyze_dvl_data(self):
        """分析DVL数据"""
        print("\n=== DVL数据分析 ===")
        
        dvl_cols = ['dvl_vx', 'dvl_vy', 'dvl_vz']
        available_cols = [col for col in dvl_cols if col in self.data.columns]
        
        if not available_cols:
            print("未找到DVL数据列")
            return
        
        # 统计信息
        print("DVL速度统计:")
        for col in available_cols:
            data_col = self.data[col]
            print(f"  {col}: 均值={data_col.mean():.4f}, 标准差={data_col.std():.4f}, "
                  f"最小值={data_col.min():.4f}, 最大值={data_col.max():.4f}")
        
        # 计算速度大小
        if all(col in self.data.columns for col in dvl_cols):
            speed = np.sqrt(self.data['dvl_vx']**2 + self.data['dvl_vy']**2 + self.data['dvl_vz']**2)
            print(f"  速度大小: 均值={speed.mean():.4f}, 最大值={speed.max():.4f} m/s")
        
        # 绘制DVL数据图
        fig, axes = plt.subplots(2, 1, figsize=(12, 8))
        
        # 各方向速度
        for col in available_cols:
            axes[0].plot(self.data.index, self.data[col], label=col, alpha=0.7)
        axes[0].set_title('DVL速度数据')
        axes[0].set_ylabel('速度 (m/s)')
        axes[0].legend()
        axes[0].grid(True)
        
        # 速度大小
        if all(col in self.data.columns for col in dvl_cols):
            axes[1].plot(self.data.index, speed, 'r-', linewidth=2, label='速度大小')
            axes[1].set_title('DVL速度大小')
            axes[1].set_xlabel('数据点')
            axes[1].set_ylabel('速度 (m/s)')
            axes[1].legend()
            axes[1].grid(True)
        
        plt.tight_layout()
        plt.savefig('results/dvl_analysis.png', dpi=300, bbox_inches='tight')
        print("DVL分析图已保存: results/dvl_analysis.png")
        plt.show()
    
    def generate_summary_report(self):
        """生成数据摘要报告"""
        print("\n=== 数据摘要报告 ===")
        
        report = {
            "数据文件": self.csv_file,
            "分析时间": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "数据规模": {
                "总行数": len(self.data),
                "总列数": len(self.data.columns),
                "数据点总数": len(self.data) * len(self.data.columns)
            },
            "传感器数据": {}
        }
        
        # IMU数据统计
        imu_cols = ['acc_x', 'acc_y', 'acc_z', 'gyr_x', 'gyr_y', 'gyr_z']
        imu_available = [col for col in imu_cols if col in self.data.columns]
        if imu_available:
            report["传感器数据"]["IMU"] = {
                "可用列": imu_available,
                "数据点数": len(self.data)
            }
        
        # GPS数据统计
        if 'gps_lat' in self.data.columns and 'gps_lon' in self.data.columns:
            valid_gps = len(self.data[(self.data['gps_lat'] != 0) & (self.data['gps_lon'] != 0)])
            report["传感器数据"]["GPS"] = {
                "总数据点": len(self.data),
                "有效数据点": valid_gps,
                "有效率": f"{valid_gps/len(self.data)*100:.1f}%"
            }
        
        # DVL数据统计
        dvl_cols = ['dvl_vx', 'dvl_vy', 'dvl_vz']
        dvl_available = [col for col in dvl_cols if col in self.data.columns]
        if dvl_available:
            report["传感器数据"]["DVL"] = {
                "可用列": dvl_available,
                "数据点数": len(self.data)
            }
        
        # 保存报告
        import json
        with open('results/data_analysis_report.json', 'w', encoding='utf-8') as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        
        print("数据摘要报告已保存: results/data_analysis_report.json")
        
        # 打印摘要
        print(f"\n数据规模: {report['数据规模']['总行数']} 行 × {report['数据规模']['总列数']} 列")
        for sensor, info in report["传感器数据"].items():
            print(f"{sensor}传感器: {info}")

def main():
    parser = argparse.ArgumentParser(description='AUV数据分析器')
    parser.add_argument('csv_file', help='CSV数据文件路径')
    parser.add_argument('--imu', action='store_true', help='分析IMU数据')
    parser.add_argument('--gps', action='store_true', help='分析GPS数据')
    parser.add_argument('--dvl', action='store_true', help='分析DVL数据')
    parser.add_argument('--all', action='store_true', help='分析所有数据')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.csv_file):
        print(f"错误: 文件不存在 {args.csv_file}")
        return
    
    # 创建结果目录
    os.makedirs('results', exist_ok=True)
    
    # 创建分析器
    analyzer = AUVDataAnalyzer(args.csv_file)
    
    # 执行分析
    if args.all or args.imu:
        analyzer.analyze_imu_data()
    
    if args.all or args.gps:
        analyzer.analyze_gps_data()
    
    if args.all or args.dvl:
        analyzer.analyze_dvl_data()
    
    if args.all or (not args.imu and not args.gps and not args.dvl):
        analyzer.generate_summary_report()

if __name__ == '__main__':
    main()
