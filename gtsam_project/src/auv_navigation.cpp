/**
 * AUV组合导航算法 - 基于GTSAM的因子图实现
 *
 * 因子图架构:
 * - IMU预积分因子: 主要约束，连接相邻状态节点
 * - DVL速度因子: 提供速度约束，防止速度漂移
 * - 偏置因子: IMU偏置随机游走模型
 * - GPS: 仅用于初始化和参考轨迹对比
 *
 * 使用ISAM2作为后端优化器，MTI-7 IMU参数
 *
 * 作者: dongchenhui
 * 日期: 2025-07-21
 */

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iomanip>
#include <cmath>
#include <sstream>
#include <chrono>

// GTSAM相关头文件
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/NavState.h>

// 用于可视化
#include <fstream>

using namespace std;
using namespace gtsam;

// 使用gtsam::Symbol简化变量命名
using symbol_shorthand::X; // 位姿
using symbol_shorthand::V; // 速度
using symbol_shorthand::B; // IMU偏置

// 解析ISO时间戳为秒数
double parseTimestamp(const string& timestamp_str) {
    // 简化处理：提取秒数部分
    // 格式: 2025-07-12T02:26:09.369439+00:00
    size_t t_pos = timestamp_str.find('T');
    size_t plus_pos = timestamp_str.find('+');

    if (t_pos == string::npos || plus_pos == string::npos) {
        return 0.0;
    }

    string time_part = timestamp_str.substr(t_pos + 1, plus_pos - t_pos - 1);

    // 解析时:分:秒.微秒
    int hour, minute;
    double second;
    sscanf(time_part.c_str(), "%d:%d:%lf", &hour, &minute, &second);

    return hour * 3600.0 + minute * 60.0 + second;
}

// 传感器数据结构
struct SensorData {
    double timestamp;
    // IMU数据
    double acc_x, acc_y, acc_z;
    double gyr_x, gyr_y, gyr_z;
    // 磁力计数据
    double mag_x, mag_y, mag_z;
    // DVL数据
    double dvl_vx, dvl_vy, dvl_vz;
    // GPS数据
    double gps_lon, gps_lat, gps_alt;
    double gps_east, gps_north, gps_up;
};

// 读取CSV文件
vector<SensorData> readCSV(const string& filename) {
    vector<SensorData> data;
    ifstream file(filename);
    
    if (!file.is_open()) {
        cerr << "Error: Could not open file " << filename << endl;
        return data;
    }
    
    string line;
    // 读取标题行
    getline(file, line);
    
    // 读取数据行
    while (getline(file, line)) {
        SensorData sd;
        stringstream ss(line);
        string value;
        
        // 读取时间戳
        getline(ss, value, ',');
        sd.timestamp = parseTimestamp(value);
        
        // 读取IMU数据
        getline(ss, value, ','); sd.acc_x = stod(value);
        getline(ss, value, ','); sd.acc_y = stod(value);
        getline(ss, value, ','); sd.acc_z = stod(value);
        getline(ss, value, ','); sd.gyr_x = stod(value);
        getline(ss, value, ','); sd.gyr_y = stod(value);
        getline(ss, value, ','); sd.gyr_z = stod(value);
        
        // 读取磁力计数据
        getline(ss, value, ','); sd.mag_x = stod(value);
        getline(ss, value, ','); sd.mag_y = stod(value);
        getline(ss, value, ','); sd.mag_z = stod(value);
        
        // 读取DVL数据
        getline(ss, value, ','); sd.dvl_vx = stod(value);
        getline(ss, value, ','); sd.dvl_vy = stod(value);
        getline(ss, value, ','); sd.dvl_vz = stod(value);
        
        // 读取GPS数据
        getline(ss, value, ','); sd.gps_lon = stod(value);
        getline(ss, value, ','); sd.gps_lat = stod(value);
        getline(ss, value, ','); sd.gps_alt = stod(value);
        
        // 跳过一些中间列
        for (int i = 0; i < 8; i++) {
            getline(ss, value, ',');
        }
        
        // 读取GPS ENU坐标
        getline(ss, value, ','); sd.gps_east = stod(value);
        getline(ss, value, ','); sd.gps_north = stod(value);
        getline(ss, value, ','); sd.gps_up = stod(value);
        
        data.push_back(sd);
    }
    
    cout << "Read " << data.size() << " data points from " << filename << endl;
    return data;
}

// 主函数
int main(int argc, char* argv[]) {
    // 检查命令行参数
    if (argc < 2) {
        cerr << "Usage: " << argv[0] << " <csv_file>" << endl;
        return 1;
    }

    string csv_file = argv[1];

    // 读取传感器数据
    vector<SensorData> sensor_data = readCSV(csv_file);

    if (sensor_data.empty()) {
        cerr << "No data read from file." << endl;
        return 1;
    }

    // 找到第一个有效GPS数据的索引
    size_t start_index = 0;
    for (size_t i = 0; i < sensor_data.size(); i++) {
        if (sensor_data[i].gps_lon != 0 && sensor_data[i].gps_lat != 0) {
            start_index = i;
            cout << "Found first valid GPS at index " << i << endl;
            cout << "GPS position: (" << sensor_data[i].gps_east << ", "
                 << sensor_data[i].gps_north << ", " << sensor_data[i].gps_up << ")" << endl;
            break;
        }
    }

    if (start_index == 0 && (sensor_data[0].gps_lon == 0 || sensor_data[0].gps_lat == 0)) {
        cerr << "No valid GPS data found in the dataset!" << endl;
        return 1;
    }
    
    // ISAM2参数 - 增加稳定性
    ISAM2Params isam_params;
    isam_params.relinearizeThreshold = 0.01;  // 更频繁的重线性化
    isam_params.relinearizeSkip = 1;
    isam_params.enableRelinearization = true;
    isam_params.evaluateNonlinearError = false;
    isam_params.factorization = ISAM2Params::CHOLESKY;
    ISAM2 isam(isam_params);
    
    // 创建因子图和初始值
    NonlinearFactorGraph graph;
    Values initial_values;
    
    // IMU预积分参数 - MTI-7规格（保守估计）
    boost::shared_ptr<PreintegrationParams> imu_params = PreintegrationParams::MakeSharedD(9.81);
    // MTI-7加速度计噪声（保守估计）
    imu_params->accelerometerCovariance = Matrix33::Identity() * 0.01;  // 0.1 m/s²
    // MTI-7陀螺仪噪声（保守估计）
    imu_params->gyroscopeCovariance = Matrix33::Identity() * 0.001;     // 0.03 rad/s
    imu_params->integrationCovariance = Matrix33::Identity() * 1e-6;    // 积分噪声
    
    // 创建IMU预积分器
    PreintegratedImuMeasurements imu_integrator(imu_params);

    // 使用第一个有效GPS位置作为初始位姿
    const SensorData& initial_data = sensor_data[start_index];
    Point3 initial_position(initial_data.gps_east, initial_data.gps_north, initial_data.gps_up);

    // 使用磁力计数据计算初始航向角
    double initial_yaw = atan2(initial_data.mag_y, initial_data.mag_x);
    Pose3 initial_pose = Pose3(Rot3::RzRyRx(0, 0, initial_yaw), initial_position);

    // 使用DVL数据作为初始速度（如果可用）
    Vector3 initial_velocity(0, 0, 0);
    if (initial_data.dvl_vx != 0 || initial_data.dvl_vy != 0 || initial_data.dvl_vz != 0) {
        initial_velocity = Vector3(initial_data.dvl_vx, initial_data.dvl_vy, initial_data.dvl_vz);
    }

    imuBias::ConstantBias initial_bias;  // 零偏置

    cout << "Initial pose: " << initial_pose.translation().transpose() << endl;
    cout << "Initial velocity: " << initial_velocity.transpose() << endl;
    
    // 添加先验因子 - 仅在初始化时使用GPS位置
    noiseModel::Diagonal::shared_ptr pose_noise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 2.0, 2.0, 5.0, 0.1, 0.1, 0.1).finished());  // GPS位置作为初始先验，姿态由磁力计确定
    graph.add(PriorFactor<Pose3>(X(start_index), initial_pose, pose_noise));

    noiseModel::Diagonal::shared_ptr velocity_noise = noiseModel::Diagonal::Sigmas(
        (Vector(3) << 0.2, 0.2, 0.2).finished());  // 初始速度不确定性
    graph.add(PriorFactor<Vector3>(V(start_index), initial_velocity, velocity_noise));

    // MTI-7偏置稳定性（保守估计）
    noiseModel::Diagonal::shared_ptr bias_noise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 0.1, 0.1, 0.1, 0.01, 0.01, 0.01).finished());  // MTI-7偏置不确定性
    graph.add(PriorFactor<imuBias::ConstantBias>(B(start_index), initial_bias, bias_noise));
    
    // 设置初始值
    initial_values.insert(X(start_index), initial_pose);
    initial_values.insert(V(start_index), initial_velocity);
    initial_values.insert(B(start_index), initial_bias);

    // 上一个状态
    NavState prev_state(initial_pose, initial_velocity);
    imuBias::ConstantBias prev_bias = initial_bias;

    // 上一个时间戳和索引
    double prev_timestamp = sensor_data[start_index].timestamp;
    size_t prev_index = start_index;
    
    // 结果轨迹
    vector<Point3> trajectory;
    trajectory.push_back(initial_pose.translation());
    
    // 从第一个有效GPS数据之后开始处理（测试版本：只处理前1000个数据点）
    size_t end_index = min(start_index + 1000, sensor_data.size());
    for (size_t i = start_index + 1; i < end_index; i++) {
        // 当前数据
        const SensorData& data = sensor_data[i];
        double dt = data.timestamp - prev_timestamp;
        
        // 跳过时间间隔过大或过小的数据
        if (dt > 1.0 || dt <= 0) {
            if (dt <= 0) {
                cout << "Invalid time interval at index " << i << ": " << dt << " seconds" << endl;
            } else {
                cout << "Large time gap at index " << i << ": " << dt << " seconds" << endl;
            }
            prev_timestamp = data.timestamp;
            continue;
        }

        // 添加IMU测量
        imu_integrator.integrateMeasurement(
            Vector3(data.acc_x, data.acc_y, data.acc_z),
            Vector3(data.gyr_x, data.gyr_y, data.gyr_z),
            dt);
        
        // 每20个IMU测量创建一个新的状态节点（IMU预积分为主线）
        if (i % 20 == 0) {
            // 创建新的状态变量
            Key pose_key = X(i);
            Key vel_key = V(i);
            Key bias_key = B(i);
            
            // 预测新的状态
            NavState predicted_state = imu_integrator.predict(prev_state, prev_bias);
            
            // 添加IMU预积分因子（主要约束）
            ImuFactor imu_factor(
                X(prev_index), V(prev_index),
                pose_key, vel_key,
                B(prev_index),
                imu_integrator);
            graph.add(imu_factor);

            cout << "IMU preintegration factor from " << prev_index << " to " << i << endl;

            // 添加偏置随机游走因子 - MTI-7偏置稳定性（保守估计）
            noiseModel::Diagonal::shared_ptr bias_walk_noise = noiseModel::Diagonal::Sigmas(
                (Vector(6) << 0.001, 0.001, 0.001, 0.0001, 0.0001, 0.0001).finished());  // MTI-7偏置随机游走
            graph.add(BetweenFactor<imuBias::ConstantBias>(
                B(prev_index), bias_key, imuBias::ConstantBias(), bias_walk_noise));
            
            // GPS仅作为参考轨迹，不添加到因子图中
            // 因子图优化仅使用IMU和DVL数据
            
            // DVL仅在有效数据时提供速度约束
            if (abs(data.dvl_vx) > 0.01 || abs(data.dvl_vy) > 0.01 || abs(data.dvl_vz) > 0.01) {
                noiseModel::Diagonal::shared_ptr dvl_noise = noiseModel::Diagonal::Sigmas(
                    (Vector(3) << 0.05, 0.05, 0.1).finished());  // DVL速度约束噪声

                // 添加DVL速度约束因子
                Vector3 dvl_velocity(data.dvl_vx, data.dvl_vy, data.dvl_vz);
                graph.add(PriorFactor<Vector3>(vel_key, dvl_velocity, dvl_noise));

                cout << "DVL velocity constraint at " << i << ": ("
                     << data.dvl_vx << ", " << data.dvl_vy << ", " << data.dvl_vz << ")" << endl;
            }
            
            // 设置初始值
            initial_values.insert(pose_key, predicted_state.pose());
            initial_values.insert(vel_key, predicted_state.velocity());
            initial_values.insert(bias_key, prev_bias);
            
            // 更新ISAM2
            isam.update(graph, initial_values);
            isam.update();
            
            // 获取当前估计
            Values current_estimate = isam.calculateEstimate();
            
            // 更新状态
            prev_state = NavState(
                current_estimate.at<Pose3>(pose_key),
                current_estimate.at<Vector3>(vel_key));
            prev_bias = current_estimate.at<imuBias::ConstantBias>(bias_key);
            
            // 保存轨迹点
            trajectory.push_back(prev_state.pose().translation());
            
            // 清除因子图和初始值，为下一次迭代做准备
            graph.resize(0);
            initial_values.clear();
            
            // 重置IMU积分器
            imu_integrator.resetIntegration();

            // 更新前一个索引
            prev_index = i;

            // 打印进度
            if (i % 1000 == 0) {
                cout << "Processed " << i << " / " << sensor_data.size()
                     << " (" << (i * 100.0 / sensor_data.size()) << "%)" << endl;
                cout << "Current position: " << prev_state.pose().translation().transpose() << endl;
            }
        }

        prev_timestamp = data.timestamp;
    }
    
    // 保存轨迹到文件
    ofstream trajectory_file("/home/dongchenhui/LSTM_TimeFGO_nav/gtsam_project/results/trajectory.csv");
    trajectory_file << "x,y,z" << endl;
    for (const auto& point : trajectory) {
        trajectory_file << point.x() << "," << point.y() << "," << point.z() << endl;
    }
    trajectory_file.close();
    
    cout << "Saved trajectory with " << trajectory.size() << " points." << endl;
    
    return 0;
}
