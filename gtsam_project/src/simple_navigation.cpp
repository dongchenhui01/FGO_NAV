/**
 * 简化版AUV导航算法 - IMU预积分 + DVL速度约束
 * 
 * 因子图架构:
 * - IMU预积分因子: 主要约束
 * - DVL速度因子: 速度约束
 * - 偏置因子: IMU偏置模型
 * 
 * 使用批量优化，简化版本
 */

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

// GTSAM相关头文件
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/NavState.h>

using namespace std;
using namespace gtsam;

using symbol_shorthand::X; // 位姿
using symbol_shorthand::V; // 速度
using symbol_shorthand::B; // IMU偏置

// 传感器数据结构
struct SensorData {
    double timestamp;
    double acc_x, acc_y, acc_z;
    double gyr_x, gyr_y, gyr_z;
    double dvl_vx, dvl_vy, dvl_vz;
    double gps_east, gps_north, gps_up;
};

// 解析时间戳
double parseTimestamp(const string& timestamp_str) {
    size_t t_pos = timestamp_str.find('T');
    size_t plus_pos = timestamp_str.find('+');
    
    if (t_pos == string::npos || plus_pos == string::npos) {
        return 0.0;
    }
    
    string time_part = timestamp_str.substr(t_pos + 1, plus_pos - t_pos - 1);
    int hour, minute;
    double second;
    sscanf(time_part.c_str(), "%d:%d:%lf", &hour, &minute, &second);
    
    return hour * 3600.0 + minute * 60.0 + second;
}

// 读取CSV文件
vector<SensorData> readCSV(const string& filename, int max_lines = 500) {
    vector<SensorData> data;
    ifstream file(filename);
    
    if (!file.is_open()) {
        cerr << "Error: Could not open file " << filename << endl;
        return data;
    }
    
    string line;
    getline(file, line); // 跳过标题行
    
    int count = 0;
    while (getline(file, line) && count < max_lines) {
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
        
        // 跳过磁力计数据
        for (int i = 0; i < 3; i++) getline(ss, value, ',');
        
        // 读取DVL数据
        getline(ss, value, ','); sd.dvl_vx = stod(value);
        getline(ss, value, ','); sd.dvl_vy = stod(value);
        getline(ss, value, ','); sd.dvl_vz = stod(value);
        
        // 跳过GPS经纬度等
        for (int i = 0; i < 15; i++) getline(ss, value, ',');
        
        // 读取GPS ENU坐标
        getline(ss, value, ','); sd.gps_east = stod(value);
        getline(ss, value, ','); sd.gps_north = stod(value);
        getline(ss, value, ','); sd.gps_up = stod(value);
        
        data.push_back(sd);
        count++;
    }
    
    cout << "Read " << data.size() << " data points" << endl;
    return data;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        cerr << "Usage: " << argv[0] << " <csv_file>" << endl;
        return 1;
    }
    
    // 读取数据（限制数量）
    vector<SensorData> sensor_data = readCSV(argv[1], 200);
    
    if (sensor_data.empty()) {
        cerr << "No data read from file." << endl;
        return 1;
    }
    
    // 找到第一个有效GPS数据
    size_t start_index = 0;
    for (size_t i = 0; i < sensor_data.size(); i++) {
        if (sensor_data[i].gps_east != 0 && sensor_data[i].gps_north != 0) {
            start_index = i;
            cout << "Found first valid GPS at index " << i << endl;
            break;
        }
    }
    
    // 创建因子图和初始值
    NonlinearFactorGraph graph;
    Values initial_values;
    
    // IMU预积分参数
    boost::shared_ptr<PreintegrationParams> imu_params = PreintegrationParams::MakeSharedD(9.81);
    imu_params->accelerometerCovariance = Matrix33::Identity() * 0.01;
    imu_params->gyroscopeCovariance = Matrix33::Identity() * 0.001;
    imu_params->integrationCovariance = Matrix33::Identity() * 1e-6;
    
    // 初始状态
    const SensorData& initial_data = sensor_data[start_index];
    Point3 initial_position(initial_data.gps_east, initial_data.gps_north, initial_data.gps_up);
    Pose3 initial_pose = Pose3(Rot3::RzRyRx(0, 0, 0), initial_position);
    Vector3 initial_velocity(initial_data.dvl_vx, initial_data.dvl_vy, initial_data.dvl_vz);
    imuBias::ConstantBias initial_bias;
    
    // 添加先验因子
    noiseModel::Diagonal::shared_ptr pose_noise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 1.0, 1.0, 2.0, 0.1, 0.1, 0.1).finished());
    graph.add(PriorFactor<Pose3>(X(0), initial_pose, pose_noise));
    
    noiseModel::Diagonal::shared_ptr velocity_noise = noiseModel::Diagonal::Sigmas(
        (Vector(3) << 0.1, 0.1, 0.1).finished());
    graph.add(PriorFactor<Vector3>(V(0), initial_velocity, velocity_noise));
    
    noiseModel::Diagonal::shared_ptr bias_noise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 0.1, 0.1, 0.1, 0.01, 0.01, 0.01).finished());
    graph.add(PriorFactor<imuBias::ConstantBias>(B(0), initial_bias, bias_noise));
    
    // 设置初始值
    initial_values.insert(X(0), initial_pose);
    initial_values.insert(V(0), initial_velocity);
    initial_values.insert(B(0), initial_bias);
    
    // 创建IMU预积分器
    PreintegratedImuMeasurements imu_integrator(imu_params);
    
    // 处理数据
    double prev_timestamp = sensor_data[start_index].timestamp;
    NavState prev_state(initial_pose, initial_velocity);
    imuBias::ConstantBias prev_bias = initial_bias;
    
    vector<Point3> trajectory;
    trajectory.push_back(initial_pose.translation());
    
    int state_count = 1;
    
    // 每10个数据点创建一个状态
    for (size_t i = start_index + 1; i < sensor_data.size() && state_count < 10; i += 10) {
        // 预积分IMU数据
        for (size_t j = max(start_index + 1, i - 9); j <= i && j < sensor_data.size(); j++) {
            const SensorData& data = sensor_data[j];
            double dt = data.timestamp - prev_timestamp;
            
            if (dt > 0 && dt < 1.0) {
                imu_integrator.integrateMeasurement(
                    Vector3(data.acc_x, data.acc_y, data.acc_z),
                    Vector3(data.gyr_x, data.gyr_y, data.gyr_z),
                    dt);
            }
            prev_timestamp = data.timestamp;
        }
        
        // 创建新状态
        Key pose_key = X(state_count);
        Key vel_key = V(state_count);
        Key bias_key = B(state_count);
        
        // 预测状态
        NavState predicted_state = imu_integrator.predict(prev_state, prev_bias);
        
        // 添加IMU因子
        ImuFactor imu_factor(X(state_count-1), V(state_count-1), pose_key, vel_key, B(state_count-1), imu_integrator);
        graph.add(imu_factor);
        
        // 添加DVL约束
        const SensorData& current_data = sensor_data[i];
        if (abs(current_data.dvl_vx) > 0.01 || abs(current_data.dvl_vy) > 0.01) {
            noiseModel::Diagonal::shared_ptr dvl_noise = noiseModel::Diagonal::Sigmas(
                (Vector(3) << 0.1, 0.1, 0.2).finished());
            Vector3 dvl_velocity(current_data.dvl_vx, current_data.dvl_vy, current_data.dvl_vz);
            graph.add(PriorFactor<Vector3>(vel_key, dvl_velocity, dvl_noise));
        }
        
        // 设置初始值
        initial_values.insert(pose_key, predicted_state.pose());
        initial_values.insert(vel_key, predicted_state.velocity());
        initial_values.insert(bias_key, prev_bias);
        
        // 更新状态
        prev_state = predicted_state;
        trajectory.push_back(predicted_state.pose().translation());
        
        // 重置积分器
        imu_integrator.resetIntegration();
        
        state_count++;
        cout << "Created state " << state_count-1 << " at index " << i << endl;
    }
    
    // 优化
    cout << "Optimizing factor graph with " << graph.size() << " factors..." << endl;
    LevenbergMarquardtOptimizer optimizer(graph, initial_values);
    Values result = optimizer.optimize();
    
    // 保存轨迹
    ofstream trajectory_file("results/trajectory.csv");
    trajectory_file << "x,y,z" << endl;
    
    for (int i = 0; i < state_count; i++) {
        Pose3 pose = result.at<Pose3>(X(i));
        Point3 position = pose.translation();
        trajectory_file << position.x() << "," << position.y() << "," << position.z() << endl;
    }
    trajectory_file.close();
    
    cout << "Saved trajectory with " << state_count << " points." << endl;
    
    return 0;
}
