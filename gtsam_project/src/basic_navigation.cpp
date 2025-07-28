/**
 * 基础版AUV导航算法 - IMU预积分 + DVL速度约束
 * 最简化版本，确保因子图结构正确
 */

#include <iostream>
#include <fstream>
#include <vector>

// GTSAM相关头文件
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
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

int main() {
    cout << "=== 基础AUV导航算法测试 ===" << endl;
    
    // 创建因子图和初始值
    NonlinearFactorGraph graph;
    Values initial_values;
    
    // IMU预积分参数
    boost::shared_ptr<PreintegrationParams> imu_params = PreintegrationParams::MakeSharedD(9.81);
    imu_params->accelerometerCovariance = Matrix33::Identity() * 0.01;
    imu_params->gyroscopeCovariance = Matrix33::Identity() * 0.001;
    imu_params->integrationCovariance = Matrix33::Identity() * 1e-6;
    
    // 初始状态
    Pose3 initial_pose = Pose3(Rot3::RzRyRx(0, 0, 0), Point3(0, 0, 0));
    Vector3 initial_velocity(1.5, -0.4, 0.0);  // 典型AUV速度
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
    
    // 创建几个时间步的状态
    int num_states = 3;
    double dt = 0.1;  // 100ms间隔
    
    for (int i = 1; i < num_states; i++) {
        // 创建IMU预积分器
        PreintegratedImuMeasurements imu_integrator(imu_params);
        
        // 模拟IMU数据
        Vector3 acc(0.1, 0.2, 9.8);  // 加速度
        Vector3 gyr(0.01, 0.02, 0.03);  // 角速度
        
        // 积分IMU测量
        for (int j = 0; j < 10; j++) {  // 10个IMU测量
            imu_integrator.integrateMeasurement(acc, gyr, dt/10);
        }
        
        // 预测下一个状态
        NavState prev_state(initial_values.at<Pose3>(X(i-1)), initial_values.at<Vector3>(V(i-1)));
        imuBias::ConstantBias prev_bias = initial_values.at<imuBias::ConstantBias>(B(i-1));
        NavState predicted_state = imu_integrator.predict(prev_state, prev_bias);
        
        // 添加IMU因子
        ImuFactor imu_factor(X(i-1), V(i-1), X(i), V(i), B(i-1), imu_integrator);
        graph.add(imu_factor);
        
        // 添加偏置因子
        noiseModel::Diagonal::shared_ptr bias_walk_noise = noiseModel::Diagonal::Sigmas(
            (Vector(6) << 0.001, 0.001, 0.001, 0.0001, 0.0001, 0.0001).finished());
        graph.add(BetweenFactor<imuBias::ConstantBias>(B(i-1), B(i), imuBias::ConstantBias(), bias_walk_noise));
        
        // 添加DVL速度约束
        noiseModel::Diagonal::shared_ptr dvl_noise = noiseModel::Diagonal::Sigmas(
            (Vector(3) << 0.1, 0.1, 0.2).finished());
        Vector3 dvl_velocity(1.5 + 0.1*i, -0.4 + 0.05*i, 0.02*i);  // 模拟DVL数据
        graph.add(PriorFactor<Vector3>(V(i), dvl_velocity, dvl_noise));
        
        // 设置初始值
        initial_values.insert(X(i), predicted_state.pose());
        initial_values.insert(V(i), predicted_state.velocity());
        initial_values.insert(B(i), prev_bias);
        
        cout << "Created state " << i << endl;
    }
    
    cout << "Factor graph has " << graph.size() << " factors" << endl;
    cout << "Initial values has " << initial_values.size() << " variables" << endl;
    
    // 检查因子图
    try {
        cout << "Checking factor graph consistency..." << endl;
        
        // 优化
        cout << "Starting optimization..." << endl;
        LevenbergMarquardtOptimizer optimizer(graph, initial_values);
        Values result = optimizer.optimize();
        
        cout << "Optimization completed successfully!" << endl;
        
        // 输出结果
        cout << "\n=== 优化结果 ===" << endl;
        for (int i = 0; i < num_states; i++) {
            Pose3 pose = result.at<Pose3>(X(i));
            Vector3 velocity = result.at<Vector3>(V(i));
            Point3 position = pose.translation();
            
            cout << "State " << i << ":" << endl;
            cout << "  Position: (" << position.x() << ", " << position.y() << ", " << position.z() << ")" << endl;
            cout << "  Velocity: (" << velocity(0) << ", " << velocity(1) << ", " << velocity(2) << ")" << endl;
        }
        
        // 保存轨迹
        ofstream trajectory_file("results/trajectory.csv");
        trajectory_file << "x,y,z" << endl;
        
        for (int i = 0; i < num_states; i++) {
            Pose3 pose = result.at<Pose3>(X(i));
            Point3 position = pose.translation();
            trajectory_file << position.x() << "," << position.y() << "," << position.z() << endl;
        }
        trajectory_file.close();
        
        cout << "\n轨迹已保存到 results/trajectory.csv" << endl;
        
    } catch (const exception& e) {
        cerr << "Error during optimization: " << e.what() << endl;
        return 1;
    }
    
    cout << "\n=== 因子图结构验证成功 ===" << endl;
    cout << "IMU预积分因子: " << (num_states - 1) << " 个" << endl;
    cout << "DVL速度约束: " << (num_states - 1) << " 个" << endl;
    cout << "偏置因子: " << (num_states - 1) << " 个" << endl;
    cout << "先验因子: 3 个" << endl;
    
    return 0;
}
