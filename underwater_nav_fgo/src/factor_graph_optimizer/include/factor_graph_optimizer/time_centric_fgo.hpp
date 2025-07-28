#pragma once

#include <memory>
#include <deque>
#include <map>
#include <mutex>
#include <thread>
#include <chrono>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>

#include "underwater_nav_msgs/msg/imu_data.hpp"
#include "underwater_nav_msgs/msg/dvl_data.hpp"
#include "underwater_nav_msgs/msg/navigation_state.hpp"

namespace factor_graph_optimizer {

using gtsam::symbol_shorthand::X; // Pose3 (位姿)
using gtsam::symbol_shorthand::V; // Vector3 (速度)
using gtsam::symbol_shorthand::B; // imuBias::ConstantBias (IMU偏差)

/**
 * @brief 时间戳数据结构
 */
struct TimestampedData {
    double timestamp;
    enum DataType { IMU, DVL, MAGNETOMETER } type;
    std::shared_ptr<void> data;
    
    TimestampedData(double t, DataType dt, std::shared_ptr<void> d) 
        : timestamp(t), type(dt), data(d) {}
};

/**
 * @brief 连续时间轨迹点
 */
struct ContinuousTrajectoryPoint {
    double timestamp;
    gtsam::Pose3 pose;
    gtsam::Vector3 velocity;
    gtsam::Vector3 acceleration;
    gtsam::Vector3 angular_velocity;
    gtsam::imuBias::ConstantBias bias;
    gtsam::Matrix covariance;
    
    ContinuousTrajectoryPoint() : timestamp(0.0) {}
    ContinuousTrajectoryPoint(double t) : timestamp(t) {}
};

/**
 * @brief 高斯过程回归参数
 */
struct GPRegressionParams {
    double length_scale;        // 长度尺度参数
    double signal_variance;     // 信号方差
    double noise_variance;      // 噪声方差
    double time_window;         // 时间窗口大小
    int max_control_points;     // 最大控制点数量
    
    GPRegressionParams() 
        : length_scale(0.1), signal_variance(1.0), noise_variance(0.01),
          time_window(1.0), max_control_points(20) {}
};

/**
 * @brief 时间中心因子图优化器
 * 
 * 基于gnssFGO论文的实现，使用连续时间轨迹表示和高斯过程回归
 */
class TimeCentricFGO {
public:
    /**
     * @brief 构造函数
     */
    TimeCentricFGO();
    
    /**
     * @brief 析构函数
     */
    ~TimeCentricFGO();
    
    /**
     * @brief 初始化优化器
     * @param initial_pose 初始位姿
     * @param initial_velocity 初始速度
     * @param initial_bias 初始IMU偏差
     * @param start_time 开始时间
     * @return 成功返回true
     */
    bool initialize(const gtsam::Pose3& initial_pose,
                   const gtsam::Vector3& initial_velocity,
                   const gtsam::imuBias::ConstantBias& initial_bias,
                   double start_time);
    
    /**
     * @brief 添加IMU测量数据
     * @param imu_data IMU数据
     */
    void addImuMeasurement(const underwater_nav_msgs::msg::ImuData& imu_data);
    
    /**
     * @brief 添加DVL测量数据
     * @param dvl_data DVL数据
     */
    void addDvlMeasurement(const underwater_nav_msgs::msg::DvlData& dvl_data);
    
    /**
     * @brief 添加磁力计测量数据
     * @param timestamp 时间戳
     * @param magnetic_field 磁场向量
     */
    void addMagnetometerMeasurement(double timestamp, const gtsam::Vector3& magnetic_field);
    
    /**
     * @brief 执行时间中心优化
     * @param current_time 当前时间
     * @return 成功返回true
     */
    bool optimizeAtTime(double current_time);
    
    /**
     * @brief 查询指定时间的轨迹状态
     * @param query_time 查询时间
     * @return 轨迹点
     */
    ContinuousTrajectoryPoint queryTrajectoryAtTime(double query_time) const;
    
    /**
     * @brief 获取当前导航状态消息
     * @return ROS2导航状态消息
     */
    underwater_nav_msgs::msg::NavigationState getNavigationStateMsg() const;
    
    /**
     * @brief 设置高斯过程回归参数
     * @param params GP参数
     */
    void setGPRegressionParams(const GPRegressionParams& params);
    
    /**
     * @brief 设置IMU参数
     * @param params IMU预积分参数
     */
    void setImuParams(const gtsam::PreintegratedCombinedMeasurements::Params& params);
    
    /**
     * @brief 设置DVL噪声模型
     * @param noise_model DVL噪声模型
     */
    void setDvlNoiseModel(const gtsam::SharedNoiseModel& noise_model);
    
    /**
     * @brief 获取优化统计信息
     */
    struct OptimizationStats {
        int iterations;
        double solve_time;
        double error_before;
        double error_after;
        size_t num_factors;
        size_t num_variables;
        size_t num_control_points;
        double time_window_size;
    };
    
    OptimizationStats getOptimizationStats() const;

private:
    /**
     * @brief 处理时间戳数据队列
     * @param current_time 当前时间
     */
    void processTimestampedData(double current_time);
    
    /**
     * @brief 创建连续时间控制点
     * @param timestamp 时间戳
     * @return 控制点键值
     */
    gtsam::Key createControlPoint(double timestamp);
    
    /**
     * @brief 高斯过程插值
     * @param query_time 查询时间
     * @param control_points 控制点
     * @return 插值结果
     */
    ContinuousTrajectoryPoint gaussianProcessInterpolation(
        double query_time, 
        const std::map<double, gtsam::Key>& control_points) const;
    
    /**
     * @brief 计算高斯过程核函数
     * @param t1 时间1
     * @param t2 时间2
     * @return 核函数值
     */
    double gaussianProcessKernel(double t1, double t2) const;
    
    /**
     * @brief 添加连续时间IMU因子
     * @param start_time 开始时间
     * @param end_time 结束时间
     * @param imu_measurements IMU测量序列
     */
    void addContinuousImuFactor(double start_time, double end_time,
                               const std::vector<underwater_nav_msgs::msg::ImuData>& imu_measurements);
    
    /**
     * @brief 添加DVL速度因子
     * @param timestamp 时间戳
     * @param velocity DVL速度测量
     */
    void addDvlVelocityFactor(double timestamp, const gtsam::Vector3& velocity);
    
    /**
     * @brief 添加磁力计因子
     * @param timestamp 时间戳
     * @param magnetic_field 磁场测量
     */
    void addMagnetometerFactor(double timestamp, const gtsam::Vector3& magnetic_field);
    
    /**
     * @brief 管理时间窗口
     * @param current_time 当前时间
     */
    void manageTimeWindow(double current_time);
    
    /**
     * @brief 边缘化旧的控制点
     * @param cutoff_time 截止时间
     */
    void marginalizeOldControlPoints(double cutoff_time);
    
    /**
     * @brief 时间戳转换
     * @param ros_time ROS时间戳
     * @return 秒级时间戳
     */
    double rosTimeToSeconds(const builtin_interfaces::msg::Time& ros_time);

private:
    // GTSAM核心组件
    std::unique_ptr<gtsam::ISAM2> isam2_;
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_values_;
    gtsam::Values current_values_;
    
    // 时间中心数据管理
    std::deque<TimestampedData> timestamped_data_queue_;
    std::map<double, gtsam::Key> control_points_;  // 时间戳 -> 控制点键值
    std::map<gtsam::Key, ContinuousTrajectoryPoint> trajectory_points_;
    
    // 高斯过程回归参数
    GPRegressionParams gp_params_;
    
    // IMU预积分
    std::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> imu_params_;
    std::map<double, std::unique_ptr<gtsam::PreintegratedCombinedMeasurements>> imu_preintegrated_map_;
    
    // 噪声模型
    gtsam::SharedNoiseModel dvl_noise_model_;
    gtsam::SharedNoiseModel magnetometer_noise_model_;
    
    // 时间管理
    double start_time_;
    double current_time_;
    double last_optimization_time_;
    
    // 统计信息
    OptimizationStats stats_;
    
    // 线程安全
    mutable std::mutex mutex_;
    
    // 初始化标志
    bool initialized_;
    
    // 键值计数器
    uint64_t key_counter_;
};

/**
 * @brief 连续时间轨迹因子
 *
 * 基于高斯过程的连续时间轨迹表示因子
 */
class ContinuousTrajectoryFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3> {
private:
    double query_time_;
    double control_time1_, control_time2_;
    GPRegressionParams gp_params_;

public:
    GTSAM_MAKE_ALIGNED_OPERATOR_NEW

    ContinuousTrajectoryFactor(gtsam::Key pose_key1, gtsam::Key pose_key2,
                              double query_time, double control_time1, double control_time2,
                              const GPRegressionParams& gp_params,
                              const gtsam::SharedNoiseModel& noise_model)
        : NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3>(noise_model, pose_key1, pose_key2),
          query_time_(query_time), control_time1_(control_time1), control_time2_(control_time2),
          gp_params_(gp_params) {}

    gtsam::Vector evaluateError(const gtsam::Pose3& pose1, const gtsam::Vector3& vel1,
                               boost::optional<gtsam::Matrix&> H1 = boost::none,
                               boost::optional<gtsam::Matrix&> H2 = boost::none) const override;

    gtsam::NonlinearFactor::shared_ptr clone() const override {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new ContinuousTrajectoryFactor(*this)));
    }
};

} // namespace factor_graph_optimizer
