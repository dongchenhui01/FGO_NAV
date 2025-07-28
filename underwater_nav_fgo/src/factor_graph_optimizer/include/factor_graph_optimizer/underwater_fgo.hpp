#pragma once

#include <memory>
#include <deque>
#include <map>
#include <queue>
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

#include "underwater_nav_msgs/msg/imu_data.hpp"
#include "underwater_nav_msgs/msg/dvl_data.hpp"
#include "underwater_nav_msgs/msg/navigation_state.hpp"

namespace factor_graph_optimizer {

using gtsam::symbol_shorthand::X; // Pose3 (位姿)
using gtsam::symbol_shorthand::V; // Vector3 (速度)
using gtsam::symbol_shorthand::B; // imuBias::ConstantBias (IMU偏差)

/**
 * @brief 时间戳数据结构 (基于gnssFGO论文)
 */
struct TimestampedMeasurement {
    double timestamp;
    enum MeasurementType { IMU, DVL, MAGNETOMETER } type;
    std::shared_ptr<void> data;
    bool processed;

    TimestampedMeasurement(double t, MeasurementType mt, std::shared_ptr<void> d)
        : timestamp(t), type(mt), data(d), processed(false) {}

    // 比较函数用于优先队列排序
    bool operator<(const TimestampedMeasurement& other) const {
        return timestamp > other.timestamp; // 最小堆，最早时间优先
    }
};

/**
 * @brief 连续时间轨迹点 (基于高斯过程回归)
 */
struct ContinuousTrajectoryPoint {
    double timestamp;
    gtsam::Pose3 pose;
    gtsam::Vector3 velocity;
    gtsam::Vector3 acceleration;
    gtsam::imuBias::ConstantBias bias;
    gtsam::Matrix covariance;
    double confidence;  // 插值置信度

    ContinuousTrajectoryPoint() : timestamp(0.0), confidence(0.0) {}
    ContinuousTrajectoryPoint(double t) : timestamp(t), confidence(0.0) {}
};

/**
 * @brief 导航状态结构 (扩展支持时间中心)
 */
struct NavigationState {
    gtsam::NavState nav_state;      // 位姿和速度
    gtsam::imuBias::ConstantBias bias; // IMU偏差
    double timestamp;               // 时间戳
    gtsam::Matrix covariance;       // 协方差矩阵

    // 时间中心扩展
    bool is_interpolated;           // 是否为插值结果
    double interpolation_confidence; // 插值置信度
};

/**
 * @brief 水下因子图优化器
 */
class UnderwaterFGO {
public:
    /**
     * @brief 构造函数
     */
    UnderwaterFGO();
    
    /**
     * @brief 析构函数
     */
    ~UnderwaterFGO();
    
    /**
     * @brief 初始化优化器
     * @param initial_pose 初始位姿
     * @param initial_velocity 初始速度
     * @param initial_bias 初始IMU偏差
     * @return 成功返回true
     */
    bool initialize(const gtsam::Pose3& initial_pose,
                   const gtsam::Vector3& initial_velocity,
                   const gtsam::imuBias::ConstantBias& initial_bias);
    
    /**
     * @brief 添加IMU测量 (支持时间中心模式)
     * @param imu_data IMU数据
     */
    void addImuMeasurement(const underwater_nav_msgs::msg::ImuData& imu_data);

    /**
     * @brief 添加DVL测量 (支持时间中心模式)
     * @param dvl_data DVL数据
     */
    void addDvlMeasurement(const underwater_nav_msgs::msg::DvlData& dvl_data);

    /**
     * @brief 添加磁力计测量 (时间中心模式)
     * @param timestamp 时间戳
     * @param magnetic_field 磁场向量
     */
    void addMagnetometerMeasurement(double timestamp, const gtsam::Vector3& magnetic_field);
    
    /**
     * @brief 执行优化
     * @return 成功返回true
     */
    bool optimize();
    
    /**
     * @brief 获取当前导航状态
     * @return 导航状态
     */
    NavigationState getCurrentState() const;
    
    /**
     * @brief 获取导航状态消息
     * @return ROS2导航状态消息
     */
    underwater_nav_msgs::msg::NavigationState getNavigationStateMsg() const;
    
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
     * @brief 设置磁力计参数
     * @param declination 磁偏角 (弧度)
     * @param noise_model 噪声模型
     */
    void setMagnetometerParams(double declination, const gtsam::SharedNoiseModel& noise_model);
    
    /**
     * @brief 查询指定时间的轨迹状态 (时间中心方法)
     * @param query_time 查询时间戳
     * @return 连续轨迹点
     */
    ContinuousTrajectoryPoint queryTrajectoryAtTime(double query_time) const;

    /**
     * @brief 启用时间中心模式
     * @param enable 是否启用
     * @param time_window 时间窗口大小(秒)
     * @param interpolation_method 插值方法 ("linear", "gp", "spline")
     */
    void enableTimeCentricMode(bool enable, double time_window = 1.0,
                              const std::string& interpolation_method = "gp");

    /**
     * @brief 处理时间戳数据队列 (时间中心核心方法)
     * @param current_time 当前时间
     */
    void processTimestampedData(double current_time);

    /**
     * @brief 获取优化统计信息 (扩展时间中心信息)
     */
    struct OptimizationStats {
        int iterations;
        double solve_time;
        double error_before;
        double error_after;
        size_t num_factors;
        size_t num_variables;

        // 时间中心扩展
        size_t num_timestamped_measurements;
        double time_window_size;
        double oldest_measurement_time;
        double newest_measurement_time;
        std::string interpolation_method;
    };

    OptimizationStats getOptimizationStats() const;

private:
    /**
     * @brief 创建关键帧
     * @param timestamp 时间戳
     * @return 关键帧ID
     */
    uint64_t createKeyframe(double timestamp);
    
    /**
     * @brief 添加IMU因子
     * @param from_key 起始关键帧
     * @param to_key 结束关键帧
     */
    void addImuFactor(uint64_t from_key, uint64_t to_key);
    
    /**
     * @brief 添加DVL因子
     * @param key 关键帧ID
     * @param velocity DVL速度测量
     */
    void addDvlFactor(uint64_t key, const gtsam::Vector3& velocity);
    
    /**
     * @brief 添加磁力计因子
     * @param key 关键帧ID
     * @param magnetic_field 磁场测量
     */
    void addMagnetometerFactor(uint64_t key, const gtsam::Vector3& magnetic_field);
    
    /**
     * @brief 边缘化旧状态
     */
    void marginalizeOldStates();
    
    /**
     * @brief 检测异常值
     * @param factor 因子
     * @return 是否为异常值
     */
    bool isOutlier(const gtsam::NonlinearFactor::shared_ptr& factor);
    
    /**
     * @brief 时间戳转换
     * @param ros_time ROS时间戳
     * @return 秒级时间戳
     */
    double rosTimeToSeconds(const builtin_interfaces::msg::Time& ros_time);

    // ========== 时间中心方法 (基于gnssFGO论文) ==========

    /**
     * @brief 高斯过程插值
     * @param query_time 查询时间
     * @param control_points 控制点映射
     * @return 插值轨迹点
     */
    ContinuousTrajectoryPoint gaussianProcessInterpolation(
        double query_time,
        const std::map<double, gtsam::Key>& control_points) const;

    /**
     * @brief 线性插值 (简化版本)
     * @param query_time 查询时间
     * @param before_point 前一个轨迹点
     * @param after_point 后一个轨迹点
     * @return 插值轨迹点
     */
    ContinuousTrajectoryPoint linearInterpolation(
        double query_time,
        const ContinuousTrajectoryPoint& before_point,
        const ContinuousTrajectoryPoint& after_point) const;

    /**
     * @brief 管理时间窗口
     * @param current_time 当前时间
     */
    void manageTimeWindow(double current_time);

    /**
     * @brief 添加时间戳测量到队列
     * @param measurement 时间戳测量
     */
    void addTimestampedMeasurement(const TimestampedMeasurement& measurement);

    /**
     * @brief 创建时间控制点
     * @param timestamp 时间戳
     * @return 控制点键值
     */
    gtsam::Key createTimeControlPoint(double timestamp);

private:
    // GTSAM核心组件
    std::unique_ptr<gtsam::ISAM2> isam2_;
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_values_;
    gtsam::Values current_values_;
    
    // IMU预积分
    std::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> imu_params_;
    std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> imu_preintegrated_;
    
    // 噪声模型
    gtsam::SharedNoiseModel dvl_noise_model_;
    gtsam::SharedNoiseModel magnetometer_noise_model_;
    
    // 状态管理
    std::deque<NavigationState> state_history_;
    uint64_t current_key_;
    double last_imu_time_;
    double last_dvl_time_;

    // 参数
    size_t window_size_;
    double keyframe_interval_;
    double magnetic_declination_;

    // ========== 时间中心扩展 ==========

    // 时间中心模式控制
    bool time_centric_enabled_;
    double time_window_size_;
    std::string interpolation_method_;

    // 时间戳数据管理
    std::priority_queue<TimestampedMeasurement> timestamped_queue_;
    std::map<double, gtsam::Key> time_control_points_;
    std::map<gtsam::Key, ContinuousTrajectoryPoint> trajectory_cache_;

    // 时间管理
    double start_time_;
    double current_optimization_time_;
    double last_processed_time_;

    // 高斯过程参数
    struct GPParams {
        double length_scale;      // 长度尺度
        double signal_variance;   // 信号方差
        double noise_variance;    // 噪声方差
        int max_control_points;   // 最大控制点数

        GPParams() : length_scale(0.1), signal_variance(1.0),
                    noise_variance(0.01), max_control_points(20) {}
    } gp_params_;

    // 统计信息
    OptimizationStats stats_;

    // 线程安全
    mutable std::mutex mutex_;
    mutable std::mutex time_queue_mutex_;

    // 初始化标志
    bool initialized_;
};

} // namespace factor_graph_optimizer
