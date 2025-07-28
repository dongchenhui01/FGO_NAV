#include "factor_graph_optimizer/underwater_fgo.hpp"
#include "factor_graph_optimizer/dvl_factor.hpp"
#include <rclcpp/rclcpp.hpp>

namespace factor_graph_optimizer {

UnderwaterFGO::UnderwaterFGO() 
    : current_key_(0), last_imu_time_(0.0), last_dvl_time_(0.0),
      window_size_(50), keyframe_interval_(0.1), magnetic_declination_(0.0),
      time_centric_enabled_(false), time_window_size_(1.0), interpolation_method_("linear"),
      start_time_(0.0), current_optimization_time_(0.0), last_processed_time_(0.0),
      initialized_(false), key_counter_(0) {
    
    // 初始化ISAM2
    gtsam::ISAM2Params isam2_params;
    isam2_params.relinearizeThreshold = 0.01;
    isam2_params.relinearizeSkip = 1;
    isam2_ = std::make_unique<gtsam::ISAM2>(isam2_params);
    
    // 初始化统计信息
    stats_ = OptimizationStats();
}

UnderwaterFGO::~UnderwaterFGO() = default;

bool UnderwaterFGO::initialize(const gtsam::Pose3& initial_pose,
                              const gtsam::Vector3& initial_velocity,
                              const gtsam::imuBias::ConstantBias& initial_bias) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (initialized_) {
        return true;
    }
    
    try {
        // 创建初始状态变量
        gtsam::Key pose_key = gtsam::Symbol('X', current_key_);
        gtsam::Key velocity_key = gtsam::Symbol('V', current_key_);
        gtsam::Key bias_key = gtsam::Symbol('B', current_key_);
        
        // 添加初始值
        initial_values_.insert(pose_key, initial_pose);
        initial_values_.insert(velocity_key, initial_velocity);
        initial_values_.insert(bias_key, initial_bias);
        
        // 添加先验因子
        auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished());
        auto velocity_noise = gtsam::noiseModel::Diagonal::Sigmas(
            gtsam::Vector3(0.01, 0.01, 0.01));
        auto bias_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.01, 0.01, 0.01).finished());
        
        graph_.addPrior(pose_key, initial_pose, pose_noise);
        graph_.addPrior(velocity_key, initial_velocity, velocity_noise);
        graph_.addPrior(bias_key, initial_bias, bias_noise);
        
        // 更新ISAM2
        isam2_->update(graph_, initial_values_);
        current_values_ = isam2_->calculateEstimate();
        
        // 清空图和初始值
        graph_.resize(0);
        initial_values_.clear();
        
        // 设置时间
        start_time_ = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        current_optimization_time_ = start_time_;
        
        // 如果启用时间中心模式，创建初始控制点
        if (time_centric_enabled_) {
            createTimeControlPoint(start_time_);
        }
        
        current_key_++;
        initialized_ = true;
        
        RCLCPP_INFO(rclcpp::get_logger("UnderwaterFGO"), "水下导航系统初始化成功");
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("UnderwaterFGO"), 
                    "初始化失败: %s", e.what());
        return false;
    }
}

void UnderwaterFGO::addImuMeasurement(const underwater_nav_msgs::msg::ImuData& imu_data) {
    if (!initialized_) return;
    
    double timestamp = rosTimeToSeconds(imu_data.stamp);
    
    // 如果启用时间中心模式，添加到时间戳队列
    if (time_centric_enabled_) {
        auto imu_ptr = std::make_shared<underwater_nav_msgs::msg::ImuData>(imu_data);
        TimestampedMeasurement measurement(timestamp, TimestampedMeasurement::IMU, imu_ptr);
        addTimestampedMeasurement(measurement);
        return;
    }
    
    // 传统模式处理
    processImuMeasurementTraditional(imu_data, timestamp);
}

void UnderwaterFGO::addDvlMeasurement(const underwater_nav_msgs::msg::DvlData& dvl_data) {
    if (!initialized_) return;
    
    double timestamp = rosTimeToSeconds(dvl_data.stamp);
    
    // 如果启用时间中心模式，添加到时间戳队列
    if (time_centric_enabled_) {
        auto dvl_ptr = std::make_shared<underwater_nav_msgs::msg::DvlData>(dvl_data);
        TimestampedMeasurement measurement(timestamp, TimestampedMeasurement::DVL, dvl_ptr);
        addTimestampedMeasurement(measurement);
        return;
    }
    
    // 传统模式处理
    processDvlMeasurementTraditional(dvl_data, timestamp);
}

void UnderwaterFGO::addMagnetometerMeasurement(double timestamp, const gtsam::Vector3& magnetic_field) {
    if (!initialized_) return;
    
    // 如果启用时间中心模式，添加到时间戳队列
    if (time_centric_enabled_) {
        auto mag_ptr = std::make_shared<gtsam::Vector3>(magnetic_field);
        TimestampedMeasurement measurement(timestamp, TimestampedMeasurement::MAGNETOMETER, mag_ptr);
        addTimestampedMeasurement(measurement);
        return;
    }
    
    // 传统模式处理
    processMagnetometerMeasurementTraditional(magnetic_field, timestamp);
}

bool UnderwaterFGO::optimize() {
    if (!initialized_) return false;
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    double current_time = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    
    // 如果启用时间中心模式，处理时间戳数据
    if (time_centric_enabled_) {
        processTimestampedData(current_time);
    }
    
    try {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // 记录优化前的误差
        stats_.error_before = graph_.error(current_values_);
        
        // 执行优化
        if (graph_.size() > 0) {
            isam2_->update(graph_, initial_values_);
            current_values_ = isam2_->calculateEstimate();
            
            // 清空图和初始值
            graph_.resize(0);
            initial_values_.clear();
        }
        
        // 记录优化后的误差
        stats_.error_after = isam2_->getFactorsUnsafe().error(current_values_);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        stats_.solve_time = std::chrono::duration<double>(end_time - start_time).count();
        stats_.num_factors = isam2_->getFactorsUnsafe().size();
        stats_.num_variables = current_values_.size();
        
        // 更新时间中心统计信息
        if (time_centric_enabled_) {
            stats_.num_timestamped_measurements = timestamped_queue_.size();
            stats_.time_window_size = time_window_size_;
            stats_.interpolation_method = interpolation_method_;
            
            if (!time_control_points_.empty()) {
                stats_.oldest_measurement_time = time_control_points_.begin()->first;
                stats_.newest_measurement_time = time_control_points_.rbegin()->first;
            }
        }
        
        current_optimization_time_ = current_time;
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("UnderwaterFGO"), 
                    "优化失败: %s", e.what());
        return false;
    }
}

NavigationState UnderwaterFGO::getCurrentState() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    NavigationState state;
    state.timestamp = current_optimization_time_;
    state.is_interpolated = false;
    state.interpolation_confidence = 1.0;
    
    if (!initialized_ || current_values_.empty()) {
        return state;
    }
    
    try {
        // 获取最新的状态
        gtsam::Key latest_pose_key = gtsam::Symbol('X', current_key_ - 1);
        gtsam::Key latest_velocity_key = gtsam::Symbol('V', current_key_ - 1);
        gtsam::Key latest_bias_key = gtsam::Symbol('B', current_key_ - 1);
        
        if (current_values_.exists(latest_pose_key) && 
            current_values_.exists(latest_velocity_key) &&
            current_values_.exists(latest_bias_key)) {
            
            gtsam::Pose3 pose = current_values_.at<gtsam::Pose3>(latest_pose_key);
            gtsam::Vector3 velocity = current_values_.at<gtsam::Vector3>(latest_velocity_key);
            gtsam::imuBias::ConstantBias bias = current_values_.at<gtsam::imuBias::ConstantBias>(latest_bias_key);
            
            state.nav_state = gtsam::NavState(pose, velocity);
            state.bias = bias;
            
            // 获取协方差 (简化处理)
            try {
                gtsam::Marginals marginals(isam2_->getFactorsUnsafe(), current_values_);
                state.covariance = marginals.marginalCovariance(latest_pose_key);
            } catch (...) {
                state.covariance = gtsam::Matrix::Identity(6, 6) * 0.01;
            }
        }
        
    } catch (const std::exception& e) {
        RCLCPP_WARN(rclcpp::get_logger("UnderwaterFGO"), 
                   "获取当前状态失败: %s", e.what());
    }
    
    return state;
}

underwater_nav_msgs::msg::NavigationState UnderwaterFGO::getNavigationStateMsg() const {
    underwater_nav_msgs::msg::NavigationState msg;
    
    auto state = getCurrentState();
    
    // 设置时间戳
    msg.stamp.sec = static_cast<int32_t>(state.timestamp);
    msg.stamp.nanosec = static_cast<uint32_t>((state.timestamp - msg.stamp.sec) * 1e9);
    
    // 设置位置
    auto position = state.nav_state.pose().translation();
    msg.position.x = position.x();
    msg.position.y = position.y();
    msg.position.z = position.z();
    
    // 设置速度
    auto velocity = state.nav_state.velocity();
    msg.velocity.x = velocity.x();
    msg.velocity.y = velocity.y();
    msg.velocity.z = velocity.z();
    
    // 设置姿态 (四元数)
    auto rotation = state.nav_state.pose().rotation();
    auto quaternion = rotation.toQuaternion();
    msg.orientation.w = quaternion.w();
    msg.orientation.x = quaternion.x();
    msg.orientation.y = quaternion.y();
    msg.orientation.z = quaternion.z();
    
    // 设置协方差 (简化为对角矩阵)
    msg.covariance.fill(0.0);
    for (int i = 0; i < std::min(15, static_cast<int>(state.covariance.rows())); ++i) {
        if (i < state.covariance.rows() && i < state.covariance.cols()) {
            msg.covariance[i * 15 + i] = state.covariance(i, i);
        }
    }
    
    // 设置精度指标
    msg.position_accuracy = std::sqrt(state.covariance.block<3,3>(0,0).trace() / 3.0);
    msg.velocity_accuracy = std::sqrt(state.covariance.block<3,3>(3,3).trace() / 3.0);
    
    // 设置状态标志
    msg.is_initialized = initialized_;
    msg.is_converged = (stats_.error_after < stats_.error_before * 0.99);
    msg.optimization_iterations = stats_.iterations;
    
    return msg;
}

UnderwaterFGO::OptimizationStats UnderwaterFGO::getOptimizationStats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
}

double UnderwaterFGO::rosTimeToSeconds(const builtin_interfaces::msg::Time& ros_time) {
    return static_cast<double>(ros_time.sec) + static_cast<double>(ros_time.nanosec) * 1e-9;
}

// ========== 传统模式处理方法 ==========

void UnderwaterFGO::processImuMeasurementTraditional(const underwater_nav_msgs::msg::ImuData& imu_data, double timestamp) {
    // 传统的IMU处理逻辑
    // 这里可以实现基于关键帧的IMU预积分

    if (timestamp - last_imu_time_ > keyframe_interval_) {
        // 创建新的关键帧
        createKeyframe(timestamp);
        last_imu_time_ = timestamp;
    }
}

void UnderwaterFGO::processDvlMeasurementTraditional(const underwater_nav_msgs::msg::DvlData& dvl_data, double timestamp) {
    // 传统的DVL处理逻辑
    if (!dvl_data.is_valid) return;

    gtsam::Vector3 velocity(dvl_data.velocity.x, dvl_data.velocity.y, dvl_data.velocity.z);

    // 添加DVL因子到当前关键帧
    gtsam::Key pose_key = gtsam::Symbol('X', current_key_ - 1);
    gtsam::Key velocity_key = gtsam::Symbol('V', current_key_ - 1);

    if (dvl_noise_model_) {
        auto dvl_factor = boost::make_shared<DvlFactor>(pose_key, velocity_key, velocity, dvl_noise_model_);
        graph_.add(dvl_factor);
    }

    last_dvl_time_ = timestamp;
}

void UnderwaterFGO::processMagnetometerMeasurementTraditional(const gtsam::Vector3& magnetic_field, double timestamp) {
    // 传统的磁力计处理逻辑
    gtsam::Key pose_key = gtsam::Symbol('X', current_key_ - 1);

    if (magnetometer_noise_model_) {
        auto mag_factor = boost::make_shared<MagnetometerFactor>(
            pose_key, magnetic_field, magnetic_declination_, magnetometer_noise_model_);
        graph_.add(mag_factor);
    }
}

// ========== 时间中心模式处理方法 ==========

void UnderwaterFGO::processImuMeasurementAtTime(const underwater_nav_msgs::msg::ImuData& imu_data, double timestamp) {
    // 时间中心的IMU处理
    // 根据时间戳创建或更新控制点

    gtsam::Key control_point = getOrCreateControlPoint(timestamp);

    // 添加IMU约束到最近的控制点
    // 这里可以实现连续时间的IMU预积分
}

void UnderwaterFGO::processDvlMeasurementAtTime(const underwater_nav_msgs::msg::DvlData& dvl_data, double timestamp) {
    // 时间中心的DVL处理
    if (!dvl_data.is_valid) return;

    gtsam::Vector3 velocity(dvl_data.velocity.x, dvl_data.velocity.y, dvl_data.velocity.z);

    // 查找或创建时间控制点
    gtsam::Key control_point = getOrCreateControlPoint(timestamp);

    // 添加DVL速度约束
    if (dvl_noise_model_) {
        // 这里需要实现时间中心的DVL因子
        addDvlVelocityFactor(timestamp, velocity);
    }
}

void UnderwaterFGO::processMagnetometerMeasurementAtTime(const gtsam::Vector3& magnetic_field, double timestamp) {
    // 时间中心的磁力计处理
    gtsam::Key control_point = getOrCreateControlPoint(timestamp);

    if (magnetometer_noise_model_) {
        addMagnetometerFactor(timestamp, magnetic_field);
    }
}

// ========== 辅助方法 ==========

uint64_t UnderwaterFGO::createKeyframe(double timestamp) {
    gtsam::Key pose_key = gtsam::Symbol('X', current_key_);
    gtsam::Key velocity_key = gtsam::Symbol('V', current_key_);
    gtsam::Key bias_key = gtsam::Symbol('B', current_key_);

    // 添加初始估计值 (基于前一个状态)
    if (current_key_ > 0) {
        gtsam::Key prev_pose_key = gtsam::Symbol('X', current_key_ - 1);
        gtsam::Key prev_velocity_key = gtsam::Symbol('V', current_key_ - 1);
        gtsam::Key prev_bias_key = gtsam::Symbol('B', current_key_ - 1);

        if (current_values_.exists(prev_pose_key)) {
            initial_values_.insert(pose_key, current_values_.at<gtsam::Pose3>(prev_pose_key));
            initial_values_.insert(velocity_key, current_values_.at<gtsam::Vector3>(prev_velocity_key));
            initial_values_.insert(bias_key, current_values_.at<gtsam::imuBias::ConstantBias>(prev_bias_key));
        }
    }

    return current_key_++;
}

gtsam::Key UnderwaterFGO::getOrCreateControlPoint(double timestamp) {
    // 查找最近的控制点
    auto it = time_control_points_.lower_bound(timestamp);

    // 如果存在足够近的控制点，使用它
    if (it != time_control_points_.end() && std::abs(it->first - timestamp) < 0.01) {
        return it->second;
    }

    // 否则创建新的控制点
    return createTimeControlPoint(timestamp);
}

void UnderwaterFGO::setImuParams(const gtsam::PreintegratedCombinedMeasurements::Params& params) {
    imu_params_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements::Params>(params);
}

void UnderwaterFGO::setDvlNoiseModel(const gtsam::SharedNoiseModel& noise_model) {
    dvl_noise_model_ = noise_model;
}

void UnderwaterFGO::setMagnetometerParams(double declination, const gtsam::SharedNoiseModel& noise_model) {
    magnetic_declination_ = declination;
    magnetometer_noise_model_ = noise_model;
}

} // namespace factor_graph_optimizer
