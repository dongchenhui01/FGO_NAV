#include "factor_graph_optimizer/underwater_fgo.hpp"
#include <cmath>
#include <algorithm>

namespace factor_graph_optimizer {

// ========== 时间中心方法实现 (基于gnssFGO论文) ==========

void UnderwaterFGO::enableTimeCentricMode(bool enable, double time_window, 
                                         const std::string& interpolation_method) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    time_centric_enabled_ = enable;
    time_window_size_ = time_window;
    interpolation_method_ = interpolation_method;
    
    if (enable) {
        RCLCPP_INFO(rclcpp::get_logger("UnderwaterFGO"), 
                   "时间中心模式已启用 - 窗口大小: %.2fs, 插值方法: %s", 
                   time_window, interpolation_method.c_str());
    }
}

void UnderwaterFGO::addTimestampedMeasurement(const TimestampedMeasurement& measurement) {
    std::lock_guard<std::mutex> lock(time_queue_mutex_);
    timestamped_queue_.push(measurement);
}

void UnderwaterFGO::processTimestampedData(double current_time) {
    if (!time_centric_enabled_) return;
    
    std::lock_guard<std::mutex> lock(time_queue_mutex_);
    
    // 处理时间窗口内的所有测量
    while (!timestamped_queue_.empty()) {
        const auto& measurement = timestamped_queue_.top();
        
        // 只处理时间窗口内的数据
        if (current_time - measurement.timestamp > time_window_size_) {
            timestamped_queue_.pop();
            continue;
        }
        
        // 如果数据太新，等待后续处理
        if (measurement.timestamp > current_time) {
            break;
        }
        
        // 处理不同类型的测量
        switch (measurement.type) {
            case TimestampedMeasurement::IMU: {
                auto imu_data = std::static_pointer_cast<underwater_nav_msgs::msg::ImuData>(measurement.data);
                processImuMeasurementAtTime(*imu_data, measurement.timestamp);
                break;
            }
            case TimestampedMeasurement::DVL: {
                auto dvl_data = std::static_pointer_cast<underwater_nav_msgs::msg::DvlData>(measurement.data);
                processDvlMeasurementAtTime(*dvl_data, measurement.timestamp);
                break;
            }
            case TimestampedMeasurement::MAGNETOMETER: {
                auto mag_data = std::static_pointer_cast<gtsam::Vector3>(measurement.data);
                processMagnetometerMeasurementAtTime(*mag_data, measurement.timestamp);
                break;
            }
        }
        
        timestamped_queue_.pop();
    }
    
    // 管理时间窗口
    manageTimeWindow(current_time);
}

ContinuousTrajectoryPoint UnderwaterFGO::queryTrajectoryAtTime(double query_time) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 检查缓存
    auto cache_it = trajectory_cache_.find(query_time);
    if (cache_it != trajectory_cache_.end()) {
        return cache_it->second;
    }
    
    // 查找最近的控制点
    auto lower = time_control_points_.lower_bound(query_time);
    auto upper = time_control_points_.upper_bound(query_time);
    
    if (lower == time_control_points_.end() && upper == time_control_points_.begin()) {
        // 没有可用的控制点
        return ContinuousTrajectoryPoint(query_time);
    }
    
    // 根据插值方法选择算法
    if (interpolation_method_ == "gp") {
        return gaussianProcessInterpolation(query_time, time_control_points_);
    } else if (interpolation_method_ == "linear") {
        // 线性插值需要前后两个点
        if (lower != time_control_points_.begin() && upper != time_control_points_.end()) {
            auto before_it = std::prev(lower);
            auto before_point = getTrajectoryPointFromKey(before_it->second);
            auto after_point = getTrajectoryPointFromKey(upper->second);
            return linearInterpolation(query_time, before_point, after_point);
        }
    }
    
    // 默认返回最近的点
    if (lower != time_control_points_.end()) {
        return getTrajectoryPointFromKey(lower->second);
    }
    
    return ContinuousTrajectoryPoint(query_time);
}

ContinuousTrajectoryPoint UnderwaterFGO::gaussianProcessInterpolation(
    double query_time, 
    const std::map<double, gtsam::Key>& control_points) const {
    
    ContinuousTrajectoryPoint result(query_time);
    
    if (control_points.empty()) {
        return result;
    }
    
    // 选择查询时间附近的控制点
    std::vector<std::pair<double, gtsam::Key>> nearby_points;
    for (const auto& cp : control_points) {
        double time_diff = std::abs(cp.first - query_time);
        if (time_diff <= gp_params_.length_scale * 3.0) { // 3-sigma范围
            nearby_points.push_back(cp);
        }
    }
    
    if (nearby_points.empty()) {
        return result;
    }
    
    // 限制控制点数量
    if (nearby_points.size() > static_cast<size_t>(gp_params_.max_control_points)) {
        // 按时间距离排序，选择最近的点
        std::sort(nearby_points.begin(), nearby_points.end(),
                 [query_time](const auto& a, const auto& b) {
                     return std::abs(a.first - query_time) < std::abs(b.first - query_time);
                 });
        nearby_points.resize(gp_params_.max_control_points);
    }
    
    // 构建高斯过程回归
    size_t n = nearby_points.size();
    gtsam::Matrix K(n, n);  // 核矩阵
    gtsam::Vector k(n);     // 查询点到控制点的核向量
    
    // 计算核矩阵和核向量
    for (size_t i = 0; i < n; ++i) {
        k(i) = gaussianProcessKernel(query_time, nearby_points[i].first);
        for (size_t j = 0; j < n; ++j) {
            K(i, j) = gaussianProcessKernel(nearby_points[i].first, nearby_points[j].first);
            if (i == j) {
                K(i, j) += gp_params_.noise_variance; // 添加噪声项
            }
        }
    }
    
    // 求解权重 w = K^(-1) * k
    gtsam::Vector weights = K.ldlt().solve(k);
    
    // 加权插值
    gtsam::Pose3 interpolated_pose = gtsam::Pose3::identity();
    gtsam::Vector3 interpolated_velocity = gtsam::Vector3::Zero();
    gtsam::imuBias::ConstantBias interpolated_bias;
    double total_weight = 0.0;
    
    for (size_t i = 0; i < n; ++i) {
        if (weights(i) > 1e-6) { // 忽略很小的权重
            auto traj_point = getTrajectoryPointFromKey(nearby_points[i].second);
            
            // 位姿插值 (简化处理，实际应使用流形插值)
            if (total_weight == 0.0) {
                interpolated_pose = traj_point.pose;
                interpolated_velocity = traj_point.velocity;
                interpolated_bias = traj_point.bias;
            } else {
                // 简化的加权平均 (实际应使用更复杂的流形插值)
                auto translation = interpolated_pose.translation() * total_weight + 
                                 traj_point.pose.translation() * weights(i);
                translation /= (total_weight + weights(i));
                
                interpolated_velocity = (interpolated_velocity * total_weight + 
                                       traj_point.velocity * weights(i)) / (total_weight + weights(i));
            }
            
            total_weight += weights(i);
        }
    }
    
    result.pose = interpolated_pose;
    result.velocity = interpolated_velocity;
    result.bias = interpolated_bias;
    result.confidence = std::min(1.0, total_weight);
    
    return result;
}

ContinuousTrajectoryPoint UnderwaterFGO::linearInterpolation(
    double query_time,
    const ContinuousTrajectoryPoint& before_point,
    const ContinuousTrajectoryPoint& after_point) const {
    
    ContinuousTrajectoryPoint result(query_time);
    
    double dt = after_point.timestamp - before_point.timestamp;
    if (dt <= 1e-6) {
        return before_point;
    }
    
    double alpha = (query_time - before_point.timestamp) / dt;
    alpha = std::max(0.0, std::min(1.0, alpha)); // 限制在[0,1]范围内
    
    // 位置线性插值
    gtsam::Point3 pos1 = before_point.pose.translation();
    gtsam::Point3 pos2 = after_point.pose.translation();
    gtsam::Point3 interpolated_pos = pos1 + alpha * (pos2 - pos1);
    
    // 旋转球面线性插值 (SLERP)
    gtsam::Rot3 rot1 = before_point.pose.rotation();
    gtsam::Rot3 rot2 = after_point.pose.rotation();
    gtsam::Rot3 interpolated_rot = rot1.slerp(alpha, rot2);
    
    result.pose = gtsam::Pose3(interpolated_rot, interpolated_pos);
    
    // 速度线性插值
    result.velocity = before_point.velocity + alpha * (after_point.velocity - before_point.velocity);
    
    // 偏差线性插值
    gtsam::Vector6 bias1 = before_point.bias.vector();
    gtsam::Vector6 bias2 = after_point.bias.vector();
    gtsam::Vector6 interpolated_bias_vec = bias1 + alpha * (bias2 - bias1);
    result.bias = gtsam::imuBias::ConstantBias(interpolated_bias_vec);
    
    result.confidence = 1.0 - std::abs(alpha - 0.5) * 2.0; // 中间点置信度最高
    
    return result;
}

double UnderwaterFGO::gaussianProcessKernel(double t1, double t2) const {
    double dt = t1 - t2;
    double squared_distance = dt * dt;
    return gp_params_.signal_variance * 
           std::exp(-0.5 * squared_distance / (gp_params_.length_scale * gp_params_.length_scale));
}

void UnderwaterFGO::manageTimeWindow(double current_time) {
    // 移除超出时间窗口的控制点
    auto cutoff_time = current_time - time_window_size_;
    
    auto it = time_control_points_.begin();
    while (it != time_control_points_.end()) {
        if (it->first < cutoff_time) {
            // 从轨迹缓存中移除
            trajectory_cache_.erase(it->first);
            it = time_control_points_.erase(it);
        } else {
            ++it;
        }
    }
}

gtsam::Key UnderwaterFGO::createTimeControlPoint(double timestamp) {
    gtsam::Key key = gtsam::Symbol('T', key_counter_++);
    time_control_points_[timestamp] = key;
    return key;
}

// 辅助方法：从键值获取轨迹点
ContinuousTrajectoryPoint UnderwaterFGO::getTrajectoryPointFromKey(gtsam::Key key) const {
    // 这里需要从当前值中提取轨迹点信息
    // 简化实现，实际需要根据键值类型进行不同处理
    ContinuousTrajectoryPoint point;

    if (current_values_.exists(key)) {
        // 根据键值类型提取不同信息
        char symbol = gtsam::Symbol(key).chr();
        if (symbol == 'X') {
            point.pose = current_values_.at<gtsam::Pose3>(key);
        } else if (symbol == 'V') {
            point.velocity = current_values_.at<gtsam::Vector3>(key);
        } else if (symbol == 'B') {
            point.bias = current_values_.at<gtsam::imuBias::ConstantBias>(key);
        }
    }

    return point;
}

// 时间中心因子添加方法
void UnderwaterFGO::addDvlVelocityFactor(double timestamp, const gtsam::Vector3& velocity) {
    // 查找最近的控制点
    gtsam::Key control_point = getOrCreateControlPoint(timestamp);

    // 创建DVL速度因子
    // 这里需要实现时间插值的DVL因子
    if (dvl_noise_model_) {
        // 简化实现：直接添加到最近的控制点
        char symbol = gtsam::Symbol(control_point).chr();
        if (symbol == 'X') {
            // 假设控制点是位姿，需要对应的速度键
            uint64_t index = gtsam::Symbol(control_point).index();
            gtsam::Key velocity_key = gtsam::Symbol('V', index);

            auto dvl_factor = boost::make_shared<DvlFactor>(
                control_point, velocity_key, velocity, dvl_noise_model_);
            graph_.add(dvl_factor);
        }
    }
}

void UnderwaterFGO::addMagnetometerFactor(double timestamp, const gtsam::Vector3& magnetic_field) {
    // 查找最近的控制点
    gtsam::Key control_point = getOrCreateControlPoint(timestamp);

    // 创建磁力计因子
    if (magnetometer_noise_model_) {
        auto mag_factor = boost::make_shared<MagnetometerFactor>(
            control_point, magnetic_field, magnetic_declination_, magnetometer_noise_model_);
        graph_.add(mag_factor);
    }
}

} // namespace factor_graph_optimizer
