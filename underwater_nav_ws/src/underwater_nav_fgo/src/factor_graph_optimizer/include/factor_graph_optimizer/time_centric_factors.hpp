#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <boost/optional.hpp>

namespace factor_graph_optimizer {

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
    bool is_interpolated; // 是否为插值结果
    
    ContinuousTrajectoryPoint() : timestamp(0.0), pose(gtsam::Pose3::identity()),
                                 velocity(gtsam::Vector3::Zero()), acceleration(gtsam::Vector3::Zero()),
                                 confidence(0.0), is_interpolated(false) {}
    
    ContinuousTrajectoryPoint(double t) : timestamp(t), pose(gtsam::Pose3::identity()),
                                         velocity(gtsam::Vector3::Zero()), acceleration(gtsam::Vector3::Zero()),
                                         confidence(1.0), is_interpolated(false) {}
};

/**
 * @brief 连续时间轨迹类 (基于gnssFGO论文完整实现)
 */
class ContinuousTimeTrajectory {
public:
    struct ControlPoint {
        double timestamp;
        gtsam::Pose3 pose;
        gtsam::Vector3 velocity;
        gtsam::Vector3 acceleration;
        gtsam::imuBias::ConstantBias bias;
        gtsam::Matrix covariance;
        
        ControlPoint() : timestamp(0.0), pose(gtsam::Pose3::identity()), 
                        velocity(gtsam::Vector3::Zero()), acceleration(gtsam::Vector3::Zero()) {}
        ControlPoint(double t) : timestamp(t), pose(gtsam::Pose3::identity()), 
                                velocity(gtsam::Vector3::Zero()), acceleration(gtsam::Vector3::Zero()) {}
    };
    
    std::map<double, ControlPoint> control_points;
    std::string interpolation_method = "gp"; // "gp", "spline", "linear"
    
    // 高斯过程参数 (论文第III-B节)
    struct GPParams {
        double length_scale = 0.1;
        double signal_variance = 1.0;
        double noise_variance = 0.01;
        std::string kernel_type = "rbf"; // "rbf", "matern32", "matern52"
    } gp_params;
    
    // 核心算法方法 - 保持完整实现
    ContinuousTrajectoryPoint queryAtTime(double query_time) const;
    ContinuousTrajectoryPoint gaussianProcessInterpolation(double query_time) const;
    ContinuousTrajectoryPoint splineInterpolation(double query_time) const;
    void addControlPoint(double timestamp, const gtsam::Pose3& pose, 
                        const gtsam::Vector3& velocity = gtsam::Vector3::Zero());
    void manageTimeWindow(double current_time, double window_size);
    double rbfKernel(double t1, double t2) const;
    double matern32Kernel(double t1, double t2) const;
    double matern52Kernel(double t1, double t2) const;
};

/**
 * @brief 时间中心IMU因子 (基于论文第IV-C节完整实现)
 */
class TimeCentricIMUFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3> {
private:
    double measurement_time_;
    gtsam::Vector3 acceleration_;
    gtsam::Vector3 angular_velocity_;
    
public:
    TimeCentricIMUFactor(gtsam::Key pose_key, gtsam::Key velocity_key,
                        double measurement_time,
                        const gtsam::Vector3& acceleration,
                        const gtsam::Vector3& angular_velocity,
                        const gtsam::SharedNoiseModel& noise_model);
    
    gtsam::Vector evaluateError(const gtsam::Pose3& pose, const gtsam::Vector3& velocity,
                               boost::optional<gtsam::Matrix&> H1 = boost::none,
                               boost::optional<gtsam::Matrix&> H2 = boost::none) const override;
};

/**
 * @brief 时间中心DVL因子 (基于论文第IV-C节完整实现)
 */
class TimeCentricDVLFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3> {
private:
    double measurement_time_;
    gtsam::Vector3 velocity_measurement_;
    
public:
    TimeCentricDVLFactor(gtsam::Key pose_key, gtsam::Key velocity_key,
                        double measurement_time,
                        const gtsam::Vector3& velocity_measurement,
                        const gtsam::SharedNoiseModel& noise_model);
    
    gtsam::Vector evaluateError(const gtsam::Pose3& pose, const gtsam::Vector3& velocity,
                               boost::optional<gtsam::Matrix&> H1 = boost::none,
                               boost::optional<gtsam::Matrix&> H2 = boost::none) const override;
};

/**
 * @brief 连续时间约束因子 (基于论文第IV-B节完整实现)
 */
class ContinuousTimeConstraintFactor : public gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3> {
private:
    double dt_; // 时间间隔
    
public:
    ContinuousTimeConstraintFactor(gtsam::Key pose1_key, gtsam::Key vel1_key,
                                  gtsam::Key pose2_key, gtsam::Key vel2_key,
                                  double dt,
                                  const gtsam::SharedNoiseModel& noise_model);
    
    gtsam::Vector evaluateError(const gtsam::Pose3& pose1, const gtsam::Vector3& vel1,
                               const gtsam::Pose3& pose2, const gtsam::Vector3& vel2,
                               boost::optional<gtsam::Matrix&> H1 = boost::none,
                               boost::optional<gtsam::Matrix&> H2 = boost::none,
                               boost::optional<gtsam::Matrix&> H3 = boost::none,
                               boost::optional<gtsam::Matrix&> H4 = boost::none) const override;
};

/**
 * @brief 高斯过程时间插值因子 (论文核心算法)
 */
class GaussianProcessInterpolationFactor : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> {
private:
    double query_time_;
    double t1_, t2_; // 控制点时间
    double length_scale_;
    
public:
    GaussianProcessInterpolationFactor(gtsam::Key pose_key, gtsam::Key vel1_key, gtsam::Key vel2_key,
                                      double query_time, double t1, double t2, double length_scale,
                                      const gtsam::SharedNoiseModel& noise_model);
    
    gtsam::Vector evaluateError(const gtsam::Pose3& pose, const gtsam::Vector3& vel1, const gtsam::Vector3& vel2,
                               boost::optional<gtsam::Matrix&> H1 = boost::none,
                               boost::optional<gtsam::Matrix&> H2 = boost::none,
                               boost::optional<gtsam::Matrix&> H3 = boost::none) const override;
};

} // namespace factor_graph_optimizer
