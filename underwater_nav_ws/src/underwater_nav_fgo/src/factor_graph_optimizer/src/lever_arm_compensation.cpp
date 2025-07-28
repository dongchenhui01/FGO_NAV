#include "factor_graph_optimizer/lever_arm_compensation.hpp"
#include <gtsam/base/numericalDerivative.h>
#include <cmath>

namespace factor_graph_optimizer {

LeverArmCompensation::LeverArmCompensation(const gtsam::Vector3& imu_to_gps, 
                                          const gtsam::Vector3& imu_to_dvl)
    : imu_to_gps_(imu_to_gps), imu_to_dvl_(imu_to_dvl) {
    if (!validateLeverArms()) {
        throw std::invalid_argument("杆臂向量不合理");
    }
}

void LeverArmCompensation::setLeverArms(const gtsam::Vector3& imu_to_gps, 
                                       const gtsam::Vector3& imu_to_dvl) {
    imu_to_gps_ = imu_to_gps;
    imu_to_dvl_ = imu_to_dvl;
    if (!validateLeverArms()) {
        throw std::invalid_argument("杆臂向量不合理");
    }
}

gtsam::Point3 LeverArmCompensation::compensateGpsPosition(
    const gtsam::Point3& gps_position,
    const gtsam::Rot3& attitude) const {
    
    // GPS位置补偿公式: P_imu = P_gps - R * L_arm
    // 其中 R 是从载体坐标系到世界坐标系的旋转矩阵
    gtsam::Point3 lever_arm_world = attitude.rotate(imu_to_gps_);
    return gps_position - lever_arm_world;
}

gtsam::Vector3 LeverArmCompensation::compensateGpsVelocity(
    const gtsam::Vector3& gps_velocity,
    const gtsam::Rot3& attitude,
    const gtsam::Vector3& angular_velocity) const {
    
    // GPS速度补偿公式: V_imu = V_gps - ω × (R * L_arm)
    // 其中 ω 是角速度，R * L_arm 是世界坐标系中的杆臂向量
    gtsam::Vector3 lever_arm_world = attitude.rotate(imu_to_gps_);
    gtsam::Vector3 angular_velocity_world = attitude.rotate(angular_velocity);
    gtsam::Vector3 lever_arm_velocity = angular_velocity_world.cross(lever_arm_world);
    
    return gps_velocity - lever_arm_velocity;
}

gtsam::Vector3 LeverArmCompensation::compensateDvlVelocity(
    const gtsam::Vector3& dvl_velocity,
    const gtsam::Vector3& angular_velocity) const {
    
    // DVL速度补偿公式: V_imu = V_dvl - ω × L_arm
    // 在载体坐标系中进行计算
    gtsam::Vector3 lever_arm_velocity = angular_velocity.cross(imu_to_dvl_);
    return dvl_velocity - lever_arm_velocity;
}

gtsam::Point3 LeverArmCompensation::projectToGpsPosition(
    const gtsam::Point3& imu_position,
    const gtsam::Rot3& attitude) const {
    
    // 反向补偿: P_gps = P_imu + R * L_arm
    gtsam::Point3 lever_arm_world = attitude.rotate(imu_to_gps_);
    return imu_position + lever_arm_world;
}

gtsam::Vector3 LeverArmCompensation::projectToDvlVelocity(
    const gtsam::Vector3& imu_velocity,
    const gtsam::Vector3& angular_velocity) const {
    
    // 反向补偿: V_dvl = V_imu + ω × L_arm
    gtsam::Vector3 lever_arm_velocity = angular_velocity.cross(imu_to_dvl_);
    return imu_velocity + lever_arm_velocity;
}

gtsam::Vector3 LeverArmCompensation::computeLeverArmAcceleration(
    const gtsam::Vector3& angular_velocity,
    const gtsam::Vector3& angular_acceleration,
    const gtsam::Vector3& lever_arm) {
    
    // 杆臂加速度公式: a_lever = α × L_arm + ω × (ω × L_arm)
    // 其中 α 是角加速度，ω 是角速度
    gtsam::Vector3 centripetal_acc = angular_velocity.cross(angular_velocity.cross(lever_arm));
    gtsam::Vector3 tangential_acc = angular_acceleration.cross(lever_arm);
    
    return tangential_acc + centripetal_acc;
}

bool LeverArmCompensation::validateLeverArms() const {
    // 检查杆臂向量的合理性
    const double MAX_LEVER_ARM = 10.0; // 最大杆臂长度 (米)
    
    if (imu_to_gps_.norm() > MAX_LEVER_ARM || imu_to_dvl_.norm() > MAX_LEVER_ARM) {
        return false;
    }
    
    // 检查是否为零向量
    if (imu_to_gps_.norm() < 1e-6 && imu_to_dvl_.norm() < 1e-6) {
        return false; // 至少一个杆臂向量应该非零
    }
    
    return true;
}

Eigen::Matrix<double, 3, 6> LeverArmCompensation::computeLeverArmJacobian(
    const gtsam::Rot3& attitude,
    const gtsam::Vector3& angular_velocity,
    const gtsam::Vector3& lever_arm) {
    
    Eigen::Matrix<double, 3, 6> jacobian = Eigen::Matrix<double, 3, 6>::Zero();
    
    // 对位置的雅可比 (前3列)
    jacobian.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    
    // 对姿态的雅可比 (后3列)
    gtsam::Vector3 lever_arm_world = attitude.rotate(lever_arm);
    jacobian.block<3, 3>(0, 3) = -skewSymmetric(lever_arm_world);
    
    return jacobian;
}

Eigen::Matrix3d LeverArmCompensation::skewSymmetric(const gtsam::Vector3& v) {
    Eigen::Matrix3d skew;
    skew << 0, -v.z(), v.y(),
            v.z(), 0, -v.x(),
            -v.y(), v.x(), 0;
    return skew;
}

// ========== LeverArmFactor 实现 ==========

LeverArmFactor::LeverArmFactor(gtsam::Key pose_key, gtsam::Key velocity_key,
                               const gtsam::Vector3& measurement,
                               const gtsam::Vector3& lever_arm,
                               const gtsam::SharedNoiseModel& noise_model)
    : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3>(noise_model, pose_key, velocity_key),
      measurement_(measurement), lever_arm_(lever_arm) {}

gtsam::Vector LeverArmFactor::evaluateError(
    const gtsam::Pose3& pose,
    const gtsam::Vector3& velocity,
    boost::optional<gtsam::Matrix&> H1,
    boost::optional<gtsam::Matrix&> H2) const {
    
    // 计算预测的测量值 (考虑杆臂效应)
    gtsam::Point3 lever_arm_world = pose.rotation().rotate(lever_arm_);
    gtsam::Point3 predicted_position = pose.translation() + lever_arm_world;
    
    // 计算误差
    gtsam::Vector3 error = measurement_ - predicted_position;
    
    // 计算雅可比矩阵
    if (H1) {
        H1->resize(3, 6);
        H1->block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity(); // 对位置的导数
        H1->block<3, 3>(0, 3) = LeverArmCompensation::skewSymmetric(lever_arm_world); // 对姿态的导数
    }
    
    if (H2) {
        H2->resize(3, 3);
        *H2 = Eigen::Matrix3d::Zero(); // 对速度的导数为零 (位置测量)
    }
    
    return error;
}

} // namespace factor_graph_optimizer
