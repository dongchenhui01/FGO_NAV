#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/Vector.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <Eigen/Dense>

namespace factor_graph_optimizer {

/**
 * @brief 杆臂误差补偿类
 * 
 * 处理不同传感器之间的空间偏移导致的测量误差
 * 基于严格的运动学模型进行补偿
 */
class LeverArmCompensation {
public:
    /**
     * @brief 构造函数
     * @param imu_to_gps IMU到GPS的杆臂向量 (载体坐标系)
     * @param imu_to_dvl IMU到DVL的杆臂向量 (载体坐标系)
     */
    LeverArmCompensation(const gtsam::Vector3& imu_to_gps, 
                        const gtsam::Vector3& imu_to_dvl);

    /**
     * @brief 设置杆臂向量
     */
    void setLeverArms(const gtsam::Vector3& imu_to_gps, 
                     const gtsam::Vector3& imu_to_dvl);

    /**
     * @brief GPS位置补偿：从GPS天线位置转换到IMU位置
     * @param gps_position GPS天线测量的位置 (世界坐标系)
     * @param attitude 载体姿态
     * @return 补偿后的IMU位置
     */
    gtsam::Point3 compensateGpsPosition(const gtsam::Point3& gps_position,
                                       const gtsam::Rot3& attitude) const;

    /**
     * @brief GPS速度补偿：从GPS天线速度转换到IMU速度
     * @param gps_velocity GPS天线测量的速度 (世界坐标系)
     * @param attitude 载体姿态
     * @param angular_velocity 载体角速度 (载体坐标系)
     * @return 补偿后的IMU速度
     */
    gtsam::Vector3 compensateGpsVelocity(const gtsam::Vector3& gps_velocity,
                                        const gtsam::Rot3& attitude,
                                        const gtsam::Vector3& angular_velocity) const;

    /**
     * @brief DVL速度补偿：从DVL测量位置的速度转换到IMU速度
     * @param dvl_velocity DVL测量的速度 (载体坐标系)
     * @param angular_velocity 载体角速度 (载体坐标系)
     * @return 补偿后的IMU速度 (载体坐标系)
     */
    gtsam::Vector3 compensateDvlVelocity(const gtsam::Vector3& dvl_velocity,
                                        const gtsam::Vector3& angular_velocity) const;

    /**
     * @brief 反向GPS位置补偿：从IMU位置转换到GPS天线位置
     * @param imu_position IMU位置 (世界坐标系)
     * @param attitude 载体姿态
     * @return GPS天线位置
     */
    gtsam::Point3 projectToGpsPosition(const gtsam::Point3& imu_position,
                                      const gtsam::Rot3& attitude) const;

    /**
     * @brief 反向DVL速度补偿：从IMU速度转换到DVL位置的速度
     * @param imu_velocity IMU速度 (载体坐标系)
     * @param angular_velocity 载体角速度 (载体坐标系)
     * @return DVL位置的速度 (载体坐标系)
     */
    gtsam::Vector3 projectToDvlVelocity(const gtsam::Vector3& imu_velocity,
                                       const gtsam::Vector3& angular_velocity) const;

    /**
     * @brief 计算杆臂效应导致的加速度
     * @param angular_velocity 角速度 (载体坐标系)
     * @param angular_acceleration 角加速度 (载体坐标系)
     * @param lever_arm 杆臂向量 (载体坐标系)
     * @return 杆臂效应加速度 (载体坐标系)
     */
    static gtsam::Vector3 computeLeverArmAcceleration(
        const gtsam::Vector3& angular_velocity,
        const gtsam::Vector3& angular_acceleration,
        const gtsam::Vector3& lever_arm);

    /**
     * @brief 获取杆臂向量
     */
    const gtsam::Vector3& getImuToGpsLeverArm() const { return imu_to_gps_; }
    const gtsam::Vector3& getImuToDvlLeverArm() const { return imu_to_dvl_; }

    /**
     * @brief 验证杆臂向量的合理性
     */
    bool validateLeverArms() const;

    /**
     * @brief 计算杆臂补偿的雅可比矩阵
     * @param attitude 载体姿态
     * @param angular_velocity 角速度
     * @param lever_arm 杆臂向量
     * @return 雅可比矩阵 [3x6] (对位置和姿态的导数)
     */
    static Eigen::Matrix<double, 3, 6> computeLeverArmJacobian(
        const gtsam::Rot3& attitude,
        const gtsam::Vector3& angular_velocity,
        const gtsam::Vector3& lever_arm);

    /**
     * @brief 计算反对称矩阵 (用于叉积运算)
     */
    static Eigen::Matrix3d skewSymmetric(const gtsam::Vector3& v);

private:
    gtsam::Vector3 imu_to_gps_;  ///< IMU到GPS的杆臂向量 (载体坐标系)
    gtsam::Vector3 imu_to_dvl_;  ///< IMU到DVL的杆臂向量 (载体坐标系)
};

/**
 * @brief 杆臂补偿因子
 * 
 * 在因子图中考虑杆臂误差的约束因子
 */
class LeverArmFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3> {
public:
    /**
     * @brief 构造函数
     * @param pose_key 位姿变量的键
     * @param velocity_key 速度变量的键
     * @param measurement 测量值
     * @param lever_arm 杆臂向量
     * @param noise_model 噪声模型
     */
    LeverArmFactor(gtsam::Key pose_key, gtsam::Key velocity_key,
                   const gtsam::Vector3& measurement,
                   const gtsam::Vector3& lever_arm,
                   const gtsam::SharedNoiseModel& noise_model);

    /**
     * @brief 计算误差
     */
    gtsam::Vector evaluateError(const gtsam::Pose3& pose,
                               const gtsam::Vector3& velocity,
                               boost::optional<gtsam::Matrix&> H1 = boost::none,
                               boost::optional<gtsam::Matrix&> H2 = boost::none) const override;

private:
    gtsam::Vector3 measurement_;  ///< 测量值
    gtsam::Vector3 lever_arm_;    ///< 杆臂向量
};

} // namespace factor_graph_optimizer
