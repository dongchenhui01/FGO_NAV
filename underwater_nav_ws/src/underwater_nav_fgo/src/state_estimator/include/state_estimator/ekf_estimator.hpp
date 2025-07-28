#pragma once

#include <Eigen/Dense>
#include <memory>
#include <chrono>

#include "underwater_nav_msgs/msg/imu_data.hpp"
#include "underwater_nav_msgs/msg/dvl_data.hpp"
#include "underwater_nav_msgs/msg/navigation_state.hpp"

namespace state_estimator {

/**
 * @brief 扩展卡尔曼滤波器状态估计器
 * 
 * 状态向量: [px, py, pz, vx, vy, vz, qw, qx, qy, qz, bax, bay, baz, bgx, bgy, bgz]
 * 维度: 16维 (位置3 + 速度3 + 四元数4 + 加速度偏差3 + 陀螺仪偏差3)
 */
class EkfEstimator {
public:
    static constexpr int STATE_SIZE = 16;
    static constexpr int POS_IDX = 0;    // 位置索引
    static constexpr int VEL_IDX = 3;    // 速度索引
    static constexpr int QUAT_IDX = 6;   // 四元数索引
    static constexpr int BA_IDX = 10;    // 加速度偏差索引
    static constexpr int BG_IDX = 13;    // 陀螺仪偏差索引
    
    using StateVector = Eigen::Matrix<double, STATE_SIZE, 1>;
    using CovarianceMatrix = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>;

public:
    /**
     * @brief 构造函数
     */
    EkfEstimator();
    
    /**
     * @brief 析构函数
     */
    ~EkfEstimator();
    
    /**
     * @brief 初始化滤波器
     * @param initial_state 初始状态
     * @param initial_covariance 初始协方差
     * @return 成功返回true
     */
    bool initialize(const StateVector& initial_state, 
                   const CovarianceMatrix& initial_covariance);
    
    /**
     * @brief IMU预测步骤
     * @param imu_data IMU数据
     * @param dt 时间间隔
     */
    void predictWithImu(const underwater_nav_msgs::msg::ImuData& imu_data, double dt);
    
    /**
     * @brief DVL更新步骤
     * @param dvl_data DVL数据
     */
    void updateWithDvl(const underwater_nav_msgs::msg::DvlData& dvl_data);
    
    /**
     * @brief 磁力计更新步骤
     * @param magnetic_field 磁场测量
     */
    void updateWithMagnetometer(const Eigen::Vector3d& magnetic_field);
    
    /**
     * @brief 获取当前状态
     * @return 状态向量
     */
    StateVector getState() const;
    
    /**
     * @brief 获取当前协方差
     * @return 协方差矩阵
     */
    CovarianceMatrix getCovariance() const;
    
    /**
     * @brief 获取导航状态消息
     * @return ROS2导航状态消息
     */
    underwater_nav_msgs::msg::NavigationState getNavigationStateMsg() const;
    
    /**
     * @brief 设置过程噪声参数
     * @param process_noise 过程噪声协方差
     */
    void setProcessNoise(const CovarianceMatrix& process_noise);
    
    /**
     * @brief 设置IMU噪声参数
     * @param acc_noise 加速度计噪声标准差
     * @param gyr_noise 陀螺仪噪声标准差
     */
    void setImuNoise(double acc_noise, double gyr_noise);
    
    /**
     * @brief 设置DVL噪声参数
     * @param dvl_noise DVL噪声标准差向量
     */
    void setDvlNoise(const Eigen::Vector3d& dvl_noise);
    
    /**
     * @brief 设置磁力计噪声参数
     * @param mag_noise 磁力计噪声标准差
     */
    void setMagnetometerNoise(double mag_noise);
    
    /**
     * @brief 检查滤波器是否已初始化
     * @return 已初始化返回true
     */
    bool isInitialized() const { return initialized_; }

private:
    /**
     * @brief 状态预测函数
     * @param state 当前状态
     * @param imu_data IMU数据
     * @param dt 时间间隔
     * @return 预测状态
     */
    StateVector stateTransitionFunction(const StateVector& state,
                                       const underwater_nav_msgs::msg::ImuData& imu_data,
                                       double dt);
    
    /**
     * @brief 计算状态转移雅可比矩阵
     * @param state 当前状态
     * @param imu_data IMU数据
     * @param dt 时间间隔
     * @return 雅可比矩阵
     */
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> 
    computeStateTransitionJacobian(const StateVector& state,
                                  const underwater_nav_msgs::msg::ImuData& imu_data,
                                  double dt);
    
    /**
     * @brief DVL观测函数
     * @param state 状态向量
     * @return 预测的DVL测量
     */
    Eigen::Vector3d dvlObservationFunction(const StateVector& state);
    
    /**
     * @brief 计算DVL观测雅可比矩阵
     * @param state 状态向量
     * @return 雅可比矩阵
     */
    Eigen::Matrix<double, 3, STATE_SIZE> computeDvlObservationJacobian(const StateVector& state);
    
    /**
     * @brief 磁力计观测函数
     * @param state 状态向量
     * @return 预测的磁场测量
     */
    Eigen::Vector3d magnetometerObservationFunction(const StateVector& state);
    
    /**
     * @brief 计算磁力计观测雅可比矩阵
     * @param state 状态向量
     * @return 雅可比矩阵
     */
    Eigen::Matrix<double, 3, STATE_SIZE> computeMagnetometerObservationJacobian(const StateVector& state);
    
    /**
     * @brief 四元数归一化
     * @param state 状态向量
     */
    void normalizeQuaternion(StateVector& state);
    
    /**
     * @brief 四元数乘法
     * @param q1 四元数1
     * @param q2 四元数2
     * @return 乘积四元数
     */
    Eigen::Vector4d quaternionMultiply(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2);
    
    /**
     * @brief 四元数转旋转矩阵
     * @param q 四元数
     * @return 旋转矩阵
     */
    Eigen::Matrix3d quaternionToRotationMatrix(const Eigen::Vector4d& q);
    
    /**
     * @brief 反对称矩阵
     * @param v 向量
     * @return 反对称矩阵
     */
    Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v);

private:
    // 滤波器状态
    StateVector state_;
    CovarianceMatrix covariance_;
    
    // 噪声参数
    CovarianceMatrix process_noise_;
    double imu_acc_noise_;
    double imu_gyr_noise_;
    Eigen::Vector3d dvl_noise_;
    double magnetometer_noise_;
    
    // 环境参数
    Eigen::Vector3d gravity_;
    Eigen::Vector3d magnetic_field_reference_;
    double magnetic_declination_;
    
    // 状态标志
    bool initialized_;
    
    // 时间管理
    std::chrono::steady_clock::time_point last_update_time_;
};

} // namespace state_estimator
