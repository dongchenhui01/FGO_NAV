#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

namespace factor_graph_optimizer {

/**
 * @brief DVL速度因子
 * 
 * 该因子将DVL测量的速度与导航状态中的速度进行约束。
 * DVL测量的是载体坐标系下的速度，需要转换到导航坐标系。
 */
class DvlFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3> {
private:
    gtsam::Vector3 measured_velocity_; ///< DVL测量的载体坐标系速度
    gtsam::Pose3 body_to_dvl_;        ///< 载体到DVL的变换

public:
    GTSAM_MAKE_ALIGNED_OPERATOR_NEW
    
    /**
     * @brief 构造函数
     * @param pose_key 位姿变量的键
     * @param velocity_key 速度变量的键
     * @param measured_velocity DVL测量的速度 (载体坐标系)
     * @param noise_model 噪声模型
     * @param body_to_dvl 载体到DVL的变换
     */
    DvlFactor(gtsam::Key pose_key, gtsam::Key velocity_key,
              const gtsam::Vector3& measured_velocity,
              const gtsam::SharedNoiseModel& noise_model,
              const gtsam::Pose3& body_to_dvl = gtsam::Pose3::identity())
        : NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3>(noise_model, pose_key, velocity_key),
          measured_velocity_(measured_velocity),
          body_to_dvl_(body_to_dvl) {}
    
    /**
     * @brief 计算误差函数
     * @param pose 当前位姿
     * @param velocity 当前速度 (导航坐标系)
     * @param H1 位姿的雅可比矩阵 (可选)
     * @param H2 速度的雅可比矩阵 (可选)
     * @return 误差向量
     */
    gtsam::Vector evaluateError(const gtsam::Pose3& pose, const gtsam::Vector3& velocity,
                               boost::optional<gtsam::Matrix&> H1 = boost::none,
                               boost::optional<gtsam::Matrix&> H2 = boost::none) const override;
    
    /**
     * @brief 克隆因子
     * @return 因子的副本
     */
    gtsam::NonlinearFactor::shared_ptr clone() const override {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new DvlFactor(*this)));
    }
    
    /**
     * @brief 打印因子信息
     * @param s 输出流
     * @param keyFormatter 键格式化器
     */
    void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override;
    
    /**
     * @brief 检查因子相等性
     * @param expected 期望的因子
     * @param tol 容差
     * @return 相等返回true
     */
    bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const override;
    
    /**
     * @brief 获取测量值
     * @return DVL测量的速度
     */
    const gtsam::Vector3& getMeasuredVelocity() const { return measured_velocity_; }
    
    /**
     * @brief 获取载体到DVL的变换
     * @return 变换矩阵
     */
    const gtsam::Pose3& getBodyToDvlTransform() const { return body_to_dvl_; }
};

/**
 * @brief 磁力计航向因子
 * 
 * 该因子使用磁力计测量约束航向角。
 * 考虑磁偏角的影响。
 */
class MagnetometerFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
private:
    gtsam::Vector3 measured_magnetic_field_; ///< 测量的磁场向量 (载体坐标系)
    double magnetic_declination_;            ///< 磁偏角 (弧度)
    gtsam::Vector3 reference_magnetic_field_; ///< 参考磁场向量 (导航坐标系)

public:
    GTSAM_MAKE_ALIGNED_OPERATOR_NEW
    
    /**
     * @brief 构造函数
     * @param pose_key 位姿变量的键
     * @param measured_magnetic_field 测量的磁场向量
     * @param magnetic_declination 磁偏角 (弧度)
     * @param noise_model 噪声模型
     * @param reference_magnetic_field 参考磁场向量 (默认指向磁北)
     */
    MagnetometerFactor(gtsam::Key pose_key,
                      const gtsam::Vector3& measured_magnetic_field,
                      double magnetic_declination,
                      const gtsam::SharedNoiseModel& noise_model,
                      const gtsam::Vector3& reference_magnetic_field = gtsam::Vector3(1, 0, 0))
        : NoiseModelFactor1<gtsam::Pose3>(noise_model, pose_key),
          measured_magnetic_field_(measured_magnetic_field),
          magnetic_declination_(magnetic_declination),
          reference_magnetic_field_(reference_magnetic_field) {}
    
    /**
     * @brief 计算误差函数
     * @param pose 当前位姿
     * @param H 位姿的雅可比矩阵 (可选)
     * @return 误差向量
     */
    gtsam::Vector evaluateError(const gtsam::Pose3& pose,
                               boost::optional<gtsam::Matrix&> H = boost::none) const override;
    
    /**
     * @brief 克隆因子
     * @return 因子的副本
     */
    gtsam::NonlinearFactor::shared_ptr clone() const override {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new MagnetometerFactor(*this)));
    }
    
    /**
     * @brief 打印因子信息
     * @param s 输出流
     * @param keyFormatter 键格式化器
     */
    void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override;
    
    /**
     * @brief 检查因子相等性
     * @param expected 期望的因子
     * @param tol 容差
     * @return 相等返回true
     */
    bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const override;
    
    /**
     * @brief 获取测量的磁场向量
     * @return 磁场向量
     */
    const gtsam::Vector3& getMeasuredMagneticField() const { return measured_magnetic_field_; }
    
    /**
     * @brief 获取磁偏角
     * @return 磁偏角 (弧度)
     */
    double getMagneticDeclination() const { return magnetic_declination_; }
};

} // namespace factor_graph_optimizer
