#include "factor_graph_optimizer/dvl_factor.hpp"
#include <gtsam/base/numericalDerivative.h>

namespace factor_graph_optimizer {

gtsam::Vector DvlFactor::evaluateError(const gtsam::Pose3& pose, const gtsam::Vector3& velocity,
                                      boost::optional<gtsam::Matrix&> H1,
                                      boost::optional<gtsam::Matrix&> H2) const {
    
    // 将导航坐标系速度转换到载体坐标系
    gtsam::Matrix3 R_nav_to_body = pose.rotation().matrix().transpose();
    gtsam::Vector3 velocity_body = R_nav_to_body * velocity;
    
    // 考虑DVL安装偏移（如果有）
    gtsam::Vector3 velocity_dvl = body_to_dvl_.rotation().matrix().transpose() * velocity_body;
    
    // 计算误差
    gtsam::Vector3 error = velocity_dvl - measured_velocity_;
    
    // 计算雅可比矩阵
    if (H1) {
        // 对位姿的雅可比矩阵
        gtsam::Matrix36 H_pose = gtsam::Matrix36::Zero();
        
        // 只有旋转部分影响速度变换
        gtsam::Matrix3 skew_velocity = gtsam::skewSymmetric(velocity);
        H_pose.block<3,3>(0,3) = -R_nav_to_body * skew_velocity;
        
        *H1 = H_pose;
    }
    
    if (H2) {
        // 对速度的雅可比矩阵
        *H2 = R_nav_to_body;
    }
    
    return error;
}

void DvlFactor::print(const std::string& s, const gtsam::KeyFormatter& keyFormatter) const {
    std::cout << s << "DvlFactor(" << keyFormatter(this->key1()) << "," 
              << keyFormatter(this->key2()) << ")\n";
    std::cout << "  measured: " << measured_velocity_.transpose() << std::endl;
}

bool DvlFactor::equals(const gtsam::NonlinearFactor& expected, double tol) const {
    const DvlFactor* e = dynamic_cast<const DvlFactor*>(&expected);
    return e != nullptr && Base::equals(*e, tol) &&
           gtsam::equal_with_abs_tol(measured_velocity_, e->measured_velocity_, tol) &&
           body_to_dvl_.equals(e->body_to_dvl_, tol);
}

gtsam::Vector MagnetometerFactor::evaluateError(const gtsam::Pose3& pose,
                                               boost::optional<gtsam::Matrix&> H) const {
    
    // 将参考磁场向量转换到载体坐标系
    gtsam::Matrix3 R_nav_to_body = pose.rotation().matrix().transpose();
    
    // 考虑磁偏角
    gtsam::Rot3 declination_rotation = gtsam::Rot3::RzRyRx(0, 0, magnetic_declination_);
    gtsam::Vector3 reference_field_corrected = declination_rotation * reference_magnetic_field_;
    
    gtsam::Vector3 expected_field_body = R_nav_to_body * reference_field_corrected;
    
    // 归一化磁场向量（只关心方向）
    gtsam::Vector3 measured_normalized = measured_magnetic_field_.normalized();
    gtsam::Vector3 expected_normalized = expected_field_body.normalized();
    
    // 计算误差（使用叉积来计算角度误差）
    gtsam::Vector3 error = measured_normalized.cross(expected_normalized);
    
    // 计算雅可比矩阵
    if (H) {
        gtsam::Matrix36 H_pose = gtsam::Matrix36::Zero();
        
        // 对旋转的雅可比矩阵
        gtsam::Matrix3 skew_expected = gtsam::skewSymmetric(expected_normalized);
        H_pose.block<3,3>(0,3) = -skew_expected * R_nav_to_body;
        
        *H = H_pose;
    }
    
    return error;
}

void MagnetometerFactor::print(const std::string& s, const gtsam::KeyFormatter& keyFormatter) const {
    std::cout << s << "MagnetometerFactor(" << keyFormatter(this->key()) << ")\n";
    std::cout << "  measured: " << measured_magnetic_field_.transpose() << std::endl;
    std::cout << "  declination: " << magnetic_declination_ << " rad" << std::endl;
}

bool MagnetometerFactor::equals(const gtsam::NonlinearFactor& expected, double tol) const {
    const MagnetometerFactor* e = dynamic_cast<const MagnetometerFactor*>(&expected);
    return e != nullptr && Base::equals(*e, tol) &&
           gtsam::equal_with_abs_tol(measured_magnetic_field_, e->measured_magnetic_field_, tol) &&
           std::abs(magnetic_declination_ - e->magnetic_declination_) < tol &&
           gtsam::equal_with_abs_tol(reference_magnetic_field_, e->reference_magnetic_field_, tol);
}

} // namespace factor_graph_optimizer
