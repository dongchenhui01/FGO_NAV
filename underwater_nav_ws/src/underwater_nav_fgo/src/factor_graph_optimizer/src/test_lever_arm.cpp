#include <iostream>
#include <iomanip>
#include "factor_graph_optimizer/lever_arm_compensation.hpp"

using namespace factor_graph_optimizer;

int main() {
    std::cout << "🔧 杆臂误差补偿测试程序" << std::endl;
    std::cout << "=========================" << std::endl;
    
    // 使用您提供的杆臂数据
    gtsam::Vector3 imu_to_gps(1.4, 0.18, -0.71);  // GPS在IMU前方1.4m, 右侧0.18m, 上方0.71m
    gtsam::Vector3 imu_to_dvl(1.4, 0.18, 0.45);   // DVL在IMU前方1.4m, 右侧0.18m, 下方0.45m
    
    std::cout << "📐 杆臂向量配置:" << std::endl;
    std::cout << "  IMU到GPS: [" << imu_to_gps.x() << ", " << imu_to_gps.y() << ", " << imu_to_gps.z() << "] 米" << std::endl;
    std::cout << "  IMU到DVL: [" << imu_to_dvl.x() << ", " << imu_to_dvl.y() << ", " << imu_to_dvl.z() << "] 米" << std::endl;
    std::cout << std::endl;
    
    // 创建杆臂补偿器
    LeverArmCompensation compensator(imu_to_gps, imu_to_dvl);
    
    // 测试场景：载体在水平面上，航向角为30度
    double roll = 0.0, pitch = 0.0, yaw = 30.0 * M_PI / 180.0;  // 30度航向角
    gtsam::Rot3 attitude = gtsam::Rot3::RzRyRx(yaw, pitch, roll);
    
    std::cout << "🧭 测试场景:" << std::endl;
    std::cout << "  载体姿态: 横滚=" << roll*180/M_PI << "°, 俯仰=" << pitch*180/M_PI << "°, 航向=" << yaw*180/M_PI << "°" << std::endl;
    std::cout << std::endl;
    
    // 测试GPS位置补偿
    std::cout << "📍 GPS位置补偿测试:" << std::endl;
    gtsam::Point3 gps_position(100.0, 200.0, -10.0);  // GPS测量位置 (ENU坐标系)
    gtsam::Point3 imu_position = compensator.compensateGpsPosition(gps_position, attitude);
    
    std::cout << "  GPS测量位置: [" << std::fixed << std::setprecision(3) 
              << gps_position.x() << ", " << gps_position.y() << ", " << gps_position.z() << "] 米" << std::endl;
    std::cout << "  IMU实际位置: [" << std::fixed << std::setprecision(3)
              << imu_position.x() << ", " << imu_position.y() << ", " << imu_position.z() << "] 米" << std::endl;
    
    gtsam::Vector3 position_correction = gps_position - imu_position;
    std::cout << "  位置修正量:   [" << std::fixed << std::setprecision(3)
              << position_correction.x() << ", " << position_correction.y() << ", " << position_correction.z() << "] 米" << std::endl;
    std::cout << "  修正距离:     " << position_correction.norm() << " 米" << std::endl;
    std::cout << std::endl;
    
    std::cout << "✅ 杆臂补偿测试完成！" << std::endl;
    std::cout << "💡 杆臂误差补偿已集成到时间中心因子图优化系统中。" << std::endl;
    
    return 0;
}
