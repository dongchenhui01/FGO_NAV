#include <gtest/gtest.h>
#include <memory>

#include "factor_graph_optimizer/underwater_fgo.hpp"
#include "factor_graph_optimizer/dvl_factor.hpp"

namespace factor_graph_optimizer {

class UnderwaterFGOTest : public ::testing::Test {
protected:
    void SetUp() override {
        fgo_ = std::make_unique<UnderwaterFGO>();
        
        // 设置测试参数
        auto imu_params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD(9.81);
        imu_params->setAccelerometerCovariance(gtsam::I_3x3 * 0.01 * 0.01);
        imu_params->setGyroscopeCovariance(gtsam::I_3x3 * 0.0017 * 0.0017);
        imu_params->setIntegrationCovariance(gtsam::I_3x3 * 1e-8);
        imu_params->setBiasAccCovariance(gtsam::I_3x3 * 0.001 * 0.001);
        imu_params->setBiasOmegaCovariance(gtsam::I_3x3 * 0.0001 * 0.0001);
        imu_params->setBiasAccOmegaInt(gtsam::Matrix::Zero(6, 6));
        
        fgo_->setImuParams(*imu_params);
        
        // 设置DVL噪声模型
        gtsam::Vector3 dvl_sigmas(0.02, 0.02, 0.05);
        auto dvl_noise = gtsam::noiseModel::Diagonal::Sigmas(dvl_sigmas);
        fgo_->setDvlNoiseModel(dvl_noise);
        
        // 设置磁力计参数
        gtsam::Vector3 mag_sigmas(0.1, 0.1, 0.1);
        auto mag_noise = gtsam::noiseModel::Diagonal::Sigmas(mag_sigmas);
        fgo_->setMagnetometerParams(0.0, mag_noise);
    }
    
    void TearDown() override {
        fgo_.reset();
    }
    
    underwater_nav_msgs::msg::ImuData createTestImuData(double timestamp) {
        underwater_nav_msgs::msg::ImuData imu_data;
        
        // 设置时间戳
        imu_data.stamp.sec = static_cast<int32_t>(timestamp);
        imu_data.stamp.nanosec = static_cast<uint32_t>((timestamp - imu_data.stamp.sec) * 1e9);
        
        // 设置测试数据
        imu_data.linear_acceleration.x = 0.1;
        imu_data.linear_acceleration.y = 0.0;
        imu_data.linear_acceleration.z = 9.81;
        
        imu_data.angular_velocity.x = 0.0;
        imu_data.angular_velocity.y = 0.0;
        imu_data.angular_velocity.z = 0.1;
        
        imu_data.magnetic_field.x = 1.0;
        imu_data.magnetic_field.y = 0.0;
        imu_data.magnetic_field.z = 0.5;
        
        imu_data.is_valid = true;
        
        return imu_data;
    }
    
    underwater_nav_msgs::msg::DvlData createTestDvlData(double timestamp) {
        underwater_nav_msgs::msg::DvlData dvl_data;
        
        // 设置时间戳
        dvl_data.stamp.sec = static_cast<int32_t>(timestamp);
        dvl_data.stamp.nanosec = static_cast<uint32_t>((timestamp - dvl_data.stamp.sec) * 1e9);
        
        // 设置测试数据
        dvl_data.velocity.x = 1.0;
        dvl_data.velocity.y = 0.0;
        dvl_data.velocity.z = 0.0;
        
        dvl_data.is_valid = true;
        dvl_data.bottom_track_valid = true;
        dvl_data.water_track_valid = false;
        
        dvl_data.altitude = 10.0;
        dvl_data.num_good_beams = 4;
        dvl_data.frequency = 600000.0;
        dvl_data.beam_angle = 30.0;
        
        return dvl_data;
    }

protected:
    std::unique_ptr<UnderwaterFGO> fgo_;
};

TEST_F(UnderwaterFGOTest, InitializationTest) {
    // 测试初始化
    gtsam::Pose3 initial_pose = gtsam::Pose3::identity();
    gtsam::Vector3 initial_velocity = gtsam::Vector3::Zero();
    gtsam::imuBias::ConstantBias initial_bias;
    
    EXPECT_TRUE(fgo_->initialize(initial_pose, initial_velocity, initial_bias));
    
    // 获取初始状态
    auto state = fgo_->getCurrentState();
    EXPECT_DOUBLE_EQ(state.timestamp, 0.0);
}

TEST_F(UnderwaterFGOTest, ImuDataProcessingTest) {
    // 初始化
    gtsam::Pose3 initial_pose = gtsam::Pose3::identity();
    gtsam::Vector3 initial_velocity = gtsam::Vector3::Zero();
    gtsam::imuBias::ConstantBias initial_bias;
    
    ASSERT_TRUE(fgo_->initialize(initial_pose, initial_velocity, initial_bias));
    
    // 添加IMU数据
    for (int i = 0; i < 10; ++i) {
        auto imu_data = createTestImuData(i * 0.01);
        fgo_->addImuMeasurement(imu_data);
    }
    
    // 执行优化
    EXPECT_TRUE(fgo_->optimize());
    
    // 检查统计信息
    auto stats = fgo_->getOptimizationStats();
    EXPECT_GT(stats.num_factors, 0);
    EXPECT_GT(stats.num_variables, 0);
}

TEST_F(UnderwaterFGOTest, DvlDataProcessingTest) {
    // 初始化
    gtsam::Pose3 initial_pose = gtsam::Pose3::identity();
    gtsam::Vector3 initial_velocity = gtsam::Vector3::Zero();
    gtsam::imuBias::ConstantBias initial_bias;
    
    ASSERT_TRUE(fgo_->initialize(initial_pose, initial_velocity, initial_bias));
    
    // 添加IMU和DVL数据
    for (int i = 0; i < 10; ++i) {
        auto imu_data = createTestImuData(i * 0.01);
        fgo_->addImuMeasurement(imu_data);
        
        if (i % 10 == 0) {  // 每10个IMU数据添加一个DVL数据
            auto dvl_data = createTestDvlData(i * 0.01);
            fgo_->addDvlMeasurement(dvl_data);
        }
    }
    
    // 执行优化
    EXPECT_TRUE(fgo_->optimize());
    
    // 检查结果
    auto nav_msg = fgo_->getNavigationStateMsg();
    EXPECT_TRUE(nav_msg.is_initialized);
}

TEST_F(UnderwaterFGOTest, OptimizationPerformanceTest) {
    // 初始化
    gtsam::Pose3 initial_pose = gtsam::Pose3::identity();
    gtsam::Vector3 initial_velocity = gtsam::Vector3::Zero();
    gtsam::imuBias::ConstantBias initial_bias;
    
    ASSERT_TRUE(fgo_->initialize(initial_pose, initial_velocity, initial_bias));
    
    // 添加大量数据
    auto start_time = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < 1000; ++i) {
        auto imu_data = createTestImuData(i * 0.01);
        fgo_->addImuMeasurement(imu_data);
        
        if (i % 10 == 0) {
            auto dvl_data = createTestDvlData(i * 0.01);
            fgo_->addDvlMeasurement(dvl_data);
        }
        
        if (i % 100 == 0) {
            EXPECT_TRUE(fgo_->optimize());
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    // 检查性能（应该在合理时间内完成）
    EXPECT_LT(duration.count(), 5000);  // 少于5秒
    
    // 检查最终统计
    auto stats = fgo_->getOptimizationStats();
    EXPECT_LT(stats.solve_time, 0.1);  // 单次优化少于100ms
}

// DVL因子测试
class DvlFactorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 创建测试噪声模型
        gtsam::Vector3 sigmas(0.02, 0.02, 0.05);
        noise_model_ = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
    }

protected:
    gtsam::SharedNoiseModel noise_model_;
};

TEST_F(DvlFactorTest, ErrorComputationTest) {
    // 创建测试数据
    gtsam::Key pose_key = gtsam::Symbol('x', 0);
    gtsam::Key velocity_key = gtsam::Symbol('v', 0);
    gtsam::Vector3 measured_velocity(1.0, 0.0, 0.0);
    
    DvlFactor factor(pose_key, velocity_key, measured_velocity, noise_model_);
    
    // 测试位姿和速度
    gtsam::Pose3 test_pose = gtsam::Pose3::identity();
    gtsam::Vector3 test_velocity(1.0, 0.0, 0.0);
    
    // 计算误差
    gtsam::Vector error = factor.evaluateError(test_pose, test_velocity);
    
    // 误差应该接近零（相同的速度）
    EXPECT_NEAR(error.norm(), 0.0, 1e-6);
}

TEST_F(DvlFactorTest, JacobianTest) {
    // 创建测试数据
    gtsam::Key pose_key = gtsam::Symbol('x', 0);
    gtsam::Key velocity_key = gtsam::Symbol('v', 0);
    gtsam::Vector3 measured_velocity(1.0, 0.5, 0.0);
    
    DvlFactor factor(pose_key, velocity_key, measured_velocity, noise_model_);
    
    // 测试位姿和速度
    gtsam::Pose3 test_pose = gtsam::Pose3::identity();
    gtsam::Vector3 test_velocity(1.2, 0.3, 0.1);
    
    // 计算雅可比矩阵
    gtsam::Matrix H1, H2;
    gtsam::Vector error = factor.evaluateError(test_pose, test_velocity, H1, H2);
    
    // 检查雅可比矩阵维度
    EXPECT_EQ(H1.rows(), 3);
    EXPECT_EQ(H1.cols(), 6);
    EXPECT_EQ(H2.rows(), 3);
    EXPECT_EQ(H2.cols(), 3);
    
    // 数值验证雅可比矩阵
    // 这里可以添加数值微分验证
}

} // namespace factor_graph_optimizer

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
