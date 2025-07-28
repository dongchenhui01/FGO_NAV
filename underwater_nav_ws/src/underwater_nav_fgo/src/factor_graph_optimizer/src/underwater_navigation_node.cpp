#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // 暂时注释掉

#include "underwater_nav_msgs/msg/imu_data.hpp"
#include "underwater_nav_msgs/msg/dvl_data.hpp"
#include "underwater_nav_msgs/msg/navigation_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_srvs/srv/empty.hpp"

#include "factor_graph_optimizer/underwater_fgo.hpp"

namespace underwater_navigation {

/**
 * @brief 传统水下导航节点
 */
class UnderwaterNavigationNode : public rclcpp::Node {
public:
    UnderwaterNavigationNode() : Node("underwater_navigation_node") {
        // 初始化参数
        initializeParameters();
        
        // 创建优化器
        fgo_ = std::make_unique<factor_graph_optimizer::UnderwaterFGO>();
        
        // 配置优化器参数
        configureOptimizer();
        
        // 创建订阅者
        imu_sub_ = this->create_subscription<underwater_nav_msgs::msg::ImuData>(
            "imu_data", 1000, 
            std::bind(&UnderwaterNavigationNode::imuCallback, this, std::placeholders::_1));
        
        dvl_sub_ = this->create_subscription<underwater_nav_msgs::msg::DvlData>(
            "dvl_data", 1000,
            std::bind(&UnderwaterNavigationNode::dvlCallback, this, std::placeholders::_1));
        
        // 创建发布者
        nav_state_pub_ = this->create_publisher<underwater_nav_msgs::msg::NavigationState>(
            "navigation_state", 10);
        
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "pose", 10);
        
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "velocity", 10);
        
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "trajectory", 10);
        
        // TF广播器
        if (publish_tf_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }
        
        // 创建定时器
        optimization_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / optimization_frequency_)),
            std::bind(&UnderwaterNavigationNode::optimizationCallback, this));
        
        // 初始化路径消息
        trajectory_path_.header.frame_id = odom_frame_;
        
        RCLCPP_INFO(this->get_logger(), "传统水下导航节点初始化完成");
    }

private:
    void initializeParameters() {
        // 声明参数
        this->declare_parameter("optimization_frequency", 20.0);
        this->declare_parameter("publish_tf", true);
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("odom_frame", "odom");
        this->declare_parameter("max_path_length", 1000);
        this->declare_parameter("output_directory", "/tmp/underwater_nav_results");
        
        // 获取参数值
        optimization_frequency_ = this->get_parameter("optimization_frequency").as_double();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();
        base_frame_ = this->get_parameter("base_frame").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        max_path_length_ = this->get_parameter("max_path_length").as_int();
        output_directory_ = this->get_parameter("output_directory").as_string();
    }
    
    void configureOptimizer() {
        // 设置IMU参数
        auto imu_params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD(9.81);
        
        // 加速度计噪声
        imu_params->setAccelerometerCovariance(gtsam::I_3x3 * 0.01 * 0.01);
        imu_params->setGyroscopeCovariance(gtsam::I_3x3 * 0.0017 * 0.0017);
        imu_params->setIntegrationCovariance(gtsam::I_3x3 * 1e-8);
        
        // 偏差参数
        imu_params->setBiasAccCovariance(gtsam::I_3x3 * 0.001 * 0.001);
        imu_params->setBiasOmegaCovariance(gtsam::I_3x3 * 0.0001 * 0.0001);
        imu_params->setBiasAccOmegaInt(gtsam::Matrix::Zero(6, 6));
        
        fgo_->setImuParams(*imu_params);
        
        // 设置DVL噪声模型
        gtsam::Vector3 dvl_sigmas(0.02, 0.02, 0.05); // x, y, z方向的标准差
        auto dvl_noise = gtsam::noiseModel::Diagonal::Sigmas(dvl_sigmas);
        fgo_->setDvlNoiseModel(dvl_noise);
        
        // 设置磁力计参数
        double magnetic_declination = 0.0; // 磁偏角，需要根据实际位置设置
        gtsam::Vector3 mag_sigmas(0.1, 0.1, 0.1);
        auto mag_noise = gtsam::noiseModel::Diagonal::Sigmas(mag_sigmas);
        fgo_->setMagnetometerParams(magnetic_declination, mag_noise);
    }
    
    void imuCallback(const underwater_nav_msgs::msg::ImuData::SharedPtr msg) {
        if (!initialized_) {
            // 收集初始化数据
            initialization_imu_data_.push_back(*msg);
            checkInitialization();
            return;
        }
        
        fgo_->addImuMeasurement(*msg);
        
        // 记录最新数据时间
        last_data_time_ = this->now();
    }
    
    void dvlCallback(const underwater_nav_msgs::msg::DvlData::SharedPtr msg) {
        if (!initialized_) {
            // 收集初始化数据
            initialization_dvl_data_.push_back(*msg);
            checkInitialization();
            return;
        }
        
        fgo_->addDvlMeasurement(*msg);
        
        // 记录最新数据时间
        last_data_time_ = this->now();
    }
    
    void checkInitialization() {
        // 简单的初始化逻辑：收集足够的数据后进行初始化
        if (initialization_imu_data_.size() > 100 && initialization_dvl_data_.size() > 10) {
            performInitialization();
        }
    }
    
    void performInitialization() {
        // 使用收集的数据进行初始化
        gtsam::Pose3 initial_pose = gtsam::Pose3::identity();
        gtsam::Vector3 initial_velocity = gtsam::Vector3::Zero();
        gtsam::imuBias::ConstantBias initial_bias;
        
        if (fgo_->initialize(initial_pose, initial_velocity, initial_bias)) {
            initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "传统导航系统初始化成功");
        } else {
            RCLCPP_ERROR(this->get_logger(), "传统导航系统初始化失败");
        }
    }
    
    void optimizationCallback() {
        if (!initialized_) {
            return;
        }
        
        // 执行优化
        if (fgo_->optimize()) {
            // 发布结果
            publishNavigationState();
            publishPose();
            publishVelocity();
            publishPath();
            
            if (publish_tf_) {
                publishTransform();
            }
            
            // 打印统计信息
            auto stats = fgo_->getOptimizationStats();
            RCLCPP_DEBUG(this->get_logger(), 
                       "优化完成 - 因子数: %zu, 变量数: %zu, 求解时间: %.3fs",
                       stats.num_factors, stats.num_variables, stats.solve_time);
        }
    }
    
    void publishNavigationState() {
        auto nav_msg = fgo_->getNavigationStateMsg();
        nav_msg.stamp = this->now();
        nav_state_pub_->publish(nav_msg);
    }
    
    void publishPose() {
        auto nav_state = fgo_->getCurrentState();
        
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = odom_frame_;
        
        // 转换位姿到ROS消息
        auto pose = nav_state.nav_state.pose();
        pose_msg.pose.position.x = pose.translation().x();
        pose_msg.pose.position.y = pose.translation().y();
        pose_msg.pose.position.z = pose.translation().z();
        
        auto quat = pose.rotation().toQuaternion();
        pose_msg.pose.orientation.w = quat.w();
        pose_msg.pose.orientation.x = quat.x();
        pose_msg.pose.orientation.y = quat.y();
        pose_msg.pose.orientation.z = quat.z();
        
        pose_pub_->publish(pose_msg);
    }
    
    void publishVelocity() {
        auto nav_state = fgo_->getCurrentState();
        
        geometry_msgs::msg::TwistStamped vel_msg;
        vel_msg.header.stamp = this->now();
        vel_msg.header.frame_id = base_frame_;
        
        auto velocity = nav_state.nav_state.velocity();
        vel_msg.twist.linear.x = velocity.x();
        vel_msg.twist.linear.y = velocity.y();
        vel_msg.twist.linear.z = velocity.z();
        
        velocity_pub_->publish(vel_msg);
    }
    
    void publishPath() {
        auto nav_state = fgo_->getCurrentState();
        
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = this->now();
        pose_stamped.header.frame_id = odom_frame_;
        
        auto pose = nav_state.nav_state.pose();
        pose_stamped.pose.position.x = pose.translation().x();
        pose_stamped.pose.position.y = pose.translation().y();
        pose_stamped.pose.position.z = pose.translation().z();
        
        auto quat = pose.rotation().toQuaternion();
        pose_stamped.pose.orientation.w = quat.w();
        pose_stamped.pose.orientation.x = quat.x();
        pose_stamped.pose.orientation.y = quat.y();
        pose_stamped.pose.orientation.z = quat.z();
        
        trajectory_path_.poses.push_back(pose_stamped);
        
        // 限制路径长度
        if (trajectory_path_.poses.size() > static_cast<size_t>(max_path_length_)) {
            trajectory_path_.poses.erase(trajectory_path_.poses.begin());
        }
        
        trajectory_path_.header.stamp = this->now();
        path_pub_->publish(trajectory_path_);
    }
    
    void publishTransform() {
        auto nav_state = fgo_->getCurrentState();
        
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = odom_frame_;
        transform.child_frame_id = base_frame_;
        
        auto pose = nav_state.nav_state.pose();
        transform.transform.translation.x = pose.translation().x();
        transform.transform.translation.y = pose.translation().y();
        transform.transform.translation.z = pose.translation().z();
        
        auto quat = pose.rotation().toQuaternion();
        transform.transform.rotation.w = quat.w();
        transform.transform.rotation.x = quat.x();
        transform.transform.rotation.y = quat.y();
        transform.transform.rotation.z = quat.z();
        
        tf_broadcaster_->sendTransform(transform);
    }

private:
    // 优化器
    std::unique_ptr<factor_graph_optimizer::UnderwaterFGO> fgo_;
    
    // ROS2组件
    rclcpp::Subscription<underwater_nav_msgs::msg::ImuData>::SharedPtr imu_sub_;
    rclcpp::Subscription<underwater_nav_msgs::msg::DvlData>::SharedPtr dvl_sub_;
    
    rclcpp::Publisher<underwater_nav_msgs::msg::NavigationState>::SharedPtr nav_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr optimization_timer_;
    
    // 参数
    double optimization_frequency_;
    bool publish_tf_;
    std::string base_frame_;
    std::string odom_frame_;
    int max_path_length_;
    std::string output_directory_;
    
    // 状态
    bool initialized_ = false;
    nav_msgs::msg::Path trajectory_path_;
    rclcpp::Time last_data_time_;
    
    // 初始化数据
    std::vector<underwater_nav_msgs::msg::ImuData> initialization_imu_data_;
    std::vector<underwater_nav_msgs::msg::DvlData> initialization_dvl_data_;
};

} // namespace underwater_navigation

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<underwater_navigation::UnderwaterNavigationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
