#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
 * @brief 时间中心水下导航节点
 * 基于gnssFGO论文的时间中心因子图优化方法
 */
class TimeCentricNavigationNode : public rclcpp::Node {
public:
    TimeCentricNavigationNode() : Node("time_centric_navigation_node") {
        // 初始化参数
        initializeParameters();
        
        // 创建优化器
        fgo_ = std::make_unique<factor_graph_optimizer::UnderwaterFGO>();
        
        // 配置优化器参数
        configureOptimizer();
        
        // 启用时间中心模式
        fgo_->enableTimeCentricMode(true, time_window_size_, interpolation_method_);
        
        // 创建订阅者
        imu_sub_ = this->create_subscription<underwater_nav_msgs::msg::ImuData>(
            "imu_data", 1000, 
            std::bind(&TimeCentricNavigationNode::imuCallback, this, std::placeholders::_1));
        
        dvl_sub_ = this->create_subscription<underwater_nav_msgs::msg::DvlData>(
            "dvl_data", 1000,
            std::bind(&TimeCentricNavigationNode::dvlCallback, this, std::placeholders::_1));
        
        // 创建发布者
        nav_state_pub_ = this->create_publisher<underwater_nav_msgs::msg::NavigationState>(
            "navigation_state", 10);
        
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "pose", 10);
        
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "velocity", 10);
        
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "trajectory", 10);
        
        // 创建服务
        query_service_ = this->create_service<std_srvs::srv::Empty>(
            "query_trajectory", 
            std::bind(&TimeCentricNavigationNode::queryTrajectoryCallback, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        // TF广播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // 创建定时器
        optimization_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / optimization_frequency_)),
            std::bind(&TimeCentricNavigationNode::optimizationCallback, this));
        
        // 初始化路径消息
        trajectory_path_.header.frame_id = "odom";
        
        RCLCPP_INFO(this->get_logger(), "时间中心水下导航节点初始化完成");
        RCLCPP_INFO(this->get_logger(), "时间窗口大小: %.2f秒", time_window_size_);
        RCLCPP_INFO(this->get_logger(), "插值方法: %s", interpolation_method_.c_str());
    }

private:
    void initializeParameters() {
        // 声明参数
        this->declare_parameter("optimization_frequency", 20.0);
        this->declare_parameter("publish_tf", true);
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("odom_frame", "odom");
        this->declare_parameter("max_path_length", 1000);
        
        // 时间中心参数
        this->declare_parameter("time_window_size", 2.0);
        this->declare_parameter("interpolation_method", "gp");
        this->declare_parameter("enable_continuous_query", true);
        this->declare_parameter("query_frequency", 50.0);
        
        // 获取参数值
        optimization_frequency_ = this->get_parameter("optimization_frequency").as_double();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();
        base_frame_ = this->get_parameter("base_frame").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        max_path_length_ = this->get_parameter("max_path_length").as_int();
        
        time_window_size_ = this->get_parameter("time_window_size").as_double();
        interpolation_method_ = this->get_parameter("interpolation_method").as_string();
        enable_continuous_query_ = this->get_parameter("enable_continuous_query").as_bool();
        query_frequency_ = this->get_parameter("query_frequency").as_double();
        
        // 验证插值方法
        if (interpolation_method_ != "linear" && interpolation_method_ != "gp" && interpolation_method_ != "spline") {
            RCLCPP_WARN(this->get_logger(), "未知的插值方法: %s, 使用默认的linear", interpolation_method_.c_str());
            interpolation_method_ = "linear";
        }
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
            RCLCPP_INFO(this->get_logger(), "时间中心导航系统初始化成功");
            
            // 如果启用连续查询，创建查询定时器
            if (enable_continuous_query_) {
                query_timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(static_cast<int>(1000.0 / query_frequency_)),
                    std::bind(&TimeCentricNavigationNode::continuousQueryCallback, this));
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "时间中心导航系统初始化失败");
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
            if (stats.num_timestamped_measurements > 0) {
                RCLCPP_DEBUG(this->get_logger(), 
                           "优化完成 - 时间戳测量: %zu, 时间窗口: %.2fs, 求解时间: %.3fs",
                           stats.num_timestamped_measurements, stats.time_window_size, stats.solve_time);
            }
        }
    }
    
    void continuousQueryCallback() {
        if (!initialized_) return;
        
        // 查询当前时间的轨迹状态
        double current_time = this->now().seconds();
        auto trajectory_point = fgo_->queryTrajectoryAtTime(current_time);
        
        // 发布插值结果
        publishInterpolatedState(trajectory_point);
    }
    
    void queryTrajectoryCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                                std::shared_ptr<std_srvs::srv::Empty::Response>) {
        if (!initialized_) return;
        
        // 演示时间查询功能
        double current_time = this->now().seconds();
        
        // 查询过去1秒的轨迹
        for (double t = current_time - 1.0; t <= current_time; t += 0.1) {
            auto trajectory_point = fgo_->queryTrajectoryAtTime(t);
            
            RCLCPP_INFO(this->get_logger(), 
                       "时间 %.2f: 位置(%.3f, %.3f, %.3f), 置信度: %.3f",
                       t, 
                       trajectory_point.pose.translation().x(),
                       trajectory_point.pose.translation().y(),
                       trajectory_point.pose.translation().z(),
                       trajectory_point.confidence);
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
    
    void publishInterpolatedState(const factor_graph_optimizer::ContinuousTrajectoryPoint& trajectory_point) {
        // 发布插值状态（可以用不同的话题名区分）
        geometry_msgs::msg::PoseStamped interpolated_pose;
        interpolated_pose.header.stamp = this->now();
        interpolated_pose.header.frame_id = odom_frame_;
        
        interpolated_pose.pose.position.x = trajectory_point.pose.translation().x();
        interpolated_pose.pose.position.y = trajectory_point.pose.translation().y();
        interpolated_pose.pose.position.z = trajectory_point.pose.translation().z();
        
        auto quat = trajectory_point.pose.rotation().toQuaternion();
        interpolated_pose.pose.orientation.w = quat.w();
        interpolated_pose.pose.orientation.x = quat.x();
        interpolated_pose.pose.orientation.y = quat.y();
        interpolated_pose.pose.orientation.z = quat.z();
        
        // 这里可以创建专门的插值状态发布者
        // interpolated_pose_pub_->publish(interpolated_pose);
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
    
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr query_service_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr optimization_timer_;
    rclcpp::TimerBase::SharedPtr query_timer_;
    
    // 参数
    double optimization_frequency_;
    bool publish_tf_;
    std::string base_frame_;
    std::string odom_frame_;
    int max_path_length_;
    
    // 时间中心参数
    double time_window_size_;
    std::string interpolation_method_;
    bool enable_continuous_query_;
    double query_frequency_;
    
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
    auto node = std::make_shared<underwater_navigation::TimeCentricNavigationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
