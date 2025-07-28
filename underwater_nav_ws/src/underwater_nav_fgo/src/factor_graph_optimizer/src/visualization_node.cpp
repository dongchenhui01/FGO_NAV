#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "underwater_nav_msgs/msg/imu_data.hpp"
#include "underwater_nav_msgs/msg/dvl_data.hpp"
#include "underwater_nav_msgs/msg/navigation_state.hpp"
#include "factor_graph_optimizer/underwater_fgo.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class VisualizationNode : public rclcpp::Node {
public:
    VisualizationNode() : Node("underwater_fgo_visualizer") {
        // 初始化发布者
        trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("/underwater_trajectory", 10);
        control_points_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/control_points", 10);
        gp_interpolation_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/gp_interpolation", 10);
        factor_graph_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/factor_graph", 10);
        sensor_data_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/sensor_data", 10);
        confidence_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/interpolation_confidence", 10);
        
        // 订阅者
        imu_sub_ = this->create_subscription<underwater_nav_msgs::msg::ImuData>(
            "/imu_data", 10, std::bind(&VisualizationNode::imuCallback, this, std::placeholders::_1));
        dvl_sub_ = this->create_subscription<underwater_nav_msgs::msg::DvlData>(
            "/dvl_data", 10, std::bind(&VisualizationNode::dvlCallback, this, std::placeholders::_1));
        nav_state_sub_ = this->create_subscription<underwater_nav_msgs::msg::NavigationState>(
            "/navigation_state", 10, std::bind(&VisualizationNode::navStateCallback, this, std::placeholders::_1));
        
        // TF广播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // 初始化FGO系统
        fgo_ = std::make_unique<factor_graph_optimizer::UnderwaterFGO>();
        fgo_->enableTimeCentricMode(true, 5.0, "gp"); // 5秒时间窗口，高斯过程插值
        
        // 定时器用于可视化更新
        visualization_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&VisualizationNode::visualizationCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "🎨 水下导航可视化节点启动完成");
        RCLCPP_INFO(this->get_logger(), "📊 实时可视化以下内容:");
        RCLCPP_INFO(this->get_logger(), "   • 轨迹路径 (/underwater_trajectory)");
        RCLCPP_INFO(this->get_logger(), "   • 控制点 (/control_points)");
        RCLCPP_INFO(this->get_logger(), "   • 高斯过程插值 (/gp_interpolation)");
        RCLCPP_INFO(this->get_logger(), "   • 因子图结构 (/factor_graph)");
        RCLCPP_INFO(this->get_logger(), "   • 传感器数据 (/sensor_data)");
        RCLCPP_INFO(this->get_logger(), "   • 插值置信度 (/interpolation_confidence)");

        // 初始化时间变量
        last_update_time_ = this->now();
        last_status_time_ = this->now();
    }

private:
    void imuCallback(const underwater_nav_msgs::msg::ImuData::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "📱 IMU数据: acc=[%.3f,%.3f,%.3f] gyro=[%.3f,%.3f,%.3f]",
                   msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
                   msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

        // 处理IMU数据 - 使用当前时间作为时间戳
        double timestamp = this->now().seconds();
        // 注意：这里不能直接调用私有方法，需要通过公共接口

        // 可视化IMU数据
        auto current_stamp = this->now();
        visualizeSensorData("imu", current_stamp,
                           {msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z});

        last_update_time_ = this->now();
    }
    
    void dvlCallback(const underwater_nav_msgs::msg::DvlData::SharedPtr msg) {
        if (!msg->is_valid) return;

        RCLCPP_INFO(this->get_logger(), "🌊 DVL数据: vel=[%.3f,%.3f,%.3f] 有效=%s",
                   msg->velocity.x, msg->velocity.y, msg->velocity.z,
                   msg->is_valid ? "是" : "否");

        // 处理DVL数据 - 使用当前时间作为时间戳
        double timestamp = this->now().seconds();
        // 注意：这里不能直接调用私有方法，需要通过公共接口

        // 可视化DVL数据
        auto current_stamp = this->now();
        visualizeSensorData("dvl", current_stamp,
                           {msg->velocity.x, msg->velocity.y, msg->velocity.z});

        last_update_time_ = this->now();
    }
    
    void navStateCallback(const underwater_nav_msgs::msg::NavigationState::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "🧭 导航状态: pos=[%.3f,%.3f,%.3f] vel=[%.3f,%.3f,%.3f]",
                   msg->position.x, msg->position.y, msg->position.z,
                   msg->velocity.x, msg->velocity.y, msg->velocity.z);

        // 更新轨迹
        updateTrajectory(msg);

        // 广播TF
        broadcastTransform(msg);

        last_update_time_ = this->now();
    }
    
    void visualizationCallback() {
        if (!fgo_) return;

        static int counter = 0;
        counter++;

        // 1. 可视化控制点
        visualizeControlPoints();

        // 2. 可视化高斯过程插值
        visualizeGaussianProcessInterpolation(counter * 0.1);

        // 3. 可视化因子图结构
        visualizeFactorGraph();

        // 4. 可视化插值置信度
        visualizeInterpolationConfidence(counter * 0.1);

        // 输出算法状态 (每10次回调输出一次)
        if (counter % 10 == 0) {
            printAlgorithmStatus();
        }
    }
    
    void visualizeControlPoints() {
        visualization_msgs::msg::MarkerArray marker_array;
        
        // 获取控制点数据（这里需要从FGO系统获取）
        // 简化实现：创建示例控制点
        for (int i = 0; i < 5; ++i) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->now();
            marker.ns = "control_points";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.position.x = i * 2.0;
            marker.pose.position.y = sin(i * 0.5) * 3.0;
            marker.pose.position.z = -i * 0.5;
            marker.pose.orientation.w = 1.0;
            
            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 0.3;
            
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.8;
            
            marker_array.markers.push_back(marker);
        }
        
        control_points_pub_->publish(marker_array);
    }
    
    void visualizeGaussianProcessInterpolation(double timestamp) {
        visualization_msgs::msg::MarkerArray marker_array;
        
        // 可视化高斯过程插值结果
        for (int i = 0; i < 20; ++i) {
            double query_time = timestamp - 2.0 + i * 0.2;
            
            // 查询轨迹点（这里需要调用FGO的查询方法）
            auto traj_point = fgo_->queryTrajectoryAtTime(query_time);
            
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->now();
            marker.ns = "gp_interpolation";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            // 使用插值结果设置位置
            auto position = traj_point.pose.translation();
            marker.pose.position.x = position.x();
            marker.pose.position.y = position.y();
            marker.pose.position.z = position.z();
            
            auto rotation = traj_point.pose.rotation().toQuaternion();
            marker.pose.orientation.x = rotation.x();
            marker.pose.orientation.y = rotation.y();
            marker.pose.orientation.z = rotation.z();
            marker.pose.orientation.w = rotation.w();
            
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            
            // 颜色表示置信度
            double confidence = traj_point.confidence;
            marker.color.r = 1.0 - confidence;
            marker.color.g = confidence;
            marker.color.b = 0.0;
            marker.color.a = 0.6;
            
            marker_array.markers.push_back(marker);
        }
        
        gp_interpolation_pub_->publish(marker_array);
    }
    
    void visualizeFactorGraph() {
        visualization_msgs::msg::MarkerArray marker_array;
        
        // 可视化因子图的节点和边
        // 这里需要从FGO系统获取图结构信息
        
        factor_graph_pub_->publish(marker_array);
    }
    
    void visualizeInterpolationConfidence(double timestamp) {
        std_msgs::msg::Float64MultiArray confidence_msg;
        
        // 计算不同时间点的插值置信度
        for (int i = 0; i < 10; ++i) {
            double query_time = timestamp - 1.0 + i * 0.2;
            auto traj_point = fgo_->queryTrajectoryAtTime(query_time);
            confidence_msg.data.push_back(traj_point.confidence);
        }
        
        confidence_pub_->publish(confidence_msg);
    }
    
    void visualizeSensorData(const std::string& sensor_type,
                           const rclcpp::Time& stamp,
                           const std::vector<double>& data) {
        RCLCPP_INFO(this->get_logger(), "📊 %s传感器数据可视化: [%.3f,%.3f,%.3f]",
                   sensor_type.c_str(), data[0], data[1], data[2]);
    }
    
    void updateTrajectory(const underwater_nav_msgs::msg::NavigationState::SharedPtr msg) {
        // 添加到轨迹路径
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map";
        pose_stamped.header.stamp = msg->stamp;
        pose_stamped.pose.position = msg->position;
        pose_stamped.pose.orientation = msg->orientation;

        trajectory_path_.header.frame_id = "map";
        trajectory_path_.header.stamp = this->now();
        trajectory_path_.poses.push_back(pose_stamped);

        // 限制轨迹长度
        if (trajectory_path_.poses.size() > 1000) {
            trajectory_path_.poses.erase(trajectory_path_.poses.begin());
        }

        trajectory_pub_->publish(trajectory_path_);
    }
    
    void broadcastTransform(const underwater_nav_msgs::msg::NavigationState::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "base_link";

        transform.transform.translation.x = msg->position.x;
        transform.transform.translation.y = msg->position.y;
        transform.transform.translation.z = msg->position.z;
        transform.transform.rotation = msg->orientation;

        tf_broadcaster_->sendTransform(transform);
    }
    
    void printAlgorithmStatus() {
        RCLCPP_INFO(this->get_logger(), " ");
        RCLCPP_INFO(this->get_logger(), "🔄 ========== 时间中心因子图算法状态 ==========");
        RCLCPP_INFO(this->get_logger(), "⏰ 当前时间: %.3f", this->now().seconds());
        RCLCPP_INFO(this->get_logger(), "🎯 时间窗口: 5.0秒");
        RCLCPP_INFO(this->get_logger(), "🧮 插值方法: 高斯过程回归");
        RCLCPP_INFO(this->get_logger(), "📈 轨迹点数: %zu", trajectory_path_.poses.size());
        RCLCPP_INFO(this->get_logger(), "🔗 因子图节点数: [从FGO获取]");
        RCLCPP_INFO(this->get_logger(), "📊 优化迭代次数: [从FGO获取]");
        RCLCPP_INFO(this->get_logger(), "✨ 平均插值置信度: [计算中]");
        RCLCPP_INFO(this->get_logger(), "===============================================");
        RCLCPP_INFO(this->get_logger(), " ");
    }

private:
    // 发布者
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr control_points_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr gp_interpolation_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr factor_graph_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sensor_data_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr confidence_pub_;
    
    // 订阅者
    rclcpp::Subscription<underwater_nav_msgs::msg::ImuData>::SharedPtr imu_sub_;
    rclcpp::Subscription<underwater_nav_msgs::msg::DvlData>::SharedPtr dvl_sub_;
    rclcpp::Subscription<underwater_nav_msgs::msg::NavigationState>::SharedPtr nav_state_sub_;
    
    // TF广播器
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // FGO系统
    std::unique_ptr<factor_graph_optimizer::UnderwaterFGO> fgo_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr visualization_timer_;
    
    // 数据存储
    nav_msgs::msg::Path trajectory_path_;
    rclcpp::Time last_update_time_;
    rclcpp::Time last_status_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisualizationNode>();
    
    RCLCPP_INFO(node->get_logger(), "🚀 启动水下导航可视化系统...");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
