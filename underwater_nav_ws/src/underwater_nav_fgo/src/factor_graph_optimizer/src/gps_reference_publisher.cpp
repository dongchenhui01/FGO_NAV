#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class GpsReferencePublisher : public rclcpp::Node {
public:
    GpsReferencePublisher() : Node("gps_reference_publisher") {
        // 创建发布器
        gps_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/gps_reference_path", 10);
        gps_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/gps_reference_pose", 10);
        
        // 订阅GPS数据 (使用Float64MultiArray临时接收GPS数据)
        gps_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/gps_reference_array", 10,
            std::bind(&GpsReferencePublisher::gpsCallback, this, std::placeholders::_1));
        
        // 初始化路径消息
        gps_path_.header.frame_id = "map";
        
        // 参考点设置（用于相对坐标转换）
        reference_set_ = false;
        reference_east_ = 0.0;
        reference_north_ = 0.0;
        reference_up_ = 0.0;
        
        RCLCPP_INFO(this->get_logger(), "GPS参考轨迹发布器启动");
    }

private:
    void gpsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        // GPS数据格式: [east, north, up, position_quality]
        if (msg->data.size() < 4) {
            return; // 数据不完整
        }

        double gps_east = msg->data[0];
        double gps_north = msg->data[1];
        double gps_up = msg->data[2];
        int position_quality = static_cast<int>(msg->data[3]);

        // 检查GPS数据有效性
        if (position_quality < 1 ||
            (std::abs(gps_east) < 1e-6 && std::abs(gps_north) < 1e-6)) {
            return; // GPS数据无效，跳过
        }

        // 设置参考点（第一个有效GPS点）
        if (!reference_set_) {
            reference_east_ = gps_east;
            reference_north_ = gps_north;
            reference_up_ = gps_up;
            reference_set_ = true;

            RCLCPP_INFO(this->get_logger(),
                       "设置GPS参考点: East=%.3f, North=%.3f, Up=%.3f",
                       reference_east_, reference_north_, reference_up_);
        }

        // 计算相对位置
        double rel_x = gps_east - reference_east_;
        double rel_y = gps_north - reference_north_;
        double rel_z = gps_up - reference_up_;

        // 创建位姿消息
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "map";
        
        pose_msg.pose.position.x = rel_x;
        pose_msg.pose.position.y = rel_y;
        pose_msg.pose.position.z = rel_z;
        
        // 设置方向（简化为单位四元数）
        pose_msg.pose.orientation.x = 0.0;
        pose_msg.pose.orientation.y = 0.0;
        pose_msg.pose.orientation.z = 0.0;
        pose_msg.pose.orientation.w = 1.0;
        
        // 发布当前位姿
        gps_pose_pub_->publish(pose_msg);
        
        // 添加到路径
        gps_path_.poses.push_back(pose_msg);
        gps_path_.header.stamp = this->now();
        
        // 限制路径长度（保留最近1000个点）
        if (gps_path_.poses.size() > 1000) {
            gps_path_.poses.erase(gps_path_.poses.begin());
        }
        
        // 发布路径
        gps_path_pub_->publish(gps_path_);
        
        // 定期输出GPS状态
        static int counter = 0;
        if (++counter % 50 == 0) {
            RCLCPP_INFO(this->get_logger(),
                       "📍 GPS参考: 相对位置(%.2f, %.2f, %.2f) 质量=%d 轨迹点数=%zu",
                       rel_x, rel_y, rel_z, position_quality, gps_path_.poses.size());
        }
    }
    
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr gps_path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr gps_pose_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gps_sub_;
    
    nav_msgs::msg::Path gps_path_;
    
    bool reference_set_;
    double reference_east_;
    double reference_north_;
    double reference_up_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GpsReferencePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
