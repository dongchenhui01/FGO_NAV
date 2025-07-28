#include <rclcpp/rclcpp.hpp>
#include <underwater_nav_msgs/msg/imu_data.hpp>
#include <underwater_nav_msgs/msg/dvl_data.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

class TestCsvPlayer : public rclcpp::Node {
public:
    TestCsvPlayer() : Node("test_csv_player") {
        // 创建发布者
        imu_pub_ = this->create_publisher<underwater_nav_msgs::msg::ImuData>("imu_data", 1000);
        dvl_pub_ = this->create_publisher<underwater_nav_msgs::msg::DvlData>("dvl_data", 1000);
        gps_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("gps_reference_point", 100);
        
        // 创建定时器，每100ms发布一次数据
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TestCsvPlayer::publishData, this));
        
        RCLCPP_INFO(this->get_logger(), "测试CSV播放器启动");
    }

private:
    void publishData() {
        auto now = this->now();
        
        // 发布IMU数据
        underwater_nav_msgs::msg::ImuData imu_msg;
        imu_msg.stamp = now;
        imu_msg.linear_acceleration.x = 0.1;
        imu_msg.linear_acceleration.y = 0.2;
        imu_msg.linear_acceleration.z = 9.8;
        imu_msg.angular_velocity.x = 0.01;
        imu_msg.angular_velocity.y = 0.02;
        imu_msg.angular_velocity.z = 0.03;
        imu_msg.magnetic_field.x = 0.3;
        imu_msg.magnetic_field.y = 0.4;
        imu_msg.magnetic_field.z = 0.5;
        imu_msg.is_valid = true;
        imu_pub_->publish(imu_msg);
        
        // 发布DVL数据
        underwater_nav_msgs::msg::DvlData dvl_msg;
        dvl_msg.stamp = now;
        dvl_msg.velocity.x = 0.5;
        dvl_msg.velocity.y = 0.3;
        dvl_msg.velocity.z = 0.1;
        dvl_msg.is_valid = true;
        dvl_msg.bottom_track_valid = true;
        dvl_msg.altitude = 10.0;
        dvl_msg.num_good_beams = 4;
        dvl_pub_->publish(dvl_msg);
        
        // 发布GPS参考数据
        geometry_msgs::msg::PointStamped gps_msg;
        gps_msg.header.stamp = now;
        gps_msg.header.frame_id = "gps";
        gps_msg.point.x = count_ * 0.1;
        gps_msg.point.y = count_ * 0.2;
        gps_msg.point.z = 0.0;
        gps_pub_->publish(gps_msg);
        
        count_++;
        
        if (count_ % 10 == 0) {
            RCLCPP_INFO(this->get_logger(), "已发布 %d 组数据", count_);
        }
    }
    
    rclcpp::Publisher<underwater_nav_msgs::msg::ImuData>::SharedPtr imu_pub_;
    rclcpp::Publisher<underwater_nav_msgs::msg::DvlData>::SharedPtr dvl_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr gps_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestCsvPlayer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
