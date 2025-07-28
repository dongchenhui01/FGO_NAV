#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include "underwater_nav_msgs/msg/imu_data.hpp"
#include "underwater_nav_msgs/msg/dvl_data.hpp"

class SimpleCsvTest : public rclcpp::Node {
public:
    SimpleCsvTest() : Node("simple_csv_test") {
        imu_pub_ = this->create_publisher<underwater_nav_msgs::msg::ImuData>("imu_data", 1000);
        dvl_pub_ = this->create_publisher<underwater_nav_msgs::msg::DvlData>("dvl_data", 1000);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 10Hz
            std::bind(&SimpleCsvTest::publishData, this));
            
        loadCsvFile();
        RCLCPP_INFO(this->get_logger(), "简单CSV测试节点启动，数据行数: %zu", data_rows_.size());
    }

private:
    void loadCsvFile() {
        std::ifstream file("/home/dongchenhui/underwater_nav_fgo/data/sensor_fusion_log_20250712_102606.csv");
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开CSV文件");
            return;
        }
        
        std::string line;
        // 跳过表头
        std::getline(file, line);
        
        // 读取前100行数据进行测试
        for (int i = 0; i < 100 && std::getline(file, line); ++i) {
            auto fields = splitCsvLine(line);
            if (fields.size() >= 13) {
                DataRow row;
                try {
                    row.acc_x = std::stod(fields[1]);
                    row.acc_y = std::stod(fields[2]);
                    row.acc_z = std::stod(fields[3]);
                    row.gyr_x = std::stod(fields[4]);
                    row.gyr_y = std::stod(fields[5]);
                    row.gyr_z = std::stod(fields[6]);
                    row.dvl_vx = std::stod(fields[10]);
                    row.dvl_vy = std::stod(fields[11]);
                    row.dvl_vz = std::stod(fields[12]);
                    data_rows_.push_back(row);
                } catch (const std::exception& e) {
                    RCLCPP_WARN(this->get_logger(), "解析第%d行失败: %s", i+2, e.what());
                }
            }
        }
        file.close();
    }
    
    std::vector<std::string> splitCsvLine(const std::string& line) {
        std::vector<std::string> fields;
        std::stringstream ss(line);
        std::string field;
        
        while (std::getline(ss, field, ',')) {
            field.erase(0, field.find_first_not_of(" \t"));
            field.erase(field.find_last_not_of(" \t") + 1);
            fields.push_back(field);
        }
        return fields;
    }
    
    void publishData() {
        if (data_rows_.empty()) return;
        
        auto& row = data_rows_[current_index_ % data_rows_.size()];
        
        // 发布IMU数据
        underwater_nav_msgs::msg::ImuData imu_msg;
        imu_msg.stamp = this->now();
        imu_msg.linear_acceleration.x = row.acc_x;
        imu_msg.linear_acceleration.y = row.acc_y;
        imu_msg.linear_acceleration.z = row.acc_z;
        imu_msg.angular_velocity.x = row.gyr_x;
        imu_msg.angular_velocity.y = row.gyr_y;
        imu_msg.angular_velocity.z = row.gyr_z;
        imu_msg.is_valid = true;
        imu_pub_->publish(imu_msg);
        
        // 发布DVL数据
        underwater_nav_msgs::msg::DvlData dvl_msg;
        dvl_msg.stamp = this->now();
        dvl_msg.velocity.x = row.dvl_vx;
        dvl_msg.velocity.y = row.dvl_vy;
        dvl_msg.velocity.z = row.dvl_vz;
        dvl_msg.is_valid = true;
        dvl_pub_->publish(dvl_msg);
        
        RCLCPP_INFO(this->get_logger(), "发布数据[%zu]: IMU acc=[%.3f,%.3f,%.3f] DVL vel=[%.3f,%.3f,%.3f]", 
                    current_index_, row.acc_x, row.acc_y, row.acc_z, row.dvl_vx, row.dvl_vy, row.dvl_vz);
        
        current_index_++;
    }
    
    struct DataRow {
        double acc_x, acc_y, acc_z;
        double gyr_x, gyr_y, gyr_z;
        double dvl_vx, dvl_vy, dvl_vz;
    };
    
    rclcpp::Publisher<underwater_nav_msgs::msg::ImuData>::SharedPtr imu_pub_;
    rclcpp::Publisher<underwater_nav_msgs::msg::DvlData>::SharedPtr dvl_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<DataRow> data_rows_;
    size_t current_index_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleCsvTest>());
    rclcpp::shutdown();
    return 0;
}
