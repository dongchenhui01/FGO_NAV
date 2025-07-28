#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "underwater_nav_msgs/msg/imu_data.hpp"
#include "underwater_nav_msgs/msg/dvl_data.hpp"
#include "data_preprocessing/csv_reader.hpp"

namespace data_preprocessing {

/**
 * @brief CSV数据播放器节点
 */
class CsvPlayerNode : public rclcpp::Node {
public:
    CsvPlayerNode() : Node("csv_player_node") {
        // 初始化参数
        initializeParameters();
        
        // 创建CSV读取器
        csv_reader_ = std::make_unique<CsvReader>();
        
        // 创建发布者
        imu_pub_ = this->create_publisher<underwater_nav_msgs::msg::ImuData>("imu_data", 1000);
        dvl_pub_ = this->create_publisher<underwater_nav_msgs::msg::DvlData>("dvl_data", 1000);
        
        if (publish_gps_reference_) {
            gps_ref_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("gps_reference_point", 100);
            gps_array_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("gps_reference_array", 100);
        }
        
        // 创建服务
        play_service_ = this->create_service<std_srvs::srv::Empty>(
            "play", std::bind(&CsvPlayerNode::playCallback, this, std::placeholders::_1, std::placeholders::_2));
        
        pause_service_ = this->create_service<std_srvs::srv::Empty>(
            "pause", std::bind(&CsvPlayerNode::pauseCallback, this, std::placeholders::_1, std::placeholders::_2));
        
        reset_service_ = this->create_service<std_srvs::srv::Empty>(
            "reset", std::bind(&CsvPlayerNode::resetCallback, this, std::placeholders::_1, std::placeholders::_2));
        
        // 加载数据文件
        if (!data_file_.empty()) {
            loadDataFile();
        }
        
        // 创建播放定时器
        createPlaybackTimer();
        
        RCLCPP_INFO(this->get_logger(), "CSV播放器节点初始化完成");
        if (!data_file_.empty()) {
            RCLCPP_INFO(this->get_logger(), "数据文件: %s", data_file_.c_str());
            RCLCPP_INFO(this->get_logger(), "播放速率: %.2fx", playback_rate_);
            RCLCPP_INFO(this->get_logger(), "循环播放: %s", loop_playback_ ? "是" : "否");
        }
    }

private:
    void initializeParameters() {
        // 声明参数
        this->declare_parameter("data_file", "");
        this->declare_parameter("playback_rate", 1.0);
        this->declare_parameter("loop_playback", false);
        this->declare_parameter("start_paused", false);
        this->declare_parameter("publish_gps_reference", true);
        this->declare_parameter("time_offset", 0.0);
        
        // 获取参数值
        data_file_ = this->get_parameter("data_file").as_string();
        playback_rate_ = this->get_parameter("playback_rate").as_double();
        loop_playback_ = this->get_parameter("loop_playback").as_bool();
        start_paused_ = this->get_parameter("start_paused").as_bool();
        publish_gps_reference_ = this->get_parameter("publish_gps_reference").as_bool();
        time_offset_ = this->get_parameter("time_offset").as_double();
        
        // 初始化状态
        is_playing_ = !start_paused_;
        current_index_ = 0;
    }
    
    void loadDataFile() {
        if (data_file_.empty()) {
            RCLCPP_WARN(this->get_logger(), "未指定数据文件");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "加载数据文件: %s", data_file_.c_str());
        
        if (!csv_reader_->loadFile(data_file_)) {
            RCLCPP_ERROR(this->get_logger(), "无法加载数据文件: %s", data_file_.c_str());
            return;
        }
        
        data_loaded_ = true;
        auto data_info = csv_reader_->getDataInfo();
        
        RCLCPP_INFO(this->get_logger(), "数据加载完成:");
        RCLCPP_INFO(this->get_logger(), "  总行数: %zu", data_info.total_rows);
        RCLCPP_INFO(this->get_logger(), "  时间范围: %.2f - %.2f 秒", data_info.start_time, data_info.end_time);
        RCLCPP_INFO(this->get_logger(), "  数据时长: %.2f 秒", data_info.duration);
        
        // 设置播放参数
        total_rows_ = data_info.total_rows;
        start_time_ = this->now();
        data_start_time_ = data_info.start_time;
    }
    
    void createPlaybackTimer() {
        // 创建高频定时器用于数据播放
        double timer_frequency = 100.0; // 100Hz
        playback_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / timer_frequency)),
            std::bind(&CsvPlayerNode::playbackCallback, this));
    }
    
    void playbackCallback() {
        if (!data_loaded_ || !is_playing_ || current_index_ >= total_rows_) {
            // 检查是否需要循环播放
            if (loop_playback_ && current_index_ >= total_rows_) {
                resetPlayback();
                return;
            }
            return;
        }
        
        // 计算当前应该播放的时间
        auto current_time = this->now();
        double elapsed_time = (current_time - start_time_).seconds() * playback_rate_;
        double target_data_time = data_start_time_ + elapsed_time + time_offset_;
        
        // 发布所有应该在当前时间之前的数据
        while (current_index_ < total_rows_) {
            auto data_row = csv_reader_->getDataRow(current_index_);
            if (!data_row.has_value()) {
                current_index_++;
                continue;
            }
            
            if (data_row->timestamp > target_data_time) {
                break; // 还没到播放时间
            }
            
            // 发布数据
            publishDataRow(*data_row);
            current_index_++;
        }
        
        // 检查是否播放完成
        if (current_index_ >= total_rows_) {
            if (loop_playback_) {
                RCLCPP_INFO(this->get_logger(), "数据播放完成，开始循环播放");
                resetPlayback();
            } else {
                RCLCPP_INFO(this->get_logger(), "数据播放完成");
                is_playing_ = false;
            }
        }
    }
    
    void publishDataRow(const CsvDataRow& data_row) {
        // 创建时间戳
        builtin_interfaces::msg::Time ros_time;
        ros_time.sec = static_cast<int32_t>(data_row.timestamp);
        ros_time.nanosec = static_cast<uint32_t>((data_row.timestamp - ros_time.sec) * 1e9);
        
        // 发布IMU数据
        if (data_row.has_imu_data) {
            underwater_nav_msgs::msg::ImuData imu_msg;
            imu_msg.stamp = ros_time;
            
            imu_msg.linear_acceleration.x = data_row.acc_x;
            imu_msg.linear_acceleration.y = data_row.acc_y;
            imu_msg.linear_acceleration.z = data_row.acc_z;
            
            imu_msg.angular_velocity.x = data_row.gyr_x;
            imu_msg.angular_velocity.y = data_row.gyr_y;
            imu_msg.angular_velocity.z = data_row.gyr_z;
            
            imu_msg.magnetic_field.x = data_row.mag_x;
            imu_msg.magnetic_field.y = data_row.mag_y;
            imu_msg.magnetic_field.z = data_row.mag_z;
            
            imu_msg.is_valid = true;
            
            imu_pub_->publish(imu_msg);
        }
        
        // 发布DVL数据
        if (data_row.has_dvl_data) {
            underwater_nav_msgs::msg::DvlData dvl_msg;
            dvl_msg.stamp = ros_time;
            
            dvl_msg.velocity.x = data_row.dvl_vx;
            dvl_msg.velocity.y = data_row.dvl_vy;
            dvl_msg.velocity.z = data_row.dvl_vz;
            
            dvl_msg.is_valid = true;
            dvl_msg.bottom_track_valid = true;
            dvl_msg.water_track_valid = false;
            
            // 设置默认值
            dvl_msg.altitude = 10.0;
            dvl_msg.num_good_beams = 4;
            dvl_msg.frequency = 600000.0;
            dvl_msg.beam_angle = 30.0;
            
            dvl_pub_->publish(dvl_msg);
        }
        
        // 发布GPS参考数据
        if (publish_gps_reference_ && data_row.has_gps_data && data_row.gps_pos_qual >= 1) {
            // 发布PointStamped格式
            geometry_msgs::msg::PointStamped gps_msg;
            gps_msg.header.stamp = ros_time;
            gps_msg.header.frame_id = "gps";

            gps_msg.point.x = data_row.gps_east;
            gps_msg.point.y = data_row.gps_north;
            gps_msg.point.z = data_row.gps_up;

            gps_ref_pub_->publish(gps_msg);

            // 发布Float64MultiArray格式给GPS参考发布器
            std_msgs::msg::Float64MultiArray gps_array_msg;
            gps_array_msg.data = {
                data_row.gps_east,
                data_row.gps_north,
                data_row.gps_up,
                static_cast<double>(data_row.gps_pos_qual)
            };

            gps_array_pub_->publish(gps_array_msg);
        }
    }
    
    void resetPlayback() {
        current_index_ = 0;
        start_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "播放重置");
    }
    
    void playCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                     std::shared_ptr<std_srvs::srv::Empty::Response>) {
        if (!data_loaded_) {
            RCLCPP_WARN(this->get_logger(), "未加载数据文件，无法播放");
            return;
        }
        
        if (!is_playing_) {
            is_playing_ = true;
            start_time_ = this->now() - rclcpp::Duration::from_seconds(
                (current_index_ > 0 ? csv_reader_->getDataRow(current_index_-1)->timestamp - data_start_time_ : 0.0) / playback_rate_);
            RCLCPP_INFO(this->get_logger(), "开始播放");
        } else {
            RCLCPP_INFO(this->get_logger(), "已在播放中");
        }
    }
    
    void pauseCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                      std::shared_ptr<std_srvs::srv::Empty::Response>) {
        if (is_playing_) {
            is_playing_ = false;
            RCLCPP_INFO(this->get_logger(), "播放暂停");
        } else {
            RCLCPP_INFO(this->get_logger(), "已处于暂停状态");
        }
    }
    
    void resetCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                      std::shared_ptr<std_srvs::srv::Empty::Response>) {
        resetPlayback();
    }

private:
    // CSV读取器
    std::unique_ptr<CsvReader> csv_reader_;
    
    // ROS2组件
    rclcpp::Publisher<underwater_nav_msgs::msg::ImuData>::SharedPtr imu_pub_;
    rclcpp::Publisher<underwater_nav_msgs::msg::DvlData>::SharedPtr dvl_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr gps_ref_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gps_array_pub_;
    
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr play_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr pause_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
    
    rclcpp::TimerBase::SharedPtr playback_timer_;
    
    // 参数
    std::string data_file_;
    double playback_rate_;
    bool loop_playback_;
    bool start_paused_;
    bool publish_gps_reference_;
    double time_offset_;
    
    // 播放状态
    bool data_loaded_ = false;
    bool is_playing_ = false;
    size_t current_index_ = 0;
    size_t total_rows_ = 0;
    
    // 时间管理
    rclcpp::Time start_time_;
    double data_start_time_ = 0.0;
};

} // namespace data_preprocessing

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<data_preprocessing::CsvPlayerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
