#pragma once

#include <vector>
#include <string>
#include <memory>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "underwater_nav_msgs/msg/navigation_state.hpp"

namespace visualization {

/**
 * @brief 轨迹点结构
 */
struct TrajectoryPoint {
    double timestamp;
    double x, y, z;           // 位置
    double vx, vy, vz;        // 速度
    double qw, qx, qy, qz;    // 姿态四元数
    double pos_accuracy;      // 位置精度
    double vel_accuracy;      // 速度精度
    double heading_accuracy;  // 航向精度
};

/**
 * @brief 精度统计结构
 */
struct AccuracyStats {
    double mean_position_error;
    double std_position_error;
    double max_position_error;
    double mean_velocity_error;
    double std_velocity_error;
    double max_velocity_error;
    double mean_heading_error;
    double std_heading_error;
    double max_heading_error;
    size_t total_points;
};

/**
 * @brief 轨迹分析器
 */
class TrajectoryAnalyzer {
public:
    /**
     * @brief 构造函数
     */
    TrajectoryAnalyzer();
    
    /**
     * @brief 析构函数
     */
    ~TrajectoryAnalyzer();
    
    /**
     * @brief 添加估计轨迹点
     * @param nav_state 导航状态
     */
    void addEstimatedPoint(const underwater_nav_msgs::msg::NavigationState& nav_state);
    
    /**
     * @brief 添加参考轨迹点
     * @param point GPS参考点
     * @param timestamp 时间戳
     */
    void addReferencePoint(const geometry_msgs::msg::Point& point, double timestamp);
    
    /**
     * @brief 计算精度统计
     * @return 精度统计结果
     */
    AccuracyStats computeAccuracyStats() const;
    
    /**
     * @brief 保存轨迹到文件
     * @param filename 文件名
     * @param include_reference 是否包含参考轨迹
     * @return 成功返回true
     */
    bool saveTrajectoryToFile(const std::string& filename, bool include_reference = true) const;
    
    /**
     * @brief 生成轨迹可视化标记
     * @param frame_id 坐标系ID
     * @return 可视化标记数组
     */
    visualization_msgs::msg::MarkerArray generateTrajectoryMarkers(const std::string& frame_id = "odom") const;
    
    /**
     * @brief 生成误差可视化标记
     * @param frame_id 坐标系ID
     * @return 可视化标记数组
     */
    visualization_msgs::msg::MarkerArray generateErrorMarkers(const std::string& frame_id = "odom") const;
    
    /**
     * @brief 生成精度报告
     * @return 精度报告字符串
     */
    std::string generateAccuracyReport() const;
    
    /**
     * @brief 清空轨迹数据
     */
    void clear();
    
    /**
     * @brief 获取估计轨迹点数量
     * @return 点数量
     */
    size_t getEstimatedTrajectorySize() const { return estimated_trajectory_.size(); }
    
    /**
     * @brief 获取参考轨迹点数量
     * @return 点数量
     */
    size_t getReferenceTrajectorySize() const { return reference_trajectory_.size(); }

private:
    /**
     * @brief 查找最近的参考点
     * @param timestamp 时间戳
     * @return 参考点索引，-1表示未找到
     */
    int findNearestReferencePoint(double timestamp) const;
    
    /**
     * @brief 插值参考轨迹
     * @param timestamp 时间戳
     * @return 插值后的参考点
     */
    TrajectoryPoint interpolateReferenceTrajectory(double timestamp) const;
    
    /**
     * @brief 计算两点间距离
     * @param p1 点1
     * @param p2 点2
     * @return 距离
     */
    double computeDistance(const TrajectoryPoint& p1, const TrajectoryPoint& p2) const;
    
    /**
     * @brief 计算航向角差
     * @param q1 四元数1
     * @param q2 四元数2
     * @return 航向角差 (弧度)
     */
    double computeHeadingDifference(const TrajectoryPoint& p1, const TrajectoryPoint& p2) const;
    
    /**
     * @brief 四元数转欧拉角
     * @param qw, qx, qy, qz 四元数分量
     * @return [roll, pitch, yaw]
     */
    std::vector<double> quaternionToEuler(double qw, double qx, double qy, double qz) const;

private:
    std::vector<TrajectoryPoint> estimated_trajectory_;
    std::vector<TrajectoryPoint> reference_trajectory_;
    
    // 时间同步参数
    double max_time_difference_;  // 最大时间差阈值
};

/**
 * @brief 轨迹可视化节点
 */
class TrajectoryVisualizationNode : public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     */
    TrajectoryVisualizationNode();
    
    /**
     * @brief 析构函数
     */
    ~TrajectoryVisualizationNode();

private:
    /**
     * @brief 导航状态回调
     * @param msg 导航状态消息
     */
    void navigationStateCallback(const underwater_nav_msgs::msg::NavigationState::SharedPtr msg);
    
    /**
     * @brief GPS参考回调
     * @param msg GPS参考点消息
     */
    void gpsReferenceCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    
    /**
     * @brief 定时器回调，发布可视化
     */
    void visualizationTimerCallback();
    
    /**
     * @brief 保存轨迹服务回调
     * @param request 请求
     * @param response 响应
     */
    void saveTrajectoryCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                               std::shared_ptr<std_srvs::srv::Empty::Response> response);
    
    /**
     * @brief 生成精度报告服务回调
     * @param request 请求
     * @param response 响应
     */
    void generateReportCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                               std::shared_ptr<std_srvs::srv::Empty::Response> response);

private:
    // 轨迹分析器
    std::unique_ptr<TrajectoryAnalyzer> analyzer_;
    
    // ROS2组件
    rclcpp::Subscription<underwater_nav_msgs::msg::NavigationState>::SharedPtr nav_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr gps_ref_sub_;
    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr error_marker_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr estimated_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_pub_;
    
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_trajectory_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr generate_report_service_;
    
    rclcpp::TimerBase::SharedPtr visualization_timer_;
    
    // 参数
    double visualization_frequency_;
    std::string output_directory_;
    std::string frame_id_;
    bool publish_markers_;
    bool publish_paths_;
    
    // 路径消息
    nav_msgs::msg::Path estimated_path_;
    nav_msgs::msg::Path reference_path_;
    size_t max_path_length_;
};

} // namespace visualization
