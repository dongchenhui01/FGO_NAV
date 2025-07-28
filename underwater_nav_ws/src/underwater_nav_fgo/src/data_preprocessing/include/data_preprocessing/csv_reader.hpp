#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <memory>
#include <chrono>
#include <optional>

#include "underwater_nav_msgs/msg/imu_data.hpp"
#include "underwater_nav_msgs/msg/dvl_data.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace data_preprocessing {

/**
 * @brief CSV数据行结构
 */
struct CsvDataRow {
    double timestamp;  // 时间戳 (秒)

    // IMU数据
    bool has_imu_data = false;
    double acc_x = 0.0, acc_y = 0.0, acc_z = 0.0;           // m/s²
    double gyr_x = 0.0, gyr_y = 0.0, gyr_z = 0.0;           // rad/s
    double mag_x = 0.0, mag_y = 0.0, mag_z = 0.0;           // 任意单位

    // DVL数据
    bool has_dvl_data = false;
    double dvl_vx = 0.0, dvl_vy = 0.0, dvl_vz = 0.0;        // m/s

    // GPS参考数据
    bool has_gps_data = false;
    double gps_lon = 0.0, gps_lat = 0.0, gps_alt = 0.0;     // 度, 度, 米
    double gps_heading = 0.0, gps_pitch = 0.0, gps_roll = 0.0; // 度
    double gps_track = 0.0, gps_vel = 0.0;            // 度, m/s
    int gps_pos_qual = 0, gps_head_qual = 0;      // 质量指标
    int gps_h_svs = 0, gps_m_svs = 0;             // 卫星数
    double gps_east = 0.0, gps_north = 0.0, gps_up = 0.0;   // 米 (ENU)
    double gps_east_vel = 0.0, gps_north_vel = 0.0, gps_up_vel = 0.0; // m/s (ENU)

    bool is_valid = true;
};

/**
 * @brief 数据信息结构体
 */
struct DataInfo {
    size_t total_rows = 0;
    double start_time = 0.0;
    double end_time = 0.0;
    double duration = 0.0;
};

/**
 * @brief CSV文件读取器
 */
class CsvReader {
public:
    /**
     * @brief 默认构造函数
     */
    CsvReader();

    /**
     * @brief 构造函数
     * @param filename CSV文件路径
     */
    explicit CsvReader(const std::string& filename);

    /**
     * @brief 析构函数
     */
    ~CsvReader();

    /**
     * @brief 加载CSV文件
     * @param filename CSV文件路径
     * @return 是否加载成功
     */
    bool loadFile(const std::string& filename);
    
    /**
     * @brief 打开CSV文件
     * @return 成功返回true
     */
    bool open();
    
    /**
     * @brief 关闭CSV文件
     */
    void close();
    
    /**
     * @brief 读取下一行数据
     * @param row 输出数据行
     * @return 成功返回true，文件结束返回false
     */
    bool readNextRow(CsvDataRow& row);
    
    /**
     * @brief 检查是否到达文件末尾
     * @return 到达末尾返回true
     */
    bool isEof() const;
    
    /**
     * @brief 获取总行数
     * @return 总行数
     */
    size_t getTotalRows() const;
    
    /**
     * @brief 获取当前行号
     * @return 当前行号
     */
    size_t getCurrentRow() const;
    
    /**
     * @brief 重置到文件开头
     */
    void reset();
    
    /**
     * @brief 跳转到指定行
     * @param row_number 目标行号
     * @return 成功返回true
     */
    bool seekToRow(size_t row_number);

    /**
     * @brief 获取数据信息
     * @return 数据信息结构体
     */
    DataInfo getDataInfo() const;

    /**
     * @brief 获取指定行的数据
     * @param row_index 行索引
     * @return 数据行，如果失败返回nullopt
     */
    std::optional<CsvDataRow> getDataRow(size_t row_index);

private:
    /**
     * @brief 解析时间戳字符串
     * @param timestamp_str ISO 8601格式时间戳
     * @return 系统时间点
     */
    std::chrono::system_clock::time_point parseTimestamp(const std::string& timestamp_str);
    
    /**
     * @brief 分割CSV行
     * @param line CSV行字符串
     * @return 字段向量
     */
    std::vector<std::string> splitCsvLine(const std::string& line);
    
    /**
     * @brief 验证数据有效性
     * @param row 数据行
     * @return 有效返回true
     */
    bool validateRow(const CsvDataRow& row);

private:
    std::string filename_;
    std::ifstream file_;
    size_t current_row_;
    size_t total_rows_;
    bool header_read_;
    std::vector<std::string> header_;
};

/**
 * @brief 数据转换工具类
 */
class DataConverter {
public:
    /**
     * @brief 将CSV数据行转换为IMU消息
     * @param row CSV数据行
     * @return IMU消息
     */
    static underwater_nav_msgs::msg::ImuData toImuMsg(const CsvDataRow& row);
    
    /**
     * @brief 将CSV数据行转换为DVL消息
     * @param row CSV数据行
     * @return DVL消息
     */
    static underwater_nav_msgs::msg::DvlData toDvlMsg(const CsvDataRow& row);
    
    /**
     * @brief 将CSV数据行转换为GPS参考位置
     * @param row CSV数据行
     * @return GPS位置点
     */
    static geometry_msgs::msg::Point toGpsPosition(const CsvDataRow& row);
    
    /**
     * @brief 角度转弧度
     * @param degrees 角度值
     * @return 弧度值
     */
    static double degToRad(double degrees);
    
    /**
     * @brief 弧度转角度
     * @param radians 弧度值
     * @return 角度值
     */
    static double radToDeg(double radians);
};

} // namespace data_preprocessing
