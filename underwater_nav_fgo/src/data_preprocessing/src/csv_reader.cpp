#include "data_preprocessing/csv_reader.hpp"
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>

namespace data_preprocessing {

CsvReader::CsvReader(const std::string& filename) 
    : filename_(filename), current_row_(0), total_rows_(0), header_read_(false) {
}

CsvReader::~CsvReader() {
    close();
}

bool CsvReader::open() {
    file_.open(filename_);
    if (!file_.is_open()) {
        return false;
    }
    
    // 读取表头
    std::string header_line;
    if (std::getline(file_, header_line)) {
        header_ = splitCsvLine(header_line);
        header_read_ = true;
        current_row_ = 1;
        
        // 计算总行数
        auto current_pos = file_.tellg();
        file_.seekg(0, std::ios::end);
        auto end_pos = file_.tellg();
        file_.seekg(current_pos);
        
        // 估算行数（简化方法）
        total_rows_ = static_cast<size_t>((end_pos - current_pos) / 100); // 假设每行约100字符
    }
    
    return header_read_;
}

void CsvReader::close() {
    if (file_.is_open()) {
        file_.close();
    }
}

bool CsvReader::readNextRow(CsvDataRow& row) {
    if (!file_.is_open() || file_.eof()) {
        return false;
    }
    
    std::string line;
    if (!std::getline(file_, line)) {
        return false;
    }
    
    auto fields = splitCsvLine(line);
    if (fields.size() < 28) { // 最少需要的字段数
        return false;
    }
    
    try {
        // 解析时间戳
        row.timestamp = parseTimestamp(fields[0]);
        
        // 解析IMU数据
        row.acc_x = std::stod(fields[1]);
        row.acc_y = std::stod(fields[2]);
        row.acc_z = std::stod(fields[3]);
        row.gyr_x = std::stod(fields[4]);
        row.gyr_y = std::stod(fields[5]);
        row.gyr_z = std::stod(fields[6]);
        row.mag_x = std::stod(fields[7]);
        row.mag_y = std::stod(fields[8]);
        row.mag_z = std::stod(fields[9]);
        
        // 解析DVL数据
        row.dvl_vx = std::stod(fields[10]);
        row.dvl_vy = std::stod(fields[11]);
        row.dvl_vz = std::stod(fields[12]);
        
        // 解析GPS数据
        row.gps_lon = std::stod(fields[13]);
        row.gps_lat = std::stod(fields[14]);
        row.gps_alt = std::stod(fields[15]);
        row.gps_heading = std::stod(fields[16]);
        row.gps_pitch = std::stod(fields[17]);
        row.gps_roll = std::stod(fields[18]);
        row.gps_track = std::stod(fields[19]);
        row.gps_vel = std::stod(fields[20]);
        row.gps_pos_qual = std::stoi(fields[21]);
        row.gps_head_qual = std::stoi(fields[22]);
        row.gps_h_svs = std::stoi(fields[23]);
        row.gps_m_svs = std::stoi(fields[24]);
        row.gps_east = std::stod(fields[25]);
        row.gps_north = std::stod(fields[26]);
        row.gps_up = std::stod(fields[27]);
        
        if (fields.size() >= 31) {
            row.gps_east_vel = std::stod(fields[28]);
            row.gps_north_vel = std::stod(fields[29]);
            row.gps_up_vel = std::stod(fields[30]);
        }
        
        row.is_valid = validateRow(row);
        current_row_++;
        
        return true;
    } catch (const std::exception& e) {
        row.is_valid = false;
        return false;
    }
}

bool CsvReader::isEof() const {
    return file_.eof();
}

size_t CsvReader::getTotalRows() const {
    return total_rows_;
}

size_t CsvReader::getCurrentRow() const {
    return current_row_;
}

void CsvReader::reset() {
    if (file_.is_open()) {
        file_.clear();
        file_.seekg(0, std::ios::beg);
        
        // 跳过表头
        std::string header_line;
        std::getline(file_, header_line);
        current_row_ = 1;
    }
}

bool CsvReader::seekToRow(size_t row_number) {
    reset();
    
    for (size_t i = 1; i < row_number && !file_.eof(); ++i) {
        std::string line;
        std::getline(file_, line);
        current_row_++;
    }
    
    return !file_.eof();
}

std::chrono::system_clock::time_point CsvReader::parseTimestamp(const std::string& timestamp_str) {
    // 解析ISO 8601格式时间戳
    std::tm tm = {};
    std::istringstream ss(timestamp_str);
    
    // 简化解析，假设格式为 "YYYY-MM-DDTHH:MM:SS.sssZ"
    ss >> std::get_time(&tm, "%Y-%m-%dT%H:%M:%S");
    
    auto time_point = std::chrono::system_clock::from_time_t(std::mktime(&tm));
    
    // 处理毫秒部分（如果存在）
    auto dot_pos = timestamp_str.find('.');
    if (dot_pos != std::string::npos) {
        auto ms_str = timestamp_str.substr(dot_pos + 1, 3);
        if (!ms_str.empty() && ms_str != "Z") {
            int milliseconds = std::stoi(ms_str);
            time_point += std::chrono::milliseconds(milliseconds);
        }
    }
    
    return time_point;
}

std::vector<std::string> CsvReader::splitCsvLine(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream ss(line);
    std::string field;
    
    while (std::getline(ss, field, ',')) {
        // 去除前后空格
        field.erase(0, field.find_first_not_of(" \t"));
        field.erase(field.find_last_not_of(" \t") + 1);
        fields.push_back(field);
    }
    
    return fields;
}

bool CsvReader::validateRow(const CsvDataRow& row) {
    // 基本数据有效性检查
    const double MAX_ACC = 100.0;  // m/s²
    const double MAX_GYR = 10.0;   // rad/s
    const double MAX_VEL = 10.0;   // m/s
    
    // 检查加速度
    if (std::abs(row.acc_x) > MAX_ACC || std::abs(row.acc_y) > MAX_ACC || std::abs(row.acc_z) > MAX_ACC) {
        return false;
    }
    
    // 检查角速度
    if (std::abs(row.gyr_x) > MAX_GYR || std::abs(row.gyr_y) > MAX_GYR || std::abs(row.gyr_z) > MAX_GYR) {
        return false;
    }
    
    // 检查DVL速度
    if (std::abs(row.dvl_vx) > MAX_VEL || std::abs(row.dvl_vy) > MAX_VEL || std::abs(row.dvl_vz) > MAX_VEL) {
        return false;
    }
    
    // 检查是否为NaN
    if (std::isnan(row.acc_x) || std::isnan(row.acc_y) || std::isnan(row.acc_z) ||
        std::isnan(row.gyr_x) || std::isnan(row.gyr_y) || std::isnan(row.gyr_z) ||
        std::isnan(row.dvl_vx) || std::isnan(row.dvl_vy) || std::isnan(row.dvl_vz)) {
        return false;
    }
    
    return true;
}

// DataConverter实现
underwater_nav_msgs::msg::ImuData DataConverter::toImuMsg(const CsvDataRow& row) {
    underwater_nav_msgs::msg::ImuData msg;
    
    // 设置线性加速度
    msg.linear_acceleration.x = row.acc_x;
    msg.linear_acceleration.y = row.acc_y;
    msg.linear_acceleration.z = row.acc_z;
    
    // 设置角速度
    msg.angular_velocity.x = row.gyr_x;
    msg.angular_velocity.y = row.gyr_y;
    msg.angular_velocity.z = row.gyr_z;
    
    // 设置磁场
    msg.magnetic_field.x = row.mag_x;
    msg.magnetic_field.y = row.mag_y;
    msg.magnetic_field.z = row.mag_z;
    
    // 设置协方差（简化为对角矩阵）
    std::fill(msg.linear_acceleration_covariance.begin(), msg.linear_acceleration_covariance.end(), 0.0);
    std::fill(msg.angular_velocity_covariance.begin(), msg.angular_velocity_covariance.end(), 0.0);
    std::fill(msg.magnetic_field_covariance.begin(), msg.magnetic_field_covariance.end(), 0.0);
    
    // 对角元素
    msg.linear_acceleration_covariance[0] = msg.linear_acceleration_covariance[4] = msg.linear_acceleration_covariance[8] = 0.01 * 0.01;
    msg.angular_velocity_covariance[0] = msg.angular_velocity_covariance[4] = msg.angular_velocity_covariance[8] = 0.0017 * 0.0017;
    msg.magnetic_field_covariance[0] = msg.magnetic_field_covariance[4] = msg.magnetic_field_covariance[8] = 0.1 * 0.1;
    
    msg.is_valid = row.is_valid;
    
    return msg;
}

underwater_nav_msgs::msg::DvlData DataConverter::toDvlMsg(const CsvDataRow& row) {
    underwater_nav_msgs::msg::DvlData msg;
    
    // 设置速度
    msg.velocity.x = row.dvl_vx;
    msg.velocity.y = row.dvl_vy;
    msg.velocity.z = row.dvl_vz;
    
    // 设置协方差（简化为对角矩阵）
    std::fill(msg.velocity_covariance.begin(), msg.velocity_covariance.end(), 0.0);
    msg.velocity_covariance[0] = msg.velocity_covariance[4] = 0.02 * 0.02;  // x, y
    msg.velocity_covariance[8] = 0.05 * 0.05;  // z
    
    // 设置状态标志
    msg.is_valid = row.is_valid;
    msg.bottom_track_valid = true;
    msg.water_track_valid = false;
    
    // 设置质量指标（模拟值）
    msg.altitude = 10.0;
    msg.beam_quality = {80.0, 85.0, 82.0, 78.0};
    msg.num_good_beams = 4;
    msg.frequency = 600000.0;  // 600kHz
    msg.beam_angle = 30.0;
    
    return msg;
}

geometry_msgs::msg::Point DataConverter::toGpsPosition(const CsvDataRow& row) {
    geometry_msgs::msg::Point point;
    point.x = row.gps_east;
    point.y = row.gps_north;
    point.z = row.gps_up;
    return point;
}

double DataConverter::degToRad(double degrees) {
    return degrees * M_PI / 180.0;
}

double DataConverter::radToDeg(double radians) {
    return radians * 180.0 / M_PI;
}

} // namespace data_preprocessing
