#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

std::vector<std::string> splitCsvLine(const std::string& line) {
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

int main() {
    std::ifstream file("/home/dongchenhui/underwater_nav_fgo/data/sensor_fusion_log_20250712_102606.csv");
    if (!file.is_open()) {
        std::cout << "无法打开文件" << std::endl;
        return 1;
    }
    
    std::string line;
    int line_count = 0;
    
    // 读取表头
    if (std::getline(file, line)) {
        auto header = splitCsvLine(line);
        std::cout << "表头字段数: " << header.size() << std::endl;
        for (size_t i = 0; i < header.size() && i < 10; ++i) {
            std::cout << "  [" << i << "] " << header[i] << std::endl;
        }
        line_count++;
    }
    
    // 读取前几行数据
    for (int i = 0; i < 5 && std::getline(file, line); ++i) {
        auto fields = splitCsvLine(line);
        std::cout << "\n第" << (i+2) << "行数据 (字段数: " << fields.size() << "):" << std::endl;
        
        if (fields.size() >= 13) {
            std::cout << "  时间戳: " << fields[0] << std::endl;
            std::cout << "  IMU加速度: [" << fields[1] << ", " << fields[2] << ", " << fields[3] << "]" << std::endl;
            std::cout << "  IMU角速度: [" << fields[4] << ", " << fields[5] << ", " << fields[6] << "]" << std::endl;
            std::cout << "  DVL速度: [" << fields[10] << ", " << fields[11] << ", " << fields[12] << "]" << std::endl;
        }
        line_count++;
    }
    
    // 计算总行数
    while (std::getline(file, line)) {
        line_count++;
    }
    
    std::cout << "\n总行数: " << line_count << std::endl;
    
    file.close();
    return 0;
}
