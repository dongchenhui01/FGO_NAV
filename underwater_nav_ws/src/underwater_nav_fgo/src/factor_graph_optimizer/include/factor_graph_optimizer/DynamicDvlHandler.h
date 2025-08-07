#ifndef DYNAMIC_DVL_HANDLER_H
#define DYNAMIC_DVL_HANDLER_H

#include <vector>
#include <deque>
#include <memory>
#include <torch/script.h>
#include <Eigen/Dense>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

namespace fgo {
namespace utils {

/**
 * @brief DVL异常检测和处理类
 * 
 * 基于WSOS (Weighted Sum of Squares) 异常检测算法和LSTM模型
 * 对DVL速度测量进行异常检测和校正
 */
class DynamicDvlHandler {
public:
    DynamicDvlHandler();
    ~DynamicDvlHandler();

    /**
     * @brief 初始化DVL处理器
     * @param velocity_model_path LSTM速度模型路径
     * @param window_size 异常检测窗口大小
     * @param threshold 异常检测阈值
     */
    void initialize(const std::string& velocity_model_path, 
                   int window_size = 10, 
                   double threshold = 0.9974);

    /**
     * @brief 处理DVL测量数据
     * @param raw_velocity 原始DVL速度测量
     * @param timestamp 时间戳
     * @return 处理后的干净速度
     */
    gtsam::Vector3 processDvlMeasurement(const gtsam::Vector3& raw_velocity, 
                                        double timestamp);

    /**
     * @brief 检查DVL测量是否为异常值
     * @param velocity 速度测量
     * @return true if anomaly detected
     */
    bool detectAnomaly(const gtsam::Vector3& velocity);

    /**
     * @brief 使用LSTM模型预测速度
     * @param history_velocities 历史速度序列
     * @return 预测的速度
     */
    gtsam::Vector3 predictVelocity(const std::vector<gtsam::Vector3>& history_velocities);

    /**
     * @brief 获取处理统计信息
     */
    struct Statistics {
        int total_measurements;
        int anomaly_detections;
        int corrections_applied;
        double average_processing_time;
    };
    
    Statistics getStatistics() const;

private:
    // LSTM模型
    torch::jit::script::Module velocity_model_;
    bool model_loaded_;

    // 异常检测参数
    int window_size_;
    double threshold_;
    double gaussian_kernel_size_;
    int neighbor_range_;

    // 数据缓冲区
    std::deque<gtsam::Vector3> velocity_buffer_;
    std::deque<double> timestamp_buffer_;

    // 统计信息
    Statistics stats_;

    /**
     * @brief 计算WSOS异常检测分数
     */
    double calculateWSOSScore(const gtsam::Vector3& velocity);

    /**
     * @brief 将GTSAM向量转换为PyTorch张量
     */
    torch::Tensor vectorToTensor(const gtsam::Vector3& vector);

    /**
     * @brief 将PyTorch张量转换为GTSAM向量
     */
    gtsam::Vector3 tensorToVector(const torch::Tensor& tensor);

    /**
     * @brief 标准化速度数据
     */
    gtsam::Vector3 normalizeVelocity(const gtsam::Vector3& velocity);

    /**
     * @brief 反标准化速度数据
     */
    gtsam::Vector3 denormalizeVelocity(const gtsam::Vector3& normalized_velocity);
};

} // namespace utils
} // namespace fgo

#endif // DYNAMIC_DVL_HANDLER_H 