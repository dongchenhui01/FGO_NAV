#ifndef LSTM_POSITION_FACTOR_H
#define LSTM_POSITION_FACTOR_H

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <torch/script.h>
#include <vector>

namespace fgo {
namespace factor {

/**
 * @brief LSTM位置预测因子
 * 
 * 基于LSTM模型预测的位置信息作为因子图优化的约束
 * 用于校正和改善导航系统的位置估计
 */
class LSTMPositionFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
public:
    using Base = gtsam::NoiseModelFactor1<gtsam::Pose3>;
    
    /**
     * @brief 构造函数
     * @param key 位姿节点键值
     * @param predicted_position LSTM预测的位置
     * @param lever_arm 杆臂偏移
     * @param noise_model 噪声模型
     */
    LSTMPositionFactor(gtsam::Key key, 
                      const gtsam::Point3& predicted_position,
                      const gtsam::Point3& lever_arm,
                      const gtsam::noiseModel::Base::shared_ptr& noise_model)
        : Base(noise_model, key), 
          predicted_position_(predicted_position), 
          lever_arm_(lever_arm) {}

    /**
     * @brief 析构函数
     */
    virtual ~LSTMPositionFactor() = default;

    /**
     * @brief 计算误差向量
     * @param pose 当前位姿估计
     * @param H 雅可比矩阵（可选）
     * @return 误差向量
     */
    gtsam::Vector evaluateError(const gtsam::Pose3& pose, 
                               gtsam::OptionalMatrixType H = nullptr) const override;

    /**
     * @brief 克隆因子
     */
    gtsam::NonlinearFactor::shared_ptr clone() const override {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new LSTMPositionFactor(*this)));
    }

    /**
     * @brief 打印因子信息
     */
    void print(const std::string& s = "", 
               const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override;

    /**
     * @brief 检查是否相等
     */
    bool equals(const gtsam::NonlinearFactor& other, double tol = 1e-9) const override;

    /**
     * @brief 获取预测位置
     */
    gtsam::Point3 getPredictedPosition() const { return predicted_position_; }

    /**
     * @brief 获取杆臂偏移
     */
    gtsam::Point3 getLeverArm() const { return lever_arm_; }

private:
    gtsam::Point3 predicted_position_;  // LSTM预测的位置
    gtsam::Point3 lever_arm_;          // 杆臂偏移
};

/**
 * @brief LSTM位置预测器类
 * 
 * 负责加载LSTM模型并进行位置预测
 */
class LSTMPositionPredictor {
public:
    LSTMPositionPredictor();
    ~LSTMPositionPredictor();

    /**
     * @brief 初始化预测器
     * @param model_path LSTM模型路径
     */
    void initialize(const std::string& model_path);

    /**
     * @brief 预测下一时刻位置
     * @param history_trajectory 历史轨迹
     * @return 预测的位置
     */
    gtsam::Point3 predictPosition(const std::vector<gtsam::Pose3>& history_trajectory);

    /**
     * @brief 检查模型是否已加载
     */
    bool isModelLoaded() const { return model_loaded_; }

private:
    torch::jit::script::Module position_model_;
    bool model_loaded_;

    /**
     * @brief 将GTSAM位姿转换为PyTorch张量
     */
    torch::Tensor poseToTensor(const gtsam::Pose3& pose);

    /**
     * @brief 将PyTorch张量转换为GTSAM点
     */
    gtsam::Point3 tensorToPoint(const torch::Tensor& tensor);

    /**
     * @brief 标准化轨迹数据
     */
    std::vector<torch::Tensor> normalizeTrajectory(const std::vector<gtsam::Pose3>& trajectory);

    /**
     * @brief 反标准化位置数据
     */
    gtsam::Point3 denormalizePosition(const gtsam::Point3& normalized_position);
};

} // namespace factor
} // namespace fgo

#endif // LSTM_POSITION_FACTOR_H 