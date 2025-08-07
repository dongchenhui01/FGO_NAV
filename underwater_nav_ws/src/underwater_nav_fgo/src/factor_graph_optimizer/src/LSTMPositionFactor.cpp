#include "factor_graph_optimizer/LSTMPositionFactor.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <iostream>

namespace fgo {
namespace factor {

// LSTMPositionFactor 实现
gtsam::Vector LSTMPositionFactor::evaluateError(const gtsam::Pose3& pose, 
                                               gtsam::OptionalMatrixType H) const {
    // 计算预测位置与实际位置之间的误差
    gtsam::Point3 actual_position = pose.translation();
    gtsam::Point3 error = actual_position - predicted_position_;
    
    // 如果请求雅可比矩阵
    if (H) {
        // 位置误差对位姿的雅可比矩阵
        // 对于位置误差，只有位置部分有影响，姿态部分为0
        *H = gtsam::Matrix::Zero(3, 6);
        (*H).block<3, 3>(0, 0) = gtsam::Matrix::Identity(3, 3); // 位置部分
        // 姿态部分的雅可比为0，因为位置误差不直接依赖于姿态
    }
    
    return gtsam::Vector3(error.x(), error.y(), error.z());
}

void LSTMPositionFactor::print(const std::string& s, 
                              const gtsam::KeyFormatter& keyFormatter) const {
    std::cout << s << "LSTMPositionFactor(" << keyFormatter(this->key()) << ")" << std::endl;
    std::cout << "  predicted position: " << predicted_position_.transpose() << std::endl;
    std::cout << "  lever arm: " << lever_arm_.transpose() << std::endl;
    Base::print("", keyFormatter);
}

bool LSTMPositionFactor::equals(const gtsam::NonlinearFactor& other, double tol) const {
    const LSTMPositionFactor* e = dynamic_cast<const LSTMPositionFactor*>(&other);
    return e != nullptr && Base::equals(other, tol) &&
           predicted_position_.equals(e->predicted_position_, tol) &&
           lever_arm_.equals(e->lever_arm_, tol);
}

// LSTMPositionPredictor 实现
LSTMPositionPredictor::LSTMPositionPredictor() : model_loaded_(false) {}

LSTMPositionPredictor::~LSTMPositionPredictor() = default;

void LSTMPositionPredictor::initialize(const std::string& model_path) {
    try {
        if (!model_path.empty()) {
            position_model_ = torch::jit::load(model_path);
            position_model_.eval();
            model_loaded_ = true;
            std::cout << "LSTM position model loaded successfully from: " << model_path << std::endl;
        } else {
            std::cout << "No position model path provided" << std::endl;
        }
    } catch (const c10::Error& e) {
        std::cerr << "Error loading LSTM position model: " << e.what() << std::endl;
        model_loaded_ = false;
    }
}

gtsam::Point3 LSTMPositionPredictor::predictPosition(const std::vector<gtsam::Pose3>& history_trajectory) {
    if (!model_loaded_ || history_trajectory.size() < 5) {
        // 返回最后一个位置作为备选
        return history_trajectory.back().translation();
    }
    
    try {
        // 将历史轨迹转换为张量
        std::vector<torch::Tensor> trajectory_tensors;
        for (const auto& pose : history_trajectory) {
            trajectory_tensors.push_back(poseToTensor(pose));
        }
        
        // 创建输入张量 [batch_size, sequence_length, features]
        auto input_tensor = torch::stack(trajectory_tensors).unsqueeze(0);
        
        // 运行模型推理
        torch::NoGradGuard no_grad;
        auto output = position_model_.forward({input_tensor});
        
        // 提取预测结果
        auto output_tensor = output.toTensor();
        gtsam::Point3 predicted_position = tensorToPoint(output_tensor.squeeze());
        
        return predicted_position;
        
    } catch (const std::exception& e) {
        std::cerr << "Error in LSTM position prediction: " << e.what() << std::endl;
        return history_trajectory.back().translation(); // 返回最后一个位置
    }
}

torch::Tensor LSTMPositionPredictor::poseToTensor(const gtsam::Pose3& pose) {
    // 提取位置和姿态信息
    auto position = pose.translation();
    auto rotation = pose.rotation();
    
    // 转换为欧拉角
    auto euler = rotation.rpy();
    
    // 创建特征向量 [x, y, z, roll, pitch, yaw]
    std::vector<float> features = {
        static_cast<float>(position.x()),
        static_cast<float>(position.y()),
        static_cast<float>(position.z()),
        static_cast<float>(euler.x()),
        static_cast<float>(euler.y()),
        static_cast<float>(euler.z())
    };
    
    return torch::tensor(features, torch::kFloat32);
}

gtsam::Point3 LSTMPositionPredictor::tensorToPoint(const torch::Tensor& tensor) {
    auto tensor_cpu = tensor.cpu();
    auto data_ptr = tensor_cpu.data_ptr<float>();
    
    return gtsam::Point3(data_ptr[0], data_ptr[1], data_ptr[2]);
}

std::vector<torch::Tensor> LSTMPositionPredictor::normalizeTrajectory(const std::vector<gtsam::Pose3>& trajectory) {
    std::vector<torch::Tensor> normalized_tensors;
    
    for (const auto& pose : trajectory) {
        auto tensor = poseToTensor(pose);
        // 简单的标准化：除以最大范围
        tensor = tensor / 100.0; // 假设最大范围为100米
        normalized_tensors.push_back(tensor);
    }
    
    return normalized_tensors;
}

gtsam::Point3 LSTMPositionPredictor::denormalizePosition(const gtsam::Point3& normalized_position) {
    // 反标准化
    return normalized_position * 100.0;
}

} // namespace factor
} // namespace fgo 