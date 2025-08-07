#include "factor_graph_optimizer/DynamicDvlHandler.h"
#include <gtsam/base/Vector.h>
#include <numeric>
#include <algorithm>
#include <iostream>

namespace fgo::utils {

void DynamicDvlHandler::initialize(const std::string& velocity_model_path, int window_size, double threshold, int neighbor_range, double kernel_size) {
    try {
        if (!velocity_model_path.empty()) {
            lstm_velocity_model_ = torch::jit::load(velocity_model_path);
            lstm_velocity_model_.eval();
            std::cout << "LSTM velocity model loaded successfully from: " << velocity_model_path << std::endl;
        } else {
            std::cerr << "LSTM velocity model path is empty!" << std::endl;
            is_initialized_ = false;
            return;
        }
    } catch (const c10::Error& e) {
        std::cerr << "Error loading the LSTM velocity model: " << e.what() << std::endl;
        is_initialized_ = false;
        return;
    }
    window_size_ = window_size;
    wsos_threshold_ = threshold;
    neighbor_range_ = neighbor_range;
    gaussian_kernel_size_ = kernel_size;
    is_initialized_ = true;
}

gtsam::Vector3 DynamicDvlHandler::processDvlMeasurement(const gtsam::Vector3& raw_dvl_velocity, double timestamp) {
    if (!is_initialized_ || !wsos_enabled_) {
        return raw_dvl_velocity;
    }
    if (history_window_.size() < window_size_) {
        history_window_.push_back(raw_dvl_velocity);
        return raw_dvl_velocity;
    }
    double score = calculateWsosScore(raw_dvl_velocity);
    if (score >= wsos_threshold_) {
        history_window_.pop_front();
        history_window_.push_back(raw_dvl_velocity);
        return raw_dvl_velocity;
    } else {
        std::cout << "DVL anomaly detected at timestamp " << timestamp << "! Score: " << score << ". Using LSTM prediction." << std::endl;
        return predictVelocity();
    }
}

double DynamicDvlHandler::calculateWsosScore(const gtsam::Vector3& new_velocity) {
    auto temp_data = history_window_;
    temp_data.push_back(new_velocity);
    int n_points = temp_data.size();
    gtsam::Matrix dist_matrix(n_points, n_points);
    for (int i = 0; i < n_points; ++i) {
        for (int j = i; j < n_points; ++j) {
            double dist = (temp_data[i] - temp_data[j]).norm();
            dist_matrix(i, j) = dist;
            dist_matrix(j, i) = dist;
        }
    }
    double kernel_denom = 2.0 * gaussian_kernel_size_ * gaussian_kernel_size_;
    gtsam::Matrix affinity_matrix = (-dist_matrix.array().square() / kernel_denom).exp();
    std::vector<double> row_affinity;
    for(int j=0; j < n_points -1; ++j) {
        row_affinity.push_back(affinity_matrix(n_points - 1, j));
    }
    std::sort(row_affinity.rbegin(), row_affinity.rend());
    double score_sum = 0.0;
    int L = std::min(neighbor_range_, (int)row_affinity.size());
    if (L == 0) return 1.0;
    for(int i = 0; i < L; ++i) {
        score_sum += row_affinity[i];
    }
    return score_sum / L;
}

gtsam::Vector3 DynamicDvlHandler::predictVelocity() {
    if(history_window_.empty()) return gtsam::Vector3::Zero();
    std::vector<double> flat_history;
    for(const auto& v : history_window_){
        flat_history.push_back(v.x());
        flat_history.push_back(v.y());
        flat_history.push_back(v.z());
    }
    torch::Tensor input_tensor = torch::from_blob(flat_history.data(), {1, window_size_, 3}, torch::kDouble).to(torch::kFloat);
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(input_tensor);
    at::Tensor output_tensor;
    try {
        output_tensor = lstm_velocity_model_.forward(inputs).toTensor();
    } catch (const c10::Error& e) {
        std::cerr << "Error during LSTM velocity prediction: " << e.what() << std::endl;
        return history_window_.back();
    }
    return gtsam::Vector3(
        output_tensor[0][0].item<double>(), 
        output_tensor[0][1].item<double>(),
        output_tensor[0][2].item<double>()
    );
}

} // namespace fgo::utils 