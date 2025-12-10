#include <plan_manage/trajectory_metrics.h>
#include <ros/ros.h>
#include <numeric>
#include <algorithm>

namespace trajectory_metrics {

TrajectoryAnalyzer::TrajectoryAnalyzer() {}

TrajectoryMetrics TrajectoryAnalyzer::analyzeTrajectory(
    const std::vector<Eigen::Vector2d>& positions,
    const std::vector<Eigen::Vector2d>& velocities,
    const std::vector<Eigen::Vector2d>& accelerations,
    double dt) {
    
    TrajectoryMetrics metrics;
    
    if (positions.size() < 2 || velocities.size() < 2 || accelerations.size() < 2) {
        ROS_WARN("Insufficient data for trajectory analysis");
        return metrics;
    }
    
    // 计算速度指标
    std::vector<double> velocity_magnitudes;
    double velocity_sum = 0.0, velocity_sq_sum = 0.0;
    
    for (const auto& vel : velocities) {
        double vel_mag = calculateNorm(vel);
        velocity_magnitudes.push_back(vel_mag);
        velocity_sum += vel_mag;
        velocity_sq_sum += vel_mag * vel_mag;
    }
    
    if (!velocity_magnitudes.empty()) {
        metrics.max_velocity = *std::max_element(velocity_magnitudes.begin(), velocity_magnitudes.end());
        metrics.avg_velocity = velocity_sum / velocity_magnitudes.size();
        metrics.velocity_variance = calculateVariance(velocity_magnitudes);
        metrics.velocity_rms = std::sqrt(velocity_sq_sum / velocity_magnitudes.size());
        metrics.velocity_std = std::sqrt(metrics.velocity_variance);
    }
    
    // 计算加速度指标
    std::vector<double> acceleration_magnitudes;
    double acceleration_sum = 0.0, acceleration_sq_sum = 0.0;
    
    for (const auto& acc : accelerations) {
        double acc_mag = calculateNorm(acc);
        acceleration_magnitudes.push_back(acc_mag);
        acceleration_sum += acc_mag;
        acceleration_sq_sum += acc_mag * acc_mag;
    }
    
    if (!acceleration_magnitudes.empty()) {
        metrics.max_acceleration = *std::max_element(acceleration_magnitudes.begin(), acceleration_magnitudes.end());
        metrics.avg_acceleration = acceleration_sum / acceleration_magnitudes.size();
        metrics.acceleration_variance = calculateVariance(acceleration_magnitudes);
        metrics.acceleration_rms = std::sqrt(acceleration_sq_sum / acceleration_magnitudes.size());
        metrics.acceleration_std = std::sqrt(metrics.acceleration_variance);
    }
    
    // 计算加加速度
    std::vector<Eigen::Vector2d> jerks = calculateJerk(accelerations, dt);
    
    // 计算Jerk指标
    std::vector<double> jerk_magnitudes;
    double total_jerk_sum = 0.0, jerk_sq_sum = 0.0;
    
    for (const auto& jerk : jerks) {
        double jerk_mag = calculateNorm(jerk);
        jerk_magnitudes.push_back(jerk_mag);
        total_jerk_sum += jerk_mag * dt;  // 积分
        jerk_sq_sum += jerk_mag * jerk_mag;
    }
    
    if (!jerk_magnitudes.empty()) {
        metrics.max_jerk = *std::max_element(jerk_magnitudes.begin(), jerk_magnitudes.end());
        metrics.avg_jerk = std::accumulate(jerk_magnitudes.begin(), jerk_magnitudes.end(), 0.0) / jerk_magnitudes.size();
        metrics.jerk_variance = calculateVariance(jerk_magnitudes);
        metrics.jerk_rms = std::sqrt(jerk_sq_sum / jerk_magnitudes.size());
        metrics.jerk_std = std::sqrt(metrics.jerk_variance);
        metrics.total_jerk = total_jerk_sum;
    }
    
    // 计算路径长度
    double path_length = 0.0;
    for (size_t i = 1; i < positions.size(); ++i) {
        path_length += (positions[i] - positions[i-1]).norm();
    }
    metrics.path_length = path_length;
    
    // 计算执行时间
    metrics.execution_time = positions.size() * dt;
    
    // 计算舒适性和平滑性指标
    metrics.comfort_index = calculateComfortIndex(jerks);
    metrics.smoothness_index = calculateSmoothnessIndex(velocities, accelerations);
    
    // 计算跟踪误差（如果有参考轨迹的话，这里简化处理）
    metrics.tracking_error = 0.0;  // 需要参考轨迹来计算
    
    // 计算能耗估计（基于加速度）
    metrics.energy_consumption = acceleration_sq_sum * dt;
    
    return metrics;
}

void TrajectoryAnalyzer::calculateSwitchingJumps(
    const Eigen::Vector2d& vel_before,
    const Eigen::Vector2d& vel_after,
    const Eigen::Vector2d& acc_before,
    const Eigen::Vector2d& acc_after,
    double& vel_jump,
    double& acc_jump) {
    
    vel_jump = calculateNorm(vel_after - vel_before);
    acc_jump = calculateNorm(acc_after - acc_before);
}

std::vector<Eigen::Vector2d> TrajectoryAnalyzer::calculateJerk(
    const std::vector<Eigen::Vector2d>& accelerations,
    double dt) {
    
    std::vector<Eigen::Vector2d> jerks;
    
    for (size_t i = 1; i < accelerations.size(); ++i) {
        Eigen::Vector2d jerk = (accelerations[i] - accelerations[i-1]) / dt;
        jerks.push_back(jerk);
    }
    
    return jerks;
}

double TrajectoryAnalyzer::calculateComfortIndex(const std::vector<Eigen::Vector2d>& jerks) {
    if (jerks.empty()) return 0.0;
    
    // 舒适性指标：基于加加速度的RMS值
    double jerk_rms = 0.0;
    for (const auto& jerk : jerks) {
        jerk_rms += jerk.squaredNorm();
    }
    jerk_rms = std::sqrt(jerk_rms / jerks.size());
    
    // 舒适性指数：加加速度越小越舒适
    return 1.0 / (1.0 + jerk_rms);  // 范围[0,1]，越大越舒适
}

double TrajectoryAnalyzer::calculateSmoothnessIndex(
    const std::vector<Eigen::Vector2d>& velocities,
    const std::vector<Eigen::Vector2d>& accelerations) {
    
    if (velocities.size() < 2 || accelerations.size() < 2) return 0.0;
    
    // 平滑性指标：基于速度和加速度的变化率
    double vel_variation = 0.0;
    double acc_variation = 0.0;
    
    for (size_t i = 1; i < velocities.size(); ++i) {
        vel_variation += calculateNorm(velocities[i] - velocities[i-1]);
    }
    
    for (size_t i = 1; i < accelerations.size(); ++i) {
        acc_variation += calculateNorm(accelerations[i] - accelerations[i-1]);
    }
    
    double total_variation = vel_variation + acc_variation;
    return 1.0 / (1.0 + total_variation);  // 范围[0,1]，越大越平滑
}

double TrajectoryAnalyzer::calculateNorm(const Eigen::Vector2d& vec) {
    return vec.norm();
}

double TrajectoryAnalyzer::calculateVariance(const std::vector<double>& data) {
    if (data.empty()) return 0.0;
    
    double mean = std::accumulate(data.begin(), data.end(), 0.0) / data.size();
    double variance = 0.0;
    
    for (const auto& val : data) {
        variance += (val - mean) * (val - mean);
    }
    
    return variance / data.size();
}

} // namespace trajectory_metrics