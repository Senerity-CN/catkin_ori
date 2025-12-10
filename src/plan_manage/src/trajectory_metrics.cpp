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
    const std::vector<double>& yaw_angles,
    double dt) {
    
    TrajectoryMetrics metrics;
    
    if (positions.size() < 2 || velocities.size() < 2 || accelerations.size() < 2) {
        ROS_WARN("Insufficient data for trajectory analysis");
        return metrics;
    }
    
    // 计算最大速度
    double max_vel = 0.0;
    for (const auto& vel : velocities) {
        double vel_mag = calculateNorm(vel);
        max_vel = std::max(max_vel, vel_mag);
    }
    metrics.max_velocity = max_vel;
    
    // 计算最大加速度
    double max_acc = 0.0;
    for (const auto& acc : accelerations) {
        double acc_mag = calculateNorm(acc);
        max_acc = std::max(max_acc, acc_mag);
    }
    metrics.max_acceleration = max_acc;
    
    // 计算加加速度
    std::vector<Eigen::Vector2d> jerks = calculateJerk(accelerations, dt);
    
    // 计算最大加加速度
    double max_jerk = 0.0;
    for (const auto& jerk : jerks) {
        double jerk_mag = calculateNorm(jerk);
        max_jerk = std::max(max_jerk, jerk_mag);
    }
    metrics.max_jerk = max_jerk;
    
    // 计算最大偏航角速度
    double max_yaw_rate = 0.0;
    if (yaw_angles.size() >= 2) {
        for (size_t i = 1; i < yaw_angles.size(); ++i) {
            double yaw_rate = std::abs((yaw_angles[i] - yaw_angles[i-1]) / dt);
            max_yaw_rate = std::max(max_yaw_rate, yaw_rate);
        }
    }
    metrics.max_yaw_rate = max_yaw_rate;
    
    return metrics;
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



double TrajectoryAnalyzer::calculateNorm(const Eigen::Vector2d& vec) {
    return vec.norm();
}



} // namespace trajectory_metrics