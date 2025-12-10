#ifndef TRAJECTORY_METRICS_H
#define TRAJECTORY_METRICS_H

#include <vector>
#include <Eigen/Dense>
#include <cmath>

namespace trajectory_metrics {

struct TrajectoryMetrics {
    // 速度指标
    double max_velocity;       // 最大速度
    double avg_velocity;       // 平均速度
    double velocity_variance;  // 速度方差
    double velocity_rms;       // 速度RMS值
    
    // 加速度指标
    double max_acceleration;   // 最大加速度
    double avg_acceleration;   // 平均加速度
    double acceleration_variance; // 加速度方差
    double acceleration_rms;   // 加速度RMS值
    
    // Jerk指标
    double max_jerk;           // 最大加加速度
    double avg_jerk;           // 平均加加速度
    double jerk_variance;      // 加加速度方差
    double jerk_rms;           // 加加速度RMS值
    double total_jerk;         // 总加加速度积分
    
    // 连续性指标
    double velocity_jump;      // 速度跳跃
    double acceleration_jump;  // 加速度跳跃
    
    // 舒适性指标
    double comfort_index;      // 舒适性指数
    double smoothness_index;   // 平滑性指数
    
    // 执行效率指标
    double tracking_error;     // 跟踪误差
    double energy_consumption; // 能耗估计
    double path_length;        // 路径长度
    double execution_time;     // 执行时间
    
    // 统计指标
    double velocity_std;       // 速度标准差
    double acceleration_std;   // 加速度标准差
    double jerk_std;          // 加加速度标准差
};

class TrajectoryAnalyzer {
public:
    TrajectoryAnalyzer();
    
    // 计算轨迹指标
    TrajectoryMetrics analyzeTrajectory(
        const std::vector<Eigen::Vector2d>& positions,
        const std::vector<Eigen::Vector2d>& velocities,
        const std::vector<Eigen::Vector2d>& accelerations,
        double dt);
    
    // 计算切换点的跳跃
    void calculateSwitchingJumps(
        const Eigen::Vector2d& vel_before,
        const Eigen::Vector2d& vel_after,
        const Eigen::Vector2d& acc_before,
        const Eigen::Vector2d& acc_after,
        double& vel_jump,
        double& acc_jump);
    
    // 计算加加速度
    std::vector<Eigen::Vector2d> calculateJerk(
        const std::vector<Eigen::Vector2d>& accelerations,
        double dt);
    
    // 计算舒适性指标
    double calculateComfortIndex(const std::vector<Eigen::Vector2d>& jerks);
    
    // 计算平滑性指标
    double calculateSmoothnessIndex(
        const std::vector<Eigen::Vector2d>& velocities,
        const std::vector<Eigen::Vector2d>& accelerations);

private:
    double calculateNorm(const Eigen::Vector2d& vec);
    double calculateVariance(const std::vector<double>& data);
};

} // namespace trajectory_metrics

#endif // TRAJECTORY_METRICS_H