#ifndef TRAJECTORY_METRICS_H
#define TRAJECTORY_METRICS_H

#include <vector>
#include <Eigen/Dense>
#include <cmath>

namespace trajectory_metrics {

struct TrajectoryMetrics {
    // 切换时刻的关键指标
    double max_velocity;       // 最大速度
    double max_acceleration;   // 最大加速度
    double max_jerk;           // 最大加加速度
    double max_yaw_rate;       // 最大偏航角速度
    
    };

class TrajectoryAnalyzer {
public:
    TrajectoryAnalyzer();
    
    // 计算轨迹指标
    TrajectoryMetrics analyzeTrajectory(
        const std::vector<Eigen::Vector2d>& positions,
        const std::vector<Eigen::Vector2d>& velocities,
        const std::vector<Eigen::Vector2d>& accelerations,
        const std::vector<double>& yaw_angles,
        double dt);
    
    private:
    double calculateNorm(const Eigen::Vector2d& vec);
    std::vector<Eigen::Vector2d> calculateJerk(
        const std::vector<Eigen::Vector2d>& accelerations,
        double dt);
};

} // namespace trajectory_metrics

#endif // TRAJECTORY_METRICS_H