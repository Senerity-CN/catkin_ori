#ifndef PLAN_MANAGE_HPP
#define PLAN_MANAGE_HPP
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <path_searching/kino_astar.h>
#include <tools/visualization.hpp>
#include <se2Plan/se2Polytraj.hpp>
#include <se2Plan/se2Trajopt.hpp>
#include <tools/CorridorBuilder2d.hpp>
#include <nav_msgs/Odometry.h>
#include <traj_optimizer/traj_optimizer.h>
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Core>
#define BUDGET 0.1
namespace plan_manage
{

    class PlanManager
    {
    public:
        PlanManager(){};
        void init(ros::NodeHandle &nh);

    private:
        bool hasTarget = false, hasOdom = false;
        Eigen::Vector3d targetPose, odom;
        geometry_msgs::Pose isEqualTarget, isEqualOdom;
        std::unique_ptr<visualization::Visualization> vis_tool;
        std::shared_ptr<Config> config_;
        std::unique_ptr<path_searching::KinoAstar> kino_path_finder_;
        map_util::OccMapUtil gridMap;
        se2Plan::Se2TrajOpt se2Opt;
        Eigen::Matrix<double, 2, 3> iniState2d, finState2d;
        Eigen::Vector3d iniState1d, finState1d;
        Eigen::MatrixXd initInnerPts2d;
        Eigen::VectorXd initInnerPts1d;
        std::vector<Eigen::MatrixXd> hPolys;
        void getGalaxConst(std::vector<Eigen::Vector3d> statelist);
        void process(const ros::TimerEvent &);
        void targetCallback(const geometry_msgs::PoseStamped &msg);
        void odomCallback(const nav_msgs::OdometryPtr &msg);
        void pclCallback(const sensor_msgs::PointCloud2 &pointcloud_map);
        bool checkReplan();
        void gridmapCallback(const nav_msgs::OccupancyGrid &msg);

        // 每次从map_server读到的原始栅格地图
        nav_msgs::OccupancyGrid origin_gridmap_;

        double pieceTime;
        /*ros related*/
        ros::Timer processTimer;
        ros::Subscriber targetSub, odomSub, pclSub, gridmap_sub_;
        ros::Publisher trajCmdPub, Dftpav_path_pub_, x_pub, y_pub, z_pub;
        ros::Publisher splicing_traj_pub_;

        PolyTrajOptimizer::Ptr ploy_traj_opt_;

        bool hasTraj = false;
        plan_utils::TrajectoryContainer trajContainer;
        Eigen::Vector4d iniFs, finFs;
        Eigen::Vector2d startPos, startVel, startAcc;
        double startYaw;
        bool useReplanState;

        // point_cloud_sub_ = nh.subscribe("/global_map", 10, &MapUtil::MapBuild, this);
        // static
        int tkReplan_num = 0;
        double startExeTime, endExeTime;

        // Event-driven replanning variables
        bool needReplan_;
        bool trackingErrorDetected_;
        bool obstacleChangeDetected_;
        sensor_msgs::PointCloud2 lastPointCloud_;
        bool hasLastPointCloud_;
        
        // Configuration parameters for event-driven replanning
        double obstacleChangeThreshold_;
        double trackingErrorThreshold_;
        double safetyCheckFrequency_;
        bool obstacleDetectionEnabled_;
        int replanTriggerCount_;
        
        // Performance monitoring
        double lastReplanTime_;
        int totalReplanCount_;
        std::string lastReplanReason_;
        
        // Trajectory splicing variables
        bool needTrajectorySpicing_;
        double splicingTime_;
        Eigen::Vector3d splicingStartState_;
        Eigen::Vector3d splicingStartVel_;
        double replanStartTime_;
        Eigen::Vector3d robotStateAtReplanStart_;
        
        // Adjustable splicing parameters
        double splice_time_param_;     // Time into new trajectory for splice point (adjustable)
        double splice_duration_;       // Duration of splicing trajectory (adjustable)
        
        // Timing statistics
        struct ReplanningStats {
            double frontend_time_ms;
            double optimization_time_ms;
            double splicing_time_ms;
            double total_time_ms;
            int replan_count;
        } replan_stats_;
        
        // Helper functions for event-driven replanning
        bool detectObstacleChange(const sensor_msgs::PointCloud2 &currentCloud);
        void resetReplanFlags();
        double calculatePointCloudDifference(const sensor_msgs::PointCloud2 &cloud1, 
                                           const sensor_msgs::PointCloud2 &cloud2);
        void printPerformanceStatistics();
        void publishDebugInfo();
        
        // Trajectory splicing functions
        plan_utils::TrajectoryContainer generateSplicingTrajectory(
            const Eigen::Vector3d& current_state, 
            const Eigen::Vector3d& current_vel,
            const Eigen::Vector3d& target_state,
            const Eigen::Vector3d& target_vel,
            double splicing_time);
        Eigen::Vector3d findSplicingPoint(double time_offset);
        void recordReplanningTime(const std::string& phase, double time_ms);
        
        // Enhanced trajectory splicing functions
        double calculateOptimalSplicingTime(const Eigen::Vector2d& current_vel);
        plan_utils::TrajectoryContainer createPolynomialTrajectory(
            const Eigen::VectorXd& coeff_x, 
            const Eigen::VectorXd& coeff_y, 
            double duration);
        plan_utils::TrajectoryContainer performSmartTrajectorySpicing(
            const plan_utils::TrajectoryContainer& newTrajContainer,
            double replan_duration);
        
        // Visualization functions for trajectory splicing
        void publishSplicingTrajectory(const Eigen::VectorXd& coeff_x, const Eigen::VectorXd& coeff_y, 
                                     double duration, const Eigen::Vector3d& start_pos);
    };
}

#endif
