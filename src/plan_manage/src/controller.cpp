#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <nav_msgs/Path.h>
#include <algorithm>
#include <vector>
#include "kinematics_simulator/ControlCmd.h"

// PID Controller Class
class PIDController {
public:
    PIDController(double kp, double ki, double kd, double max_integral, double max_output)
        : kp_(kp), ki_(ki), kd_(kd), max_integral_(max_integral), max_output_(max_output),
          integral_(0.0), last_error_(0.0), initialized_(false) {}
    
    double compute(double error, double dt) {
        if (!initialized_) {
            last_error_ = error;
            initialized_ = true;
        }
        
        // Integral term with windup protection
        integral_ += error * dt;
        integral_ = std::max(std::min(integral_, max_integral_), -max_integral_);
        
        // Derivative term
        double derivative = (error - last_error_) / dt;
        
        // PID output
        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        output = std::max(std::min(output, max_output_), -max_output_);
        
        last_error_ = error;
        return output;
    }
    
    void reset() {
        integral_ = 0.0;
        last_error_ = 0.0;
        initialized_ = false;
    }
    
    void setParams(double kp, double ki, double kd) {
        kp_ = kp; ki_ = ki; kd_ = kd;
    }

private:
    double kp_, ki_, kd_;
    double max_integral_, max_output_;
    double integral_, last_error_;
    bool initialized_;
};

// Enhanced Controller Class
class EnhancedController {
public:
    EnhancedController(ros::NodeHandle& nh) : nh_(nh), current_path_index_(0) {
        loadParameters();
        initializePIDControllers();
        
        // Initialize subscribers and publishers
        path_sub_ = nh_.subscribe("/Dftpav_path", 1, &EnhancedController::pathCallback, this);
        odom_sub_ = nh_.subscribe("/ugv/odometry", 1, &EnhancedController::odomCallback, this);
        goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &EnhancedController::goalCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/ugv/command", 1);
        
        // Control timer
        control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_frequency_), 
                                       &EnhancedController::controlLoop, this);
        
        state_ = WAITING;
        last_time_ = ros::Time::now();
        
        ROS_INFO("Enhanced PID Controller initialized");
    }

private:
    enum State { WAITING = 0, RECEIVING, RUNNING };
    
    ros::NodeHandle& nh_;
    ros::Subscriber path_sub_, odom_sub_, goal_sub_;
    ros::Publisher cmd_pub_;
    ros::Timer control_timer_;
    
    // PID Controllers
    std::unique_ptr<PIDController> lateral_pid_;
    std::unique_ptr<PIDController> longitudinal_pid_;
    std::unique_ptr<PIDController> heading_pid_;
    
    // Control parameters
    double control_frequency_;
    double max_linear_vel_, max_angular_vel_;
    double max_linear_acc_, max_angular_acc_;
    double lookahead_distance_;
    double goal_tolerance_;
    double velocity_smoothing_factor_;
    double angular_velocity_factor_;
    
    // PID parameters (for loading from config)
    double lateral_kp_, lateral_ki_, lateral_kd_;
    double longitudinal_kp_, longitudinal_ki_, longitudinal_kd_;
    double heading_kp_, heading_ki_, heading_kd_;
    
    // State variables
    State state_;
    nav_msgs::Path current_path_;
    geometry_msgs::Pose current_pose_;
    geometry_msgs::Pose goal_pose_;
    geometry_msgs::Twist last_cmd_;
    ros::Time last_time_;
    size_t current_path_index_;
    
    // Performance monitoring
    double total_tracking_error_;
    int control_iterations_;
    
    void loadParameters();
    void initializePIDControllers();
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void controlLoop(const ros::TimerEvent& event);
    
    geometry_msgs::Pose findLookaheadPoint();
    double calculateLateralError(const geometry_msgs::Pose& target);
    double calculateLongitudinalError(const geometry_msgs::Pose& target);
    double calculateHeadingError(const geometry_msgs::Pose& target);
    double normalizeAngle(double angle);
    bool isPoseEqual(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b, 
                    double pos_tol, double ang_tol);
    geometry_msgs::Twist smoothVelocity(const geometry_msgs::Twist& cmd);
};

// Global variables for compatibility
EnhancedController* controller_ptr = nullptr;

void EnhancedController::loadParameters() {
    // Simplified PID parameters
    nh_.param("lateral_kp", lateral_kp_, 2.0);
    nh_.param("lateral_ki", lateral_ki_, 0.1);
    nh_.param("lateral_kd", lateral_kd_, 0.5);
    
    // Use lateral PID for longitudinal (simplified)
    longitudinal_kp_ = lateral_kp_;
    longitudinal_ki_ = lateral_ki_;
    longitudinal_kd_ = lateral_kd_;
    
    nh_.param("heading_kp", heading_kp_, 3.0);
    nh_.param("heading_ki", heading_ki_, 0.2);
    nh_.param("heading_kd", heading_kd_, 0.8);
    
    // Simplified control parameters
    nh_.param("max_linear_velocity", max_linear_vel_, 1.0);
    nh_.param("max_angular_velocity", max_angular_vel_, 1.5);
    nh_.param("lookahead_distance", lookahead_distance_, 0.5);
    nh_.param("control_frequency", control_frequency_, 20.0);
    
    // Set reasonable defaults for other parameters
    max_linear_acc_ = 2.0;
    max_angular_acc_ = 3.0;
    velocity_smoothing_factor_ = 0.8;
    angular_velocity_factor_ = 0.5;
    goal_tolerance_ = 0.1;
    
    // Initialize performance monitoring
    total_tracking_error_ = 0.0;
    control_iterations_ = 0;
    
    ROS_INFO("Controller parameters loaded: lateral_kp=%.1f, heading_kp=%.1f, max_vel=%.1f", 
             lateral_kp_, heading_kp_, max_linear_vel_);
}

void EnhancedController::initializePIDControllers() {
    lateral_pid_.reset(new PIDController(lateral_kp_, lateral_ki_, lateral_kd_, 1.0, 2.0));
    longitudinal_pid_.reset(new PIDController(longitudinal_kp_, longitudinal_ki_, longitudinal_kd_, 0.5, 1.5));
    heading_pid_.reset(new PIDController(heading_kp_, heading_ki_, heading_kd_, 0.5, 2.0));
}

void EnhancedController::pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    current_path_ = *msg;
    current_path_index_ = 0;
    
    if (state_ == RECEIVING) {
        state_ = RUNNING;
        ROS_INFO("Received trajectory with %zu points, starting control", current_path_.poses.size());
        
        // Reset PID controllers for new trajectory
        lateral_pid_->reset();
        longitudinal_pid_->reset();
        heading_pid_->reset();
    }
}

void EnhancedController::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_pose_ = msg->pose.pose;
    
    // Check if goal is reached
    if (isPoseEqual(goal_pose_, current_pose_, goal_tolerance_, 0.3)) {
        ROS_INFO("Goal reached! Stopping controller.");
        state_ = WAITING;
        
        // Publish zero velocity
        geometry_msgs::Twist stop_cmd;
        cmd_pub_.publish(stop_cmd);
    }
}

void EnhancedController::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    goal_pose_ = msg->pose;
    state_ = RECEIVING;
    ROS_INFO("New goal received, waiting for trajectory...");
}

void EnhancedController::controlLoop(const ros::TimerEvent& event) {
    if (state_ != RUNNING || current_path_.poses.empty()) {
        // Publish zero velocity when not running
        geometry_msgs::Twist stop_cmd;
        cmd_pub_.publish(stop_cmd);
        return;
    }
    
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time_).toSec();
    if (dt <= 0) dt = 1.0 / control_frequency_;
    last_time_ = current_time;
    
    // Find lookahead point
    geometry_msgs::Pose target_pose = findLookaheadPoint();
    
    // Calculate errors
    double lateral_error = calculateLateralError(target_pose);
    double longitudinal_error = calculateLongitudinalError(target_pose);
    double heading_error = calculateHeadingError(target_pose);
    
    // Compute PID outputs
    double lateral_output = lateral_pid_->compute(lateral_error, dt);
    double longitudinal_output = longitudinal_pid_->compute(longitudinal_error, dt);
    double heading_output = heading_pid_->compute(heading_error, dt);
    
    // Combine outputs to generate control commands
    geometry_msgs::Twist cmd_vel;
    
    // Linear velocity based on longitudinal error and heading alignment
    double heading_factor = 1.0 - angular_velocity_factor_ * std::abs(heading_error) / M_PI;
    cmd_vel.linear.x = longitudinal_output * heading_factor;
    
    // Angular velocity from heading control and lateral correction
    cmd_vel.angular.z = heading_output + lateral_output;
    
    // Apply velocity limits
    cmd_vel.linear.x = std::max(std::min(cmd_vel.linear.x, max_linear_vel_), -max_linear_vel_);
    cmd_vel.angular.z = std::max(std::min(cmd_vel.angular.z, max_angular_vel_), -max_angular_vel_);
    
    // Apply velocity smoothing
    cmd_vel = smoothVelocity(cmd_vel);
    
    // Publish command
    cmd_pub_.publish(cmd_vel);
    
    // Update performance monitoring
    double tracking_error = std::sqrt(lateral_error * lateral_error + longitudinal_error * longitudinal_error);
    total_tracking_error_ += tracking_error;
    control_iterations_++;
    
    // Remove debug output to reduce terminal clutter
    // Debug info removed for cleaner output
}

geometry_msgs::Pose EnhancedController::findLookaheadPoint() {
    if (current_path_.poses.empty()) {
        return current_pose_;
    }
    
    // Find the closest point on path
    double min_distance = std::numeric_limits<double>::max();
    size_t closest_index = current_path_index_;
    
    for (size_t i = current_path_index_; i < current_path_.poses.size(); ++i) {
        double dx = current_path_.poses[i].pose.position.x - current_pose_.position.x;
        double dy = current_path_.poses[i].pose.position.y - current_pose_.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        if (distance < min_distance) {
            min_distance = distance;
            closest_index = i;
        }
    }
    
    current_path_index_ = closest_index;
    
    // Find lookahead point
    for (size_t i = closest_index; i < current_path_.poses.size(); ++i) {
        double dx = current_path_.poses[i].pose.position.x - current_pose_.position.x;
        double dy = current_path_.poses[i].pose.position.y - current_pose_.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        if (distance >= lookahead_distance_) {
            return current_path_.poses[i].pose;
        }
    }
    
    // If no lookahead point found, return the last point
    return current_path_.poses.back().pose;
}

double EnhancedController::calculateLateralError(const geometry_msgs::Pose& target) {
    // Calculate lateral error (cross-track error)
    double dx = target.position.x - current_pose_.position.x;
    double dy = target.position.y - current_pose_.position.y;
    
    // Get current heading
    tf::Quaternion q;
    tf::quaternionMsgToTF(current_pose_.orientation, q);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    // Calculate lateral error (perpendicular to heading direction)
    double lateral_error = -dx * std::sin(yaw) + dy * std::cos(yaw);
    return lateral_error;
}

double EnhancedController::calculateLongitudinalError(const geometry_msgs::Pose& target) {
    // Calculate longitudinal error (along heading direction)
    double dx = target.position.x - current_pose_.position.x;
    double dy = target.position.y - current_pose_.position.y;
    
    // Get current heading
    tf::Quaternion q;
    tf::quaternionMsgToTF(current_pose_.orientation, q);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    // Calculate longitudinal error (along heading direction)
    double longitudinal_error = dx * std::cos(yaw) + dy * std::sin(yaw);
    return longitudinal_error;
}

double EnhancedController::calculateHeadingError(const geometry_msgs::Pose& target) {
    // Calculate desired heading to target
    double dx = target.position.x - current_pose_.position.x;
    double dy = target.position.y - current_pose_.position.y;
    double target_heading = std::atan2(dy, dx);
    
    // Get current heading
    tf::Quaternion q;
    tf::quaternionMsgToTF(current_pose_.orientation, q);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    // Calculate heading error
    double heading_error = normalizeAngle(target_heading - yaw);
    return heading_error;
}

double EnhancedController::normalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

bool EnhancedController::isPoseEqual(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b, 
                                   double pos_tol, double ang_tol) {
    double dx = a.position.x - b.position.x;
    double dy = a.position.y - b.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    tf::Quaternion qa, qb;
    tf::quaternionMsgToTF(a.orientation, qa);
    tf::quaternionMsgToTF(b.orientation, qb);
    double angle_diff = std::abs(qa.angleShortestPath(qb));
    
    return (distance <= pos_tol) && (angle_diff <= ang_tol);
}

geometry_msgs::Twist EnhancedController::smoothVelocity(const geometry_msgs::Twist& cmd) {
    geometry_msgs::Twist smoothed_cmd;
    
    // Apply velocity smoothing
    smoothed_cmd.linear.x = velocity_smoothing_factor_ * last_cmd_.linear.x + 
                           (1.0 - velocity_smoothing_factor_) * cmd.linear.x;
    smoothed_cmd.angular.z = velocity_smoothing_factor_ * last_cmd_.angular.z + 
                            (1.0 - velocity_smoothing_factor_) * cmd.angular.z;
    
    // Apply acceleration limits
    double dt = 1.0 / control_frequency_;
    double linear_acc = (smoothed_cmd.linear.x - last_cmd_.linear.x) / dt;
    double angular_acc = (smoothed_cmd.angular.z - last_cmd_.angular.z) / dt;
    
    if (std::abs(linear_acc) > max_linear_acc_) {
        smoothed_cmd.linear.x = last_cmd_.linear.x + 
                               std::copysign(max_linear_acc_ * dt, linear_acc);
    }
    
    if (std::abs(angular_acc) > max_angular_acc_) {
        smoothed_cmd.angular.z = last_cmd_.angular.z + 
                                std::copysign(max_angular_acc_ * dt, angular_acc);
    }
    
    last_cmd_ = smoothed_cmd;
    return smoothed_cmd;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh("~");
    
    EnhancedController controller(nh);
    controller_ptr = &controller;
    
    ROS_INFO("Enhanced PID Controller started");
    ros::spin();
    
    return 0;
}