#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <nav_msgs/Path.h>
#include "kinematics_simulator/ControlCmd.h"

double v_max = 1.0, w_max = 1.0, a_v_max = 2.0, a_w_max = 2.0;
double v = 0, w = 0, t = 0, seg = 0;
enum State
{
    kWaiting = 0,
    kReciving,
    kRunning
};
enum State state = kWaiting;
geometry_msgs::Pose goal;
geometry_msgs::Pose target;
nav_msgs::Path path;
geometry_msgs::Pose current_p;

bool isPoseEqual(const geometry_msgs::Pose &a, const geometry_msgs::Pose &b, double p_eps, double rad_eps)
{
    tf::Quaternion qa, qb;
    tf::quaternionMsgToTF(a.orientation, qa);
    tf::quaternionMsgToTF(b.orientation, qb);
    if (std::abs(a.position.x - b.position.x) <= std::abs(p_eps) && std::abs(a.position.y - b.position.y) <= std::abs(p_eps) &&
        std::abs(qa.angleShortestPath(qb)) <= std::abs(rad_eps))
    {
        return true;
    }
    else
        return false;
}

double getDistanceSquare(const geometry_msgs::Pose &a, const geometry_msgs::Pose &b)
{
    return (a.position.x - b.position.x) * (a.position.x - b.position.x) + (a.position.y - b.position.y) * (a.position.y - b.position.y);
}

void TrajCallback(nav_msgs::Path msg)
{
    // ROS_INFO("HAVE dftpav path message!");
    path = msg;
    if (state == kReciving)
    {
        ROS_WARN("state = kRunning; ");
        state = kRunning;
    }
}

void OdometryCallback(const nav_msgs::OdometryConstPtr msg)
{
    // v = msg->twist.twist.linear.x;
    // w = msg->twist.twist.angular.z;
    // ROS_INFO("HAVE odom message!");
    current_p.position.x = msg->pose.pose.position.x;
    current_p.position.y = msg->pose.pose.position.y;
    current_p.orientation = msg->pose.pose.orientation;
    if (isPoseEqual(goal, current_p, 0.3, 0.3))
    {
        ROS_WARN("The robot has reached the goal point. ");
        state = kWaiting;
    }
}

void GoalCallback(geometry_msgs::PoseStamped::ConstPtr msg)
{
    // ROS_INFO("HAVE goal message!");
    state = kReciving;
    goal = msg->pose;
}

int main(int argc, char **args)
{
    ros::init(argc, args, "controller node");
    ros::NodeHandle nh("~");

    ros::Subscriber Dftpav_sub = nh.subscribe("/Dftpav_path", 1, TrajCallback);
    ros::Subscriber odom_sub = nh.subscribe("/ugv/odometry", 1, OdometryCallback);
    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 1, GoalCallback);
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/ugv/command", 1);

    double rate = 10;
    ros::Rate loop_rate = rate;
    geometry_msgs::Twist cmd_vel;
    kinematics_simulator::ControlCmd cmd;
    while (ros::ok())
    {
        ros::spinOnce();

        if (state != kRunning)
        {
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            cmd_pub.publish(cmd_vel);
            loop_rate.sleep();
            continue;
        }

        int i = 2;
        geometry_msgs::Pose target = path.poses[i].pose;
        if (isPoseEqual(target, current_p, 0.003, 0.3))
        {
            target = path.poses[i + 1].pose;
            i = i + 1;
        }

        tf::Quaternion q(
            current_p.orientation.x,
            current_p.orientation.y,
            current_p.orientation.z,
            current_p.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        double dx = target.position.x - current_p.position.x;
        double dy = target.position.y - current_p.position.y;
        double target_yaw = std::atan2(dy, dx);
        double dfai = target_yaw - yaw;

        // 规范化角度到[-pi, pi]
        dfai = std::atan2(std::sin(dfai), std::cos(dfai));

        // 判断是否需要倒车
        bool reverse = std::abs(dfai) > M_PI / 2;
        if (reverse)
        {
            dfai = std::atan2(std::sin(dfai + M_PI), std::cos(dfai + M_PI));
            v = -1.0 * v_max * (1 - 0.92 * std::abs(dfai) / w_max);
        }
        else
        {
            v = 1.0 * v_max * (1 - 0.92 * std::abs(dfai) / w_max);
        }

        double w = std::max(std::min(2 * dfai, w_max), -w_max);

        cmd_vel.linear.x = v;
        cmd_vel.angular.z = w;
        cmd_pub.publish(cmd_vel);

        ROS_INFO("pub v=%lf, w=%lf", v, w);
        loop_rate.sleep();
    }
    ros::shutdown();
}