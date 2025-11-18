#include "ros/ros.h"
#include "kinematics_simulator/ControlCmd.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<kinematics_simulator::ControlCmd>("chatter", 1000);
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        kinematics_simulator::ControlCmd msg;
        msg.longitude_acc = 1;
        msg.steer_vel = 0.2;
        ROS_INFO("%s", msg.data.c_str());
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}