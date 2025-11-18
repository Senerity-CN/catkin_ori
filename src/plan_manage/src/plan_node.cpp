#include <plan_manage/plan_manage.h>
#include <ros/ros.h>
#include <ros/console.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_planning_node");
    ros::NodeHandle nh("~");
    plan_manage::PlanManager manager;
    manager.init(nh);
    ros::spin();
    return 0;
}
