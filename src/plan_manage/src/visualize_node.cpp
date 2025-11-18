#include<plan_manage/plan_manage.h>
#include <ros/ros.h>
#include <ros/console.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visualize_node");
    ros::NodeHandle nh("~");

    // plan_manage::PlanManager manager;
    // manager.init(nh);
    ros::Publisher vis1, vis2, vis3, vis4;
    vis1 = nh.advertise<nav_msgs::Path>("/vis1", 1);
    vis2 = nh.advertise<nav_msgs::Path>("/vis2", 1);
    vis3 = nh.advertise<nav_msgs::Path>("/vis3", 1);
    vis4 = nh.advertise<nav_msgs::Path>("/vis4", 1);
    nav_msgs::Path vis_pathmsg1, vis_pathmsg2, vis_pathmsg3, vis_pathmsg4;

    {
        std::ifstream fin;
        fin.open("/home/han/2023codes/rebuttle/zero.txt");
        string str;
        while (!fin.eof())
        {
            getline(fin, str);
            double x;
            stringstream ss(str);  //建立stringstream对象，初始化流内容为line所代表的字符串  
            int i = 0;
            Eigen::Vector2d pos;
            while(ss>>x)            //从line中一次读取数字存入x  
                pos[i++] = x;
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "world";
            pose.pose.position.x = pos[0];//x
            pose.pose.position.y = pos[1];//y
            pose.pose.position.z = 2.2;
            pose.header.stamp =  ros::Time().now();//time
            vis_pathmsg1.poses.push_back(pose);
        }
        vis_pathmsg1.header.frame_id = "world";
    }

    {
        std::ifstream fin;
        fin.open("/home/han/2023codes/rebuttle/small.txt");
        string str;
        while (!fin.eof())
        {
            getline(fin, str);
            double x;
            stringstream ss(str);  //建立stringstream对象，初始化流内容为line所代表的字符串  
            int i = 0;
            Eigen::Vector2d pos;
            while(ss>>x)            //从line中一次读取数字存入x  
                pos[i++] = x;
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "world";
            pose.pose.position.x = pos[0];//x
            pose.pose.position.y = pos[1];//y
            pose.pose.position.z = 2.2;
            pose.header.stamp =  ros::Time().now();//time
            vis_pathmsg2.poses.push_back(pose);
        }
        vis_pathmsg2.header.frame_id = "world";
    }

    {
        std::ifstream fin;
        fin.open("/home/han/2023codes/rebuttle/middle.txt");
        string str;
        while (!fin.eof())
        {
            getline(fin, str);
            double x;
            stringstream ss(str);  //建立stringstream对象，初始化流内容为line所代表的字符串  
            int i = 0;
            Eigen::Vector2d pos;
            while(ss>>x)            //从line中一次读取数字存入x  
                pos[i++] = x;
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "world";
            pose.pose.position.x = pos[0];//x
            pose.pose.position.y = pos[1];//y
            pose.pose.position.z = 2.2;
            pose.header.stamp =  ros::Time().now();//time
            vis_pathmsg3.poses.push_back(pose);
        }
        vis_pathmsg3.header.frame_id = "world";
    }

    {
        std::ifstream fin;
        fin.open("/home/han/2023codes/rebuttle/high.txt");
        string str;
        while (!fin.eof())
        {
            getline(fin, str);
            double x;
            stringstream ss(str);  //建立stringstream对象，初始化流内容为line所代表的字符串  
            int i = 0;
            Eigen::Vector2d pos;
            while(ss>>x)            //从line中一次读取数字存入x  
                pos[i++] = x;
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "world";
            pose.pose.position.x = pos[0];//x
            pose.pose.position.y = pos[1];//y
            pose.pose.position.z = 2.2;
            pose.header.stamp =  ros::Time().now();//time
            vis_pathmsg4.poses.push_back(pose);
        }
        vis_pathmsg4.header.frame_id = "world";
    }
    while(ros::ok()){
        vis1.publish(vis_pathmsg1);
        vis2.publish(vis_pathmsg2);
        vis3.publish(vis_pathmsg3);
        vis4.publish(vis_pathmsg4);
        ros::Duration(0.1).sleep();
    }

    ros::spin();
    return 0;
}
