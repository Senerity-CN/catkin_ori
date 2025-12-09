#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cmath>

const double PI = 3.14159265358979323846;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_node");
    ros::NodeHandle nh;

    ros::Publisher laser_pub = nh.advertise<sensor_msgs::PointCloud2>("/ugv/laser", 10);

    ros::Rate loop_rate(10); // 10 Hz

    double radius = 4.0;
    double center_x = 15.0;
    double center_y = 7.0;
    double angle = 0.0;
    double distance = 2.0;      // 障碍物在x方向上的运动距离
    double speed = 0.01;         // 运动速度
    double pos = center_y;      // 当前障碍物位置
    bool moving_forward = true; // 障碍物是否向前运动

    while (ros::ok())
    {
        // 创建PointCloud2消息
        sensor_msgs::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.frame_id = "world";
        cloud_msg.height = 1;  // 无序点云
        cloud_msg.width = 100; // 假设障碍物有100个点

        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        modifier.resize(cloud_msg.width * cloud_msg.height);

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");

        // 生成障碍物的点云数据
        for (size_t i = 0; i < cloud_msg.width; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
        {
            // // 圆 障碍物中心位置
            // double obstacle_x = center_x + radius * std::cos(angle);
            // double obstacle_y = center_y + radius * std::sin(angle);

            // 直线 障碍物位置
            double obstacle_x = center_x;
            double obstacle_y = pos;

            // 在障碍物中心附近随机生成点云
            *iter_x = obstacle_x + static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 0.2 - 0.1;
            *iter_y = obstacle_y + static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 0.2 - 0.1;
            *iter_z = 0.0; // 假设激光雷达在平面上

            // 给点云点一个固定的颜色
            *iter_r = 255;
            *iter_g = 0;
            *iter_b = 0;
        }

        // // 更新角度，使障碍物沿着圆周移动
        // angle += 0.01;
        // if (angle > 2.0 * PI)
        // {
        //     angle -= 2.0 * PI;
        // }

        // 更新位置，使障碍物沿着直线来回移动
        if (moving_forward)
        {
            pos += speed;
            if (pos >= center_y + distance)
            {
                moving_forward = false;
            }
        }
        else
        {
            pos -= speed;
            if (pos <= center_y)
            {
                moving_forward = true;
            }
        }

        laser_pub.publish(cloud_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}