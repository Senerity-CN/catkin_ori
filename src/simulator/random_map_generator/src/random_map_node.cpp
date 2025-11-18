#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <random>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

// pcl
pcl::PointCloud<pcl::PointXYZ> cloud_map;
pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
pcl::PointXYZ sensor_pose;
vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;
vector<Eigen::Vector2d> cors;

// random
// random_device rd;
// default_random_engine eng(rd());
default_random_engine eng;
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;
uniform_real_distribution<double> rand_w;
uniform_real_distribution<double> rand_theta;
uniform_real_distribution<double> rand_radius;

// ros
ros::Publisher local_map_pub;
ros::Publisher global_map_pub;
ros::Publisher mesh_map_pub;
ros::Subscriber odom_sub;
ros::Timer sensor_timer;
sensor_msgs::PointCloud2 global_msg;
visualization_msgs::Marker mesh_msg;

// params
bool has_odom = false;
bool has_map = false;
vector<int> obs_num = {1, 1, 1};
double resolution = 0.6;
double size_x = 30.0;
double size_y = 30.0;
double min_width = 0.3;
double min_dis = 0.3;
double max_width = 0.8;
double sensor_rate = 10.0;
double sensor_range = 5.0;

// laser
constexpr int LINE_NUM = 1024;
double laser_res = 2.0 * M_PI / LINE_NUM;
Eigen::VectorXi idx_map = Eigen::VectorXi::Constant(LINE_NUM, -1);
Eigen::VectorXd dis_map = Eigen::VectorXd::Constant(LINE_NUM, 9999.0);

bool crossBoolean2(Eigen::Vector2d a, Eigen::Vector2d b) {
  return (a(0) * b(1) - b(0) * a(1) > 0);
}

pcl::PointCloud<pcl::PointXYZ>
fillConvexPolygon(vector<Eigen::Vector2d> poly_vs) {
  pcl::PointCloud<pcl::PointXYZ> cloud_polygon;

  if (poly_vs.size() < 3)
    return cloud_polygon;

  double down = 9999.0;
  double up = -9999.0;
  double left = 9999.0;
  double right = -9999.0;

  // AABB box
  for (size_t i = 0; i < poly_vs.size(); i++) {
    if (poly_vs[i][0] > right)
      right = poly_vs[i][0];
    if (poly_vs[i][0] < left)
      left = poly_vs[i][0];
    if (poly_vs[i][1] > up)
      up = poly_vs[i][1];
    if (poly_vs[i][1] < down)
      down = poly_vs[i][1];
  }

  for (double x = left; x < right + resolution; x += resolution) {
    for (double y = down; y < up + resolution; y += resolution) {
      bool in_poly = false;
      Eigen::Vector2d O(x, y);

      for (size_t i = 0; i < poly_vs.size() - 2; i++) {
        // if a point is in triangle
        Eigen::Vector2d A = poly_vs[0];
        Eigen::Vector2d B = poly_vs[i + 1];
        Eigen::Vector2d C = poly_vs[i + 2];
        if (crossBoolean2(B - A, O - A) && crossBoolean2(C - B, O - B) &&
            crossBoolean2(A - C, O - C)) {
          in_poly = true;
          break;
        }
      }

      if (in_poly) {
        pcl::PointXYZ pt;
        pt.x = x;
        pt.y = y;
        pt.z = 0.0;
        cloud_polygon.push_back(pt);
      }
    }
  }

  return cloud_polygon;
}

pair<vector<Eigen::Vector2d>, pcl::PointCloud<pcl::PointXYZ>>
generatePolygon(int K) {
  pcl::PointCloud<pcl::PointXYZ> cloud_polygon;

  rand_w = uniform_real_distribution<double>(min_width, max_width);
  rand_theta = uniform_real_distribution<double>(-M_PI, M_PI);

  double radius = rand_w(eng);
  double theta = rand_theta(eng);
  double angle_res = 2.0 * M_PI / K;
  double small_r = radius * sin(angle_res / 2.0);

  rand_radius = uniform_real_distribution<double>(-small_r, small_r);

  vector<Eigen::Vector2d> vs;
  for (int i = 0; i < K; i++) {
    double a = angle_res * i + theta;
    double delta_theta = rand_theta(eng);
    double delta_radius = rand_radius(eng);
    Eigen::Vector2d p(cos(a) * radius + cos(a + delta_theta) * delta_radius,
                      sin(a) * radius + sin(a + delta_theta) * delta_radius);
    vs.push_back(p);
  }
  cloud_polygon = fillConvexPolygon(vs);

  return std::make_pair(vs, cloud_polygon);
}
bool checkDis(double x, double y) {
  bool result = false;
  Eigen::Vector2d tmp(x, y);
  for (const auto pt : cors) {
    if ((tmp - pt).norm() < min_dis) {
      result = true;
      return result;
    }
  }
  return result;
}
void addObs() {
  int number = 5;
  double width = 10.0;
  double baseS1 = 0.0;
  double height1 = -0.8;
  double baseS2 = width / 2.0;
  double height2 = -height1;
  for (int i = 0; i < number; i++) {
    pcl::PointXYZ pt_random;
    {
      pcl::PointCloud<pcl::PointXYZ> cloud_polygon;
      vector<Eigen::Vector2d> vs;

      Eigen::Vector2d p1(baseS1 + i * width, 5.0);
      vs.push_back(p1);
      Eigen::Vector2d p2(baseS1 + i * width + width / 2.0, height1);
      vs.push_back(p2);
      Eigen::Vector2d p3(baseS1 + i * width + width, 5.0);
      vs.push_back(p3);
      cloud_polygon = fillConvexPolygon(vs);
      for (size_t i = 0; i < cloud_polygon.points.size(); i++) {
        pt_random.x = cloud_polygon.points[i].x;
        pt_random.y = cloud_polygon.points[i].y;
        pt_random.z = 0.0;
        cloud_map.points.push_back(pt_random);
      }
      vector<Eigen::Vector2d> vector_polygon = vs;
      geometry_msgs::Point init_p;
      init_p.x = vector_polygon[0].x();
      init_p.y = vector_polygon[0].y();
      init_p.z = 0.0;
      for (int i = 1; i < 2; i++) {
        mesh_msg.points.push_back(init_p);
        geometry_msgs::Point p;
        p.x = vector_polygon[i].x();
        p.y = vector_polygon[i].y();
        p.z = 0.0;
        mesh_msg.points.push_back(p);
        p.x = vector_polygon[i + 1].x();
        p.y = vector_polygon[i + 1].y();
        p.z = 0.0;
        mesh_msg.points.push_back(p);
      }
    }
    {
      pcl::PointCloud<pcl::PointXYZ> cloud_polygon;
      vector<Eigen::Vector2d> vs;

      Eigen::Vector2d p1(baseS2 + i * width, -5.0);
      vs.push_back(p1);
      Eigen::Vector2d p2(baseS2 + i * width + width, -5.0);
      vs.push_back(p2);
      Eigen::Vector2d p3(baseS2 + i * width + width / 2.0, height2);
      vs.push_back(p3);
      cloud_polygon = fillConvexPolygon(vs);
      for (size_t i = 0; i < cloud_polygon.points.size(); i++) {
        pt_random.x = cloud_polygon.points[i].x;
        pt_random.y = cloud_polygon.points[i].y;
        pt_random.z = 0.0;
        cloud_map.points.push_back(pt_random);
      }
      vector<Eigen::Vector2d> vector_polygon = vs;
      geometry_msgs::Point init_p;
      init_p.x = vector_polygon[0].x();
      init_p.y = vector_polygon[0].y();
      init_p.z = 0.0;
      for (int i = 1; i < 2; i++) {
        mesh_msg.points.push_back(init_p);
        geometry_msgs::Point p;
        p.x = vector_polygon[i].x();
        p.y = vector_polygon[i].y();
        p.z = 0.0;
        mesh_msg.points.push_back(p);
        p.x = vector_polygon[i + 1].x();
        p.y = vector_polygon[i + 1].y();
        p.z = 0.0;
        mesh_msg.points.push_back(p);
      }
    }
  }
}

void generateMap() {
  cors.clear();
  pcl::PointXYZ pt_random;

  rand_x = uniform_real_distribution<double>(-size_x / 2.0, size_x / 2.0);
  rand_y = uniform_real_distribution<double>(-size_y / 2.0, size_y / 2.0);

  // generate polygon obs
  for (int k = 0; k < obs_num.size(); k++) {
    for (int i = 0; i < obs_num[k]; i++) {
      double x, y;
      x = rand_x(eng);
      y = rand_y(eng);

      if (sqrt(pow(x, 2) + pow(y, 2)) < 1.5) {
        i--;
        continue;
      }

      x = floor(x / resolution) * resolution + resolution / 2.0;
      y = floor(y / resolution) * resolution + resolution / 2.0;
      if (checkDis(x, y)) {
        i--;
        continue;
      }
      cors.push_back(Eigen::Vector2d(x, y));
      pair<vector<Eigen::Vector2d>, pcl::PointCloud<pcl::PointXYZ>>
          cloud_polygon = generatePolygon(k + 3);
      for (size_t p = 0; p < cloud_polygon.second.points.size(); p++) {
        pt_random.x = cloud_polygon.second.points[p].x + x;
        pt_random.y = cloud_polygon.second.points[p].y + y;
        pt_random.z = 0.0;
        cloud_map.points.push_back(pt_random);
      }

      vector<Eigen::Vector2d> vector_polygon = cloud_polygon.first;
      geometry_msgs::Point init_p;
      init_p.x = vector_polygon[0].x() + x;
      init_p.y = vector_polygon[0].y() + y;
      init_p.z = 0.0;
      for (int m = 1; m < k + 2; m++) {
        mesh_msg.points.push_back(init_p);
        geometry_msgs::Point p;
        p.x = vector_polygon[m].x() + x;
        p.y = vector_polygon[m].y() + y;
        p.z = 0.0;
        mesh_msg.points.push_back(p);
        p.x = vector_polygon[m + 1].x() + x;
        p.y = vector_polygon[m + 1].y() + y;
        p.z = 0.0;
        mesh_msg.points.push_back(p);
      }
    }
  }
  {
    pcl::PointCloud<pcl::PointXYZ> cloud_polygon;
    vector<Eigen::Vector2d> vs;
    Eigen::Vector2d p1(0.0, 6.0);
    vs.push_back(p1);
    Eigen::Vector2d p2(0.0, 5.0);
    vs.push_back(p2);
    Eigen::Vector2d p3(60.0, 5.0);
    vs.push_back(p3);
    Eigen::Vector2d p4(60.0, 6.0);
    vs.push_back(p4);
    cloud_polygon = fillConvexPolygon(vs);
    for (size_t i = 0; i < cloud_polygon.points.size(); i++) {
      pt_random.x = cloud_polygon.points[i].x;
      pt_random.y = cloud_polygon.points[i].y;
      pt_random.z = 0.0;
      cloud_map.points.push_back(pt_random);
    }
    vector<Eigen::Vector2d> vector_polygon = vs;
    geometry_msgs::Point init_p;
    init_p.x = vector_polygon[0].x();
    init_p.y = vector_polygon[0].y();
    init_p.z = 0.0;
    for (int i = 1; i < 3; i++) {
      mesh_msg.points.push_back(init_p);
      geometry_msgs::Point p;
      p.x = vector_polygon[i].x();
      p.y = vector_polygon[i].y();
      p.z = 0.0;
      mesh_msg.points.push_back(p);
      p.x = vector_polygon[i + 1].x();
      p.y = vector_polygon[i + 1].y();
      p.z = 0.0;
      mesh_msg.points.push_back(p);
    }
  }
  {
    pcl::PointCloud<pcl::PointXYZ> cloud_polygon;
    vector<Eigen::Vector2d> vs;

    Eigen::Vector2d p1(0.0, -5.0);
    vs.push_back(p1);
    Eigen::Vector2d p2(0.0, -6.0);
    vs.push_back(p2);
    Eigen::Vector2d p3(60.0, -6.0);
    vs.push_back(p3);
    Eigen::Vector2d p4(60.0, -5.0);
    vs.push_back(p4);
    cloud_polygon = fillConvexPolygon(vs);
    for (size_t i = 0; i < cloud_polygon.points.size(); i++) {
      pt_random.x = cloud_polygon.points[i].x;
      pt_random.y = cloud_polygon.points[i].y;
      pt_random.z = 0.0;
      cloud_map.points.push_back(pt_random);
    }
    vector<Eigen::Vector2d> vector_polygon = vs;
    geometry_msgs::Point init_p;
    init_p.x = vector_polygon[0].x();
    init_p.y = vector_polygon[0].y();
    init_p.z = 0.0;
    for (int i = 1; i < 3; i++) {
      mesh_msg.points.push_back(init_p);
      geometry_msgs::Point p;
      p.x = vector_polygon[i].x();
      p.y = vector_polygon[i].y();
      p.z = 0.0;
      mesh_msg.points.push_back(p);
      p.x = vector_polygon[i + 1].x();
      p.y = vector_polygon[i + 1].y();
      p.z = 0.0;
      mesh_msg.points.push_back(p);
    }
  }
  addObs();

  cloud_map.width = cloud_map.points.size();
  cloud_map.height = 1;
  cloud_map.is_dense = true;
  kd_tree.setInputCloud(cloud_map.makeShared());
  has_map = true;

  pcl::toROSMsg(cloud_map, global_msg);
  global_msg.header.frame_id = "world";

  mesh_msg.id = 0;
  mesh_msg.type = visualization_msgs::Marker::TRIANGLE_LIST;
  mesh_msg.action = visualization_msgs::Marker::ADD;
  mesh_msg.scale.x = 1.0;
  mesh_msg.scale.y = 1.0;
  mesh_msg.scale.z = 1.0;
  mesh_msg.color.r = 0.2;
  mesh_msg.color.g = 0.2;
  mesh_msg.color.b = 0.2;
  mesh_msg.color.a = 0.3;
  mesh_msg.header.frame_id = "world";
}

void rcvOdomCallBack(const nav_msgs::OdometryConstPtr msg) {
  sensor_pose.x = msg->pose.pose.position.x;
  sensor_pose.y = msg->pose.pose.position.y;
  sensor_pose.z = 0.0;
  has_odom = true;
}

void sensorCallback(const ros::TimerEvent &e) {
  if (!has_map || !has_odom)
    return;
  static int count = 0;
  count++;

  global_map_pub.publish(global_msg);
  mesh_map_pub.publish(mesh_msg);

  pcl::PointCloud<pcl::PointXYZ> local_map;

  pointIdxRadiusSearch.clear();
  pointRadiusSquaredDistance.clear();
  idx_map.setConstant(-1);
  dis_map.setConstant(9999.0);

  pcl::PointXYZ pt;
  if (kd_tree.radiusSearch(sensor_pose, sensor_range, pointIdxRadiusSearch,
                           pointRadiusSquaredDistance) > 0) {
    for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++) {
      pt = cloud_map.points[pointIdxRadiusSearch[i]];
      int idx = floor((atan2(pt.y - sensor_pose.y, pt.x - sensor_pose.x) +
                       M_PI + laser_res / 2.0) /
                      laser_res);
      if (idx >= 0 && idx < LINE_NUM &&
          dis_map[idx] > pointRadiusSquaredDistance[i]) {
        idx_map[idx] = idx;
        dis_map[idx] = pointRadiusSquaredDistance[i];
      }
    }

    for (int i = 0; i < LINE_NUM; i++) {
      if (idx_map[i] != -1) {
        double angle = idx_map[i] * laser_res - M_PI;
        double dist = sqrt(dis_map[i]);
        pt.x = dist * cos(angle) + sensor_pose.x;
        pt.y = dist * sin(angle) + sensor_pose.y;
        pt.z = 0.0;
        local_map.push_back(pt);
      }
    }
  }

  local_map.width = local_map.points.size();
  local_map.height = 1;
  local_map.is_dense = true;

  sensor_msgs::PointCloud2 local_msg;
  pcl::toROSMsg(local_map, local_msg);
  local_msg.header.frame_id = "world";
  local_map_pub.publish(local_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "random_map_node");
  ros::NodeHandle nh("~");

  nh.param<std::vector<int>>("map/obs_num", obs_num, std::vector<int>());
  // 优先从全局配置读取 resolution，如果不存在则从 map/resolution 读取
  if (!nh.getParam("/global/map_resolution", resolution)) {
    nh.getParam("map/resolution", resolution);
  }
  nh.getParam("map/size_x", size_x);
  nh.getParam("map/size_y", size_y);
  nh.getParam("map/min_width", min_width);
  nh.getParam("map/max_width", max_width);
  nh.getParam("map/min_dis", min_dis);
  nh.getParam("map/sensor_rate", sensor_rate);
  nh.getParam("map/sensor_range", sensor_range);

  ROS_INFO("Random map generator using resolution: %.3f", resolution);

  odom_sub = nh.subscribe("odom", 1000, rcvOdomCallBack);
  // local_map_pub = nh.advertise<sensor_msgs::PointCloud2>("local_cloud", 1);
  // global_map_pub = nh.advertise<sensor_msgs::PointCloud2>("global_cloud", 1);
  // mesh_map_pub = nh.advertise<visualization_msgs::Marker>("mesh_obstacles",
  // 1);

  // generateMap();

  sensor_timer =
      nh.createTimer(ros::Duration(1.0 / sensor_rate), sensorCallback);

  ros::spin();

  return 0;
}