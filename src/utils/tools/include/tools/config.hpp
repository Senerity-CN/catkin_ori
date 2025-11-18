#ifndef CONFIG.HPP
#define CONFIG .HPP
#include <Eigen/Eigen>
#include <XmlRpcValue.h>
#include <cfloat>
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <vector>
struct Config {
  double mini_T = 0.0;
  double wei_time_ = 500.0;
  double kmax = 0.45;
  double vmax = 3.0;
  double latAccmax = 3.0;
  double lonAccmax = 3.0;
  double accRatemax = 8.0;
  double kdotmax = 5.0;
  int traj_res = 16;
  double scaling_wf_min_ = 0.01, scaling_wc_min_ = 0.01;
  double rho = 1.0, rho_max = 1000.0;
  double gamma = 1;
  double cons_eps_ = 0.20;
  int mem_size = 256;
  int past = 3; // 3
  double g_epsilon = 1.0e-3;
  double min_step = 1.0e-32;
  double delta = 1.0e-4;
  int max_iterations = 12000;
  int amlMaxIter = 15;
  double pieceTime = 0.7;
  XmlRpc::XmlRpcValue xmlconpts;
  std::vector<Eigen::Vector2d> conpts;
  /*map parameters*/
  double mapRes = 0.1;
  double mapX = 50.0, mapY = 50.0, mapZ = 10.0;
  int expandSize = 1;
  /*front end*/
  double wheel_base = 0.5;
  double horizon_ = 50.0;
  double yaw_resolution_ = 0.3;
  double lambda_heu_ = 5.0;
  int allocate_num_ = 100000;
  int check_num_ = 5;
  double max_seach_time = 1.0;
  double step_arc = 0.9;
  double checkl = 0.2;
  double non_siguav = 0.0;

  Config(const ros::NodeHandle &nh_priv) {
    nh_priv.param("mini_T", mini_T, 0.0);
    nh_priv.param("wei_time_", wei_time_, 500.0);
    nh_priv.param("kmax", kmax, 0.45);
    nh_priv.param("vmax", vmax, 3.0);
    nh_priv.param("latAccmax", latAccmax, 3.0);
    nh_priv.param("lonAccmax", lonAccmax, 3.0);
    nh_priv.param("accRatemax", accRatemax, 8.0);
    nh_priv.param("kdotmax", kdotmax, 5.0);
    nh_priv.param("traj_res", traj_res, 16);
    nh_priv.param("rho", rho, 1.0);
    nh_priv.param("rho_max", rho_max, 1000.0);
    nh_priv.param("gamma", gamma, 1.0);
    nh_priv.param("cons_eps_", cons_eps_, 0.2);
    nh_priv.param("mem_size", mem_size, 256);
    nh_priv.param("past", past, 3);
    nh_priv.param("g_epsilon", g_epsilon, 1.0e-3);
    nh_priv.param("min_step", min_step, 1.0e-32);
    nh_priv.param("delta", delta, 1.0e-4);
    nh_priv.param("max_iterations", max_iterations, 10000);
    nh_priv.param("amlMaxIter", amlMaxIter, 15);
    nh_priv.param("pieceTime", pieceTime, 0.7);
    nh_priv.getParam("conpts", xmlconpts);
    for (int i = 0; i < xmlconpts.size(); ++i) {
      Eigen::Vector2d pt;
      for (int j = 0; j < 2; ++j) {
        pt[j] = xmlconpts[i][j];
      }
      conpts.push_back(pt);
    }
    // 优先从全局配置读取 mapRes，如果不存在则从 mapRes 读取
    if (!nh_priv.getParam("/global/map_resolution", mapRes)) {
      nh_priv.param("mapRes", mapRes, 0.1);
    }
    nh_priv.param("mapX", mapX, 50.0);
    nh_priv.param("mapY", mapY, 50.0);
    nh_priv.param("expandSize", expandSize, 2);

    ROS_INFO("Config using map resolution: %.3f", mapRes);
    nh_priv.param("wheel_base", wheel_base, 0.5);
    nh_priv.param("horizon_", horizon_, 50.0);
    nh_priv.param("yaw_resolution_", yaw_resolution_, 0.3);
    nh_priv.param("lambda_heu_", lambda_heu_, 5.0);
    nh_priv.param("allocate_num_", allocate_num_, 100000);
    nh_priv.param("check_num_", check_num_, 5);
    nh_priv.param("max_seach_time", max_seach_time, 1.0);
    nh_priv.param("step_arc", step_arc, 0.9);
    nh_priv.param("checkl", checkl, 0.2);
    nh_priv.param("non_siguav", non_siguav, 0.0);
  }
};
double normalize_angle(const double &theta) {
  double theta_tmp = theta;
  theta_tmp -= (theta >= acos(-1.0)) * 2 * acos(-1.0);
  theta_tmp += (theta < -acos(-1.0)) * 2 * acos(-1.0);
  return theta_tmp;
}
typedef std::shared_ptr<Config> ConfigPtr;

#endif