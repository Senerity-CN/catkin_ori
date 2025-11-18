#include <plan_manage/plan_manage.h>
#include <tf/tf.h>
#include <tools/tic_toc.hpp>

#include <fstream>
using namespace plan_manage;
void PlanManager::init(ros::NodeHandle &nh)
{
  vis_tool.reset(new visualization::Visualization(nh));
  config_.reset(new Config(nh));
  gridMap.setParam(config_, nh);
  // point_cloud_sub_ = nh.subscribe("/global_map", 10, &MapUtil::MapBuild, this);
  gridmap_sub_ = nh.subscribe("/map", 1, &PlanManager::gridmapCallback, this);
  std::cout << "dim:" << gridMap.getDim() << std::endl;
  kino_path_finder_.reset(new path_searching::KinoAstar);
  kino_path_finder_->init(config_, nh);
  kino_path_finder_->intialMap(&gridMap);
  hasTarget = false;
  hasOdom = false;
  processTimer = nh.createTimer(ros::Duration(0.02), &PlanManager::process, this);
  targetSub = nh.subscribe("/move_base_simple/goal", 1, &PlanManager::targetCallback, this);
  odomSub = nh.subscribe("/ugv/odometry", 1, &PlanManager::odomCallback, this); // car
  // odomSub = nh.subscribe("/odom", 1, &PlanManager::odomCallback, this); // ball_sim
  pclSub = nh.subscribe("/ugv/laser", 10, &PlanManager::pclCallback, this);
  vis_tool->registe<nav_msgs::Path>("/visualization/kinoPath");
  vis_tool->registe<nav_msgs::Path>("/visualization/optTraj");
  vis_tool->registe<visualization_msgs::MarkerArray>("/visualization/optArrowTraj");
  vis_tool->registe<decomp_ros_msgs::PolyhedronArray>("/visualization/sfc");
  pieceTime = config_->pieceTime;
  Dftpav_path_pub_ = nh.advertise<nav_msgs::Path>("/Dftpav_path", 1);
  ploy_traj_opt_.reset(new PolyTrajOptimizer);
  ploy_traj_opt_->setParam(nh, config_);
}
void PlanManager::pclCallback(const sensor_msgs::PointCloud2 &pointcloud_map)
{
  double t1 = ros::Time::now().toSec();
  gridMap.mapUpdate(pointcloud_map);
  // ROS_INFO("Laser find dynamic obstacle!");
  double t2 = ros::Time::now().toSec();
}
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
geometry_msgs::Pose EigenToPose(const Eigen::Vector3d &vec)
{
  geometry_msgs::Pose pose;

  pose.position.x = vec[0];
  pose.position.y = vec[1];

  double yaw = vec[2];

  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = sin(yaw / 2.0);
  pose.orientation.w = cos(yaw / 2.0);

  return pose;
}
void PlanManager::getGalaxConst(std::vector<Eigen::Vector3d> statelist)
{
  vec_Vec2f vec_obs = gridMap.getCloud();
  hPolys.clear();
  for (const auto state : statelist)
  {
    Eigen::MatrixXd hPoly;
    Eigen::Vector2d pos = state.head(2);
    double yaw = state[2];
    std::vector<Eigen::Vector2d> add_vec_obs;
    Eigen::Matrix2d R;
    R << cos(yaw), -sin(yaw),
        sin(yaw), cos(yaw);
    Eigen::Vector2d p;
    double d_x = 10.0;
    double d_y = 10.0;
    p = R * Eigen::Vector2d(d_x, d_y);
    add_vec_obs.push_back(pos + p);
    p = R * Eigen::Vector2d(d_x, -d_y);
    add_vec_obs.push_back(pos + p);
    p = R * Eigen::Vector2d(-d_x, d_y);
    add_vec_obs.push_back(pos + p);
    p = R * Eigen::Vector2d(-d_x, -d_y);
    add_vec_obs.push_back(pos + p);
    plan_utils::corridorBuilder2d(pos, 100.0, 10.0, 10.0, vec_obs, add_vec_obs, hPoly);
    hPolys.push_back(hPoly);
  }
  return;
}
void PlanManager::odomCallback(const nav_msgs::OdometryPtr &msg)
{
  static double t0 = ros::Time::now().toSec();
  odom[0] = msg->pose.pose.position.x;
  odom[1] = msg->pose.pose.position.y;
  // Eigen::Quaterniond q(msg->pose.pose.orientation.w,
  //                      msg->pose.pose.orientation.x,
  //                      msg->pose.pose.orientation.y,
  //                      msg->pose.pose.orientation.z);
  // Eigen::Matrix3d R(q);
  // odom[2] = atan2(R.col(0)[1], R.col(0)[0]);
  odom[2] = tf::getYaw(msg->pose.pose.orientation);
  hasOdom = true;
  startPos << msg->pose.pose.position.x, msg->pose.pose.position.y;
  startVel << msg->twist.twist.linear.x, msg->twist.twist.linear.y;
  startAcc << 0.0, 0.0;

  double t1 = ros::Time::now().toSec();
  return;
}
void PlanManager::targetCallback(const geometry_msgs::PoseStamped &msg)
{
  ROS_INFO("Recieved target!");
  targetPose << msg.pose.position.x, msg.pose.position.y,
      tf::getYaw(msg.pose.orientation);
  // targetPose << 60, 29, 0;
  std::cout << "targetPose: " << targetPose.transpose() << std::endl;
  bool flag;
  gridMap.CheckIfCollisionUsingPos(targetPose, &flag);
  if (flag)
    std::cout << "zai zhangaiwu li : " << std::endl;
  else
    std::cout << "dangqiandian hefa: " << std::endl;

  hasTarget = true;
  tkReplan_num = 0;
  startExeTime = ros::Time::now().toSec();

  return;
}
void PlanManager::gridmapCallback(const nav_msgs::OccupancyGrid &msg)
{
  // ++gridmap_update_cnt_;
  ROS_INFO("Receive a gridmap, the resolution is %.3f.", msg.info.resolution);
  origin_gridmap_ = msg;
  // 创建一个PointCloud对象
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles(new pcl::PointCloud<pcl::PointXYZ>());
  std::cout << "width and height: " << origin_gridmap_.info.width << "," << origin_gridmap_.info.height << std::endl;
  // 遍历OccupancyGrid
  for (unsigned int y = 0; y < origin_gridmap_.info.height; y++)
  {
    for (unsigned int x = 0; x < origin_gridmap_.info.width; x++)
    {
      // 获取对应格子的值
      unsigned int i = x + (origin_gridmap_.info.width * y);
      if (origin_gridmap_.data[i] > 50)
      { // 如果该格点有障碍物
        // std::cout << "this grid have obstacle: " << std::endl;
        pcl::PointXYZ point;
        point.x = (x * origin_gridmap_.info.resolution) + origin_gridmap_.info.origin.position.x;
        point.y = (y * origin_gridmap_.info.resolution) + origin_gridmap_.info.origin.position.y;
        // point.x = x + origin_gridmap_.info.origin.position.x;
        // point.y = y + origin_gridmap_.info.origin.position.y;
        point.z = 0; // 假设你的地图是2D的，我们只设置z为0
        obstacles->points.push_back(point);
      }
    }
  }

  // 将PointCloud转为sensor_msgs::PointCloud2
  sensor_msgs::PointCloud2 cloud;
  pcl::toROSMsg(*obstacles, cloud);

  // 设置header
  // cloud.header.stamp = ros::Time::now();
  cloud.header.frame_id = "world"; // 通常来说，PointCloud的frame_id应与OccupancyGrid相同
  gridMap.MapBuild(cloud);
}

// replan time?

bool PlanManager::checkReplan()
{
  bool needReplan = false;

  if (!hasTraj)
  {
    needReplan = true;
    iniFs << odom, 0.0;
    finFs << targetPose, 0.0;
    useReplanState = false;
    return true;
  }
  double timeNow = ros::Time::now().toSec();
  double deltaTime = timeNow - trajContainer.startTime;

  // collision check
  for (double t = deltaTime; t <= std::min(deltaTime + 2.5, trajContainer.getTotalDuration()); t += 0.05)
  {
    Eigen::Vector3d state = trajContainer.getState(t);
    bool isocc = false;
    gridMap.CheckIfCollisionUsingPosAndYaw(state, &isocc);
    if (isocc)
    {
      needReplan = true;
    }
  }
  // tracking error - check
  Eigen::Vector2d desiredPos = trajContainer.getPos(deltaTime);
  Eigen::Vector2d endPos = trajContainer.getPos(trajContainer.getTotalDuration());
  if ((desiredPos - startPos).norm() > 0.15)
  {
    tkReplan_num++;
    needReplan = true;
  }
  // reach tmp goal
  if ((desiredPos - endPos).norm() <= 4.0)
  {
    needReplan = true;
  }

  iniFs << odom, 0.0;
  finFs << targetPose, 0.0;
  if (needReplan)
  {
    useReplanState = true;
    return true;
  }
  else
  {
    return false;
  }
}
void PlanManager::process(const ros::TimerEvent &)
{
  if (!hasTarget || !hasOdom)
    return;

  if (isPoseEqual(EigenToPose(odom), EigenToPose(targetPose), 0.3, 0.3)) // reach goal
  {
    std::cout << "we has reached the target~" << std::endl;
    std::cout << "\033[31m"
              << "replan number: " << tkReplan_num << std::endl;
    endExeTime = ros::Time::now().toSec();
    std::cout << "execution Time: " << (endExeTime - startExeTime) << " seconds \033[0m" << std::endl;
    hasTarget = false;
    return;
  }
  if (!checkReplan())
    return;

  kino_path_finder_->reset();
  /*front end kino Search*/
  TicToc time_profile_tool_; // 混合A*规划计时器
  time_profile_tool_.tic();
  path_searching::KinoTrajData kino_trajs_;
  std::cout << "iniFs: " << iniFs.transpose() << " finFs: " << finFs.transpose() << std::endl;

  // 调用kino_path_finder_对象的search方法，以iniFs为起始状态，以finFs为终止状态，进行路径搜索。如果成功找到一条路径，那么就会返回KinoAstar::VALID，即表示路径有效。
  int status = kino_path_finder_->search(iniFs, Eigen::Vector2d::Zero(), finFs, false);
  if (status == path_searching::KinoAstar::NO_PATH)
  {
    ROS_ERROR("Failed to search");
    return;
  }
  kino_path_finder_->getKinoNode(kino_trajs_); // 获取路径搜索结果。这将返回一个包含KinoTrajData的向量，即kino_trajs_，每个KinoTrajData代表了一段运动学轨迹。
  std::cout << "kino_trajs_.size: " << kino_trajs_.size() << std::endl;

  std::vector<Eigen::Vector3d> visKinoPath;
  for (double t = 0.0; t < kino_path_finder_->totalTrajTime; t += 0.01)
  {
    Eigen::Vector3d pos;
    pos = kino_path_finder_->evaluatePos(t);
    pos[2] = 0.0;
    visKinoPath.push_back(pos);
  }
  /*vis*/
  vis_tool->visualize_path(visKinoPath, "/visualization/kinoPath");
  ROS_INFO_STREAM("Front end Sucessfully completed! Front end time: " << time_profile_tool_.toc() / 1000.0 << "ms");

  Eigen::MatrixXd flat_finalState(2, 3), flat_headState(2, 3);
  Eigen::VectorXd ego_piece_dur_vec;
  Eigen::MatrixXd ego_innerPs;
  double basetime = 0.0;

  /*try to merge optimization process*/
  std::vector<std::vector<Eigen::MatrixXd>> sfc_container;
  std::vector<int> singul_container;
  Eigen::VectorXd duration_container;
  std::vector<Eigen::MatrixXd> waypoints_container;
  std::vector<Eigen::MatrixXd> iniState_container, finState_container;
  duration_container.resize(kino_trajs_.size());
  for (unsigned int i = 0; i < kino_trajs_.size(); i++)
  {
    double timePerPiece = config_->pieceTime;
    path_searching::FlatTrajData kino_traj = kino_trajs_.at(i);
    singul_container.push_back(kino_traj.singul);
    std::vector<Eigen::Vector3d> pts = kino_traj.traj_pts; // x,y,duration t
    int piece_nums;
    double initTotalduration = 0.0;
    for (const auto pt : pts)
    {
      initTotalduration += pt[2];
    }
    piece_nums = std::max(int(initTotalduration / timePerPiece + 0.5), 2);
    timePerPiece = initTotalduration / piece_nums;
    std::cout << "piecenum: " << piece_nums << std::endl;
    ego_piece_dur_vec.resize(piece_nums);
    ego_piece_dur_vec.setConstant(timePerPiece);
    duration_container[i] = timePerPiece * piece_nums;
    ego_innerPs.resize(2, piece_nums - 1);
    std::vector<Eigen::Vector3d> statelist;
    double res_time = 0;
    for (int i = 0; i < piece_nums; i++)
    {
      int resolution;
      if (i == 0 || i == piece_nums - 1)
      {
        resolution = config_->traj_res;
      }
      else
      {
        resolution = config_->traj_res;
      }
      for (int k = 0; k <= resolution; k++)
      {
        double t = basetime + res_time + 1.0 * k / resolution * ego_piece_dur_vec[i];
        Eigen::Vector3d pos = kino_path_finder_->evaluatePos(t);
        statelist.push_back(pos);
        if (k == resolution && i != piece_nums - 1)
        {
          ego_innerPs.col(i) = pos.head(2);
        }
      }
      res_time += ego_piece_dur_vec[i];
    }
    std::cout << "s: " << kino_traj.singul << "\n";
    double tm1 = ros::Time::now().toSec();
    getGalaxConst(statelist);
    sfc_container.push_back(hPolys);
    waypoints_container.push_back(ego_innerPs);
    iniState_container.push_back(kino_traj.start_state);
    finState_container.push_back(kino_traj.final_state);
    basetime += initTotalduration;
  }

  std::cout << "try to optimize!\n";

  // if (useReplanState)
  //   iniState_container[0] << startPos, startVel, startAcc;

  int flag_success = ploy_traj_opt_->OptimizeTrajectory(iniState_container, finState_container,
                                                        waypoints_container, duration_container,
                                                        sfc_container, singul_container, 0.0);

  std::cout << "iniState_container: " << iniState_container[0] << std::endl;
  std::cout << "finState_container: " << finState_container[0] << std::endl;
  for (int i = 0; i < singul_container.size(); i++)
  {
    std::cout << "singul_container: " << singul_container[i] << std::endl;
  }

  vis_tool->visualize_sfc(hPolys, "/visualization/sfc");

  trajContainer.clear();
  for (unsigned int i = 0; i < kino_trajs_.size(); i++)
  {
    trajContainer.push_back((*ploy_traj_opt_->getMinJerkOptPtr())[i].getTraj(singul_container[i]));
  }
  hasTraj = true;
  trajContainer.startTime = ros::Time::now().toSec();
  vis_tool->visualize_traj(trajContainer, "/visualization/optTraj");

  // publish dftpav_path with vel and acceleration
  double start_time = 0;
  int segment_index = 0;
  int piece_index = 0;
  auto trajtime = trajContainer.getTotalDuration();
  std::vector<Eigen::VectorXd> path; // x,y,yaw,vx,vy,ax,ay
  for (double t = 0; t <= trajtime; t += 0.5)
  {
    Eigen::Vector3d v = Eigen::Vector3d::Zero();    // 速度
    Eigen::Vector3d a = Eigen::Vector3d::Zero();    // 加速度
    Eigen::Vector3d jerk = Eigen::Vector3d::Zero(); // jerk
    while (true)
    {
      double end_time = start_time + trajContainer[segment_index][piece_index].getDuration();

      // 如果t在当前的piece中，表示找到了对应的piece
      if (t <= end_time)
      {
        // 这里调用piece的系数
        // trajContainer[segment_index][piece_index]就是t所在的piece
        Eigen::Matrix<double, 2, 6> c = trajContainer[segment_index][piece_index].getCoeffMat();
        Eigen::Matrix<double, 2, 5> c_derivative = Eigen::Matrix<double, 2, 5>::Zero();        // 一阶导数的系数
        Eigen::Matrix<double, 2, 4> c_second_derivative = Eigen::Matrix<double, 2, 4>::Zero(); // 二阶导数的系数
        Eigen::Matrix<double, 2, 3> c_third_derivative = Eigen::Matrix<double, 2, 3>::Zero();  // 3阶导数的系数

        for (int i = 0; i < 5; i++)
        {
          // 计算一阶导数的系数
          c_derivative(0, i) = (5 - i) * c(0, i);
          c_derivative(1, i) = (5 - i) * c(1, i);

          // 计算二阶导数的系数
          if (i < 4)
          {
            c_second_derivative(0, i) = (4 - i) * c_derivative(0, i);
            c_second_derivative(1, i) = (4 - i) * c_derivative(1, i);
          }

          if (i < 3)
          {
            c_third_derivative(0, i) = (3 - i) * c_second_derivative(0, i);
            c_third_derivative(1, i) = (3 - i) * c_second_derivative(1, i);
          }
        }
        for (int i = 0; i < 5; i++)
        {
          v[0] += c_derivative(0, i) * pow((t - start_time), 4 - i);
          v[1] += c_derivative(1, i) * pow((t - start_time), 4 - i);

          if (i < 4)
          {
            a[0] += c_second_derivative(0, i) * pow((t - start_time), 3 - i);
            a[1] += c_second_derivative(1, i) * pow((t - start_time), 3 - i);
          }
          if (i < 3)
          {
            jerk[0] += c_third_derivative(0, i) * pow((t - start_time), 2 - i);
            jerk[1] += c_third_derivative(1, i) * pow((t - start_time), 2 - i);
          }
        }
        v[2] = std::sqrt(v[0] * v[0] + v[1] * v[1]);
        a[2] = std::sqrt(a[0] * a[0] + a[1] * a[1]);
        jerk[2] = std::sqrt(jerk[0] * jerk[0] + jerk[1] * jerk[1]);
        break;
      }

      // 如果t不在当前的piece中，更新start_time和piece的索引
      start_time = end_time;
      piece_index++;

      // 如果所有的piece都已经遍历完，就更新segment的索引并重置piece的索引
      if (piece_index >= trajContainer[segment_index].getPieceNum())
      {
        segment_index++;
        piece_index = 0;
      }
    }
    Eigen::VectorXd pos = trajContainer.getState(t);
    Eigen::VectorXd p3(6);
    p3 << pos, v[2], a[2], jerk[2];
    // std::cout << "v = " << v << "a = " << a << std::endl;
    path.push_back(p3);
  }
  nav_msgs::Path path_msg;
  ROS_INFO_STREAM("Number of poses in path message1: " << path_msg.poses.size());
  geometry_msgs::PoseStamped tmpPose;
  tmpPose.header.frame_id = "world";
  for (const auto &pt : path)
  {
    tmpPose.pose.position.x = pt[0];
    tmpPose.pose.position.y = pt[1];
    tmpPose.pose.position.z = pt[2];
    tmpPose.pose.orientation.x = pt[3];
    tmpPose.pose.orientation.y = pt[4];
    tmpPose.pose.orientation.z = pt[5];
    // tmpPose.pose.orientation.w = pt[6];
    path_msg.poses.push_back(tmpPose);
  }
  path_msg.header.frame_id = "world";
  path_msg.header.stamp = ros::Time::now();
  ROS_INFO_STREAM("Number of poses in path message2: " << path_msg.poses.size());
  Dftpav_path_pub_.publish(path_msg);

  // // position cmd
  // for (int i = 0; i < trajContainer.getSegNum(); i++)
  // {
  //   for (int j = 0; j < trajContainer[i].getPieceNum(); j++)
  //   {
  //     mpc_controller::SinglePoly piece;
  //     if (trajContainer[i][j].getSingual() > 0)
  //       piece.reverse = false;
  //     else
  //       piece.reverse = true;
  //     piece.duration = trajContainer[i][j].getDuration();
  //     Eigen::Matrix<double, 2, 6> c = trajContainer[i][j].getCoeffMat();
  //     for (int k = 0; k < 6; k++)
  //     {
  //       piece.coef_x.push_back(c(0, k));
  //       piece.coef_y.push_back(c(1, k));
  //     }
  //     trajmsg.trajs.push_back(piece);
  //   }
  // }

  // hasTarget = false;
}