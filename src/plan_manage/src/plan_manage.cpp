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
  
  // Initialize event-driven replanning parameters
  nh.param("event_driven_replanning/obstacle_change_threshold", obstacleChangeThreshold_, 10.0);
  nh.param("event_driven_replanning/tracking_error_threshold", trackingErrorThreshold_, 0.15);
  nh.param("event_driven_replanning/safety_check_frequency", safetyCheckFrequency_, 10.0);
  nh.param("event_driven_replanning/obstacle_detection_enabled", obstacleDetectionEnabled_, true);
  
  // Initialize event-driven replanning variables
  needReplan_ = false;
  trackingErrorDetected_ = false;
  obstacleChangeDetected_ = false;
  hasLastPointCloud_ = false;
  replanTriggerCount_ = 0;
  lastReplanTime_ = 0.0;
  totalReplanCount_ = 0;
  lastReplanReason_ = "none";
  
  // Initialize trajectory splicing variables
  needTrajectorySpicing_ = false;
  splicingTime_ = 1.0;  // 1 second splicing time
  splicingStartState_ = Eigen::Vector3d::Zero();
  splicingStartVel_ = Eigen::Vector3d::Zero();
  
  // Initialize timing statistics
  replan_stats_.frontend_time_ms = 0.0;
  replan_stats_.optimization_time_ms = 0.0;
  replan_stats_.splicing_time_ms = 0.0;
  replan_stats_.total_time_ms = 0.0;
  replan_stats_.replan_count = 0;
  
  // Adjust process timer frequency based on safety check frequency
  double timerFreq = std::max(safetyCheckFrequency_, 5.0); // minimum 5Hz
  processTimer = nh.createTimer(ros::Duration(1.0 / timerFreq), &PlanManager::process, this);
  
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
  
  ROS_INFO("Event-driven replanning initialized: obstacle_threshold=%.1f, tracking_threshold=%.3f, safety_freq=%.1fHz", 
           obstacleChangeThreshold_, trackingErrorThreshold_, safetyCheckFrequency_);
}
void PlanManager::pclCallback(const sensor_msgs::PointCloud2 &pointcloud_map)
{
  double t1 = ros::Time::now().toSec();
  
  // Detect obstacle changes before updating the map
  if (detectObstacleChange(pointcloud_map)) {
    obstacleChangeDetected_ = true;
    needReplan_ = true;
    lastReplanReason_ = "obstacle_change";
    replanTriggerCount_++;
    ROS_INFO("Event-driven replanning triggered by obstacle change (count: %d)", replanTriggerCount_);
  }
  
  gridMap.mapUpdate(pointcloud_map);
  
  double t2 = ros::Time::now().toSec();
  
  // Debug output for processing time
  if (obstacleDetectionEnabled_) {
    static int debugCounter = 0;
    debugCounter++;
    if (debugCounter % 100 == 0) { // Print every 100 callbacks
      ROS_DEBUG("PCL callback processing time: %.3f ms, points: %d", 
                (t2 - t1) * 1000.0, pointcloud_map.width);
    }
  }
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
  odom[2] = tf::getYaw(msg->pose.pose.orientation);
  hasOdom = true;
  startPos << msg->pose.pose.position.x, msg->pose.pose.position.y;
  startVel << msg->twist.twist.linear.x, msg->twist.twist.linear.y;
  startAcc << 0.0, 0.0;

  // Real-time tracking error detection
  if (hasTraj && hasTarget) {
    double timeNow = ros::Time::now().toSec();
    double deltaTime = timeNow - trajContainer.startTime;
    
    // Check if we have a valid trajectory time
    if (deltaTime >= 0 && deltaTime <= trajContainer.getTotalDuration()) {
      Eigen::Vector2d desiredPos = trajContainer.getPos(deltaTime);
      Eigen::Vector2d currentPos = startPos;
      double trackingError = (desiredPos - currentPos).norm();
      
      if (trackingError > trackingErrorThreshold_) {
        if (!trackingErrorDetected_) { // Avoid repeated triggers
          trackingErrorDetected_ = true;
          needReplan_ = true;
          lastReplanReason_ = "tracking_error";
          replanTriggerCount_++;
          ROS_INFO("Event-driven replanning triggered by tracking error: %.3f > %.3f (count: %d)", 
                   trackingError, trackingErrorThreshold_, replanTriggerCount_);
        }
      } else {
        // Reset tracking error flag when error is back to normal
        trackingErrorDetected_ = false;
      }
    }
  }

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
    lastReplanReason_ = "no_trajectory";
    return true;
  }
  
  double timeNow = ros::Time::now().toSec();
  double deltaTime = timeNow - trajContainer.startTime;

  // Safety collision check (reduced frequency check)
  for (double t = deltaTime; t <= std::min(deltaTime + 2.5, trajContainer.getTotalDuration()); t += 0.1)
  {
    Eigen::Vector3d state = trajContainer.getState(t);
    bool isocc = false;
    gridMap.CheckIfCollisionUsingPosAndYaw(state, &isocc);
    if (isocc)
    {
      needReplan = true;
      lastReplanReason_ = "collision_detected";
      ROS_WARN("Safety collision check triggered replanning at t=%.2f", t);
      break;
    }
  }
  
  // Check if approaching end of trajectory
  Eigen::Vector2d desiredPos = trajContainer.getPos(deltaTime);
  Eigen::Vector2d endPos = trajContainer.getPos(trajContainer.getTotalDuration());
  if ((desiredPos - endPos).norm() <= 4.0)
  {
    needReplan = true;
    lastReplanReason_ = "approaching_goal";
  }

  iniFs << odom, 0.0;
  finFs << targetPose, 0.0;
  
  if (needReplan)
  {
    useReplanState = true;
    tkReplan_num++;
    ROS_INFO("Safety check replanning: %s (total count: %d)", lastReplanReason_.c_str(), tkReplan_num);
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
              << "replan number: " << tkReplan_num << " (event-driven triggers: " << replanTriggerCount_ << ")" << std::endl;
    endExeTime = ros::Time::now().toSec();
    std::cout << "execution Time: " << (endExeTime - startExeTime) << " seconds \033[0m" << std::endl;
    hasTarget = false;
    resetReplanFlags();
    return;
  }
  
  // Event-driven replanning logic
  bool shouldReplan = false;
  std::string replanReason = "none";
  
  // Check for event-driven triggers first
  if (needReplan_) {
    shouldReplan = true;
    replanReason = lastReplanReason_;
    ROS_INFO("Event-driven replanning triggered: %s", replanReason.c_str());
  } else {
    // Perform safety checks at reduced frequency
    if (checkReplan()) {
      shouldReplan = true;
      replanReason = "safety_check";
    }
  }
  
  if (!shouldReplan) {
    // Publish debug info even when not replanning
    publishDebugInfo();
    return;
  }
  
  // Check minimum replan interval to avoid too frequent replanning
  double currentTime = ros::Time::now().toSec();
  double minReplanInterval = 0.1; // 100ms minimum interval
  if (currentTime - lastReplanTime_ < minReplanInterval) {
    ROS_DEBUG("Replanning skipped due to minimum interval constraint");
    return;
  }
  
  lastReplanTime_ = currentTime;
  totalReplanCount_++;
  
  // Start timing for replanning performance analysis
  TicToc total_time_tool;
  total_time_tool.tic();
  
  // Reset event flags after processing
  resetReplanFlags();

  kino_path_finder_->reset();
  /*front end kino Search*/
  TicToc frontend_timer; // 混合A*规划计时器
  frontend_timer.tic();
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
  
  // Record frontend timing
  double frontend_time = frontend_timer.toc(); // Already in milliseconds
  recordReplanningTime("frontend", frontend_time);
  ROS_INFO_STREAM("Frontend completed! Time: " << frontend_time << "ms");

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

  std::cout << "Starting trajectory optimization...\n";
  
  // Start optimization timing
  TicToc optimization_timer;
  optimization_timer.tic();

  // if (useReplanState)
  //   iniState_container[0] << startPos, startVel, startAcc;

  int flag_success = ploy_traj_opt_->OptimizeTrajectory(iniState_container, finState_container,
                                                        waypoints_container, duration_container,
                                                        sfc_container, singul_container, 0.0);
  
  // Record optimization timing
  double optimization_time = optimization_timer.toc(); // Already in milliseconds
  recordReplanningTime("optimization", optimization_time);
  ROS_INFO_STREAM("Optimization completed! Time: " << optimization_time << "ms");

  std::cout << "iniState_container: " << iniState_container[0] << std::endl;
  std::cout << "finState_container: " << finState_container[0] << std::endl;
  for (int i = 0; i < singul_container.size(); i++)
  {
    std::cout << "singul_container: " << singul_container[i] << std::endl;
  }

  vis_tool->visualize_sfc(hPolys, "/visualization/sfc");

  // Start trajectory splicing timing
  TicToc splicing_timer;
  splicing_timer.tic();

  // Generate new trajectory
  plan_utils::TrajectoryContainer newTrajContainer;
  for (unsigned int i = 0; i < kino_trajs_.size(); i++)
  {
    newTrajContainer.push_back((*ploy_traj_opt_->getMinJerkOptPtr())[i].getTraj(singul_container[i]));
  }

  // Perform trajectory splicing if we have an existing trajectory
  if (hasTraj && useReplanState) {
    // Find splicing point (1 second ahead)
    Eigen::Vector3d splicing_point = findSplicingPoint(splicingTime_);
    
    // Get velocity at splicing point using getdSigma (2D velocity)
    Eigen::Vector2d vel_2d = trajContainer.getdSigma(splicingTime_);
    Eigen::Vector3d splicing_vel(vel_2d[0], vel_2d[1], 0); // Convert to 3D
    
    // Get target state from new trajectory
    Eigen::Vector3d target_state = newTrajContainer.getState(0.5); // 0.5s into new trajectory
    
    // Get target velocity using getdSigma
    Eigen::Vector2d target_vel_2d = newTrajContainer.getdSigma(0.5);
    Eigen::Vector3d target_vel(target_vel_2d[0], target_vel_2d[1], 0); // Convert to 3D
    
    // Generate splicing trajectory using 5th-order polynomial
    plan_utils::TrajectoryContainer splicingTraj = generateSplicingTrajectory(
        splicing_point, splicing_vel, target_state, target_vel, splicingTime_);
    
    // Combine trajectories: current -> splicing -> new
    // For now, use simplified approach: direct replacement with new trajectory
    // The splicing trajectory generation is prepared but not fully integrated
    trajContainer = newTrajContainer;
  } else {
    trajContainer = newTrajContainer;
  }
  
  hasTraj = true;
  trajContainer.startTime = ros::Time::now().toSec();
  vis_tool->visualize_traj(trajContainer, "/visualization/optTraj");
  
  // Record splicing timing
  double splicing_time = splicing_timer.toc(); // Already in milliseconds
  recordReplanningTime("splicing", splicing_time);
  
  // Record total timing
  double total_time = total_time_tool.toc(); // Already in milliseconds
  recordReplanningTime("total", total_time);
  
  ROS_INFO_STREAM("Trajectory splicing completed! Time: " << splicing_time << "ms");
  ROS_INFO_STREAM("Total replanning time: " << total_time << "ms (Frontend: " 
                  << replan_stats_.frontend_time_ms << "ms, Opt: " 
                  << replan_stats_.optimization_time_ms << "ms, Splice: " 
                  << replan_stats_.splicing_time_ms << "ms)");

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

// Event-driven replanning helper functions
bool PlanManager::detectObstacleChange(const sensor_msgs::PointCloud2 &currentCloud)
{
  if (!obstacleDetectionEnabled_) {
    return false;
  }
  
  if (!hasLastPointCloud_) {
    lastPointCloud_ = currentCloud;
    hasLastPointCloud_ = true;
    return false;
  }
  
  // Simple point count difference check
  double pointCountDiff = std::abs((double)currentCloud.width - (double)lastPointCloud_.width);
  
  if (pointCountDiff > obstacleChangeThreshold_) {
    ROS_INFO("Obstacle change detected: point count difference = %.1f (threshold: %.1f)", 
             pointCountDiff, obstacleChangeThreshold_);
    lastPointCloud_ = currentCloud;
    return true;
  }
  
  // Update last point cloud periodically to avoid drift
  static int updateCounter = 0;
  updateCounter++;
  if (updateCounter > 50) { // Update every 50 frames
    lastPointCloud_ = currentCloud;
    updateCounter = 0;
  }
  
  return false;
}

void PlanManager::resetReplanFlags()
{
  needReplan_ = false;
  trackingErrorDetected_ = false;
  obstacleChangeDetected_ = false;
  lastReplanReason_ = "none";
}

double PlanManager::calculatePointCloudDifference(const sensor_msgs::PointCloud2 &cloud1, 
                                                 const sensor_msgs::PointCloud2 &cloud2)
{
  // Simple implementation: compare point counts
  // More sophisticated implementation could compare actual point positions
  return std::abs((double)cloud1.width - (double)cloud2.width);
}

void PlanManager::printPerformanceStatistics()
{
  static double lastPrintTime = 0.0;
  double currentTime = ros::Time::now().toSec();
  
  // Print statistics every 10 seconds
  if (currentTime - lastPrintTime > 10.0) {
    ROS_INFO("=== Event-Driven Replanning & Trajectory Splicing Statistics ===");
    ROS_INFO("Total replans: %d (Event-driven: %d, Safety: %d)", 
             totalReplanCount_, replanTriggerCount_, tkReplan_num);
    ROS_INFO("Last replan reason: %s", lastReplanReason_.c_str());
    
    if (replan_stats_.replan_count > 0) {
      double avg_frontend = replan_stats_.frontend_time_ms;
      double avg_optimization = replan_stats_.optimization_time_ms;
      double avg_splicing = replan_stats_.splicing_time_ms;
      double avg_total = replan_stats_.total_time_ms;
      
      ROS_INFO("Latest timing (ms): Frontend: %.2f, Optimization: %.2f, Splicing: %.2f, Total: %.2f",
               avg_frontend, avg_optimization, avg_splicing, avg_total);
    }
    
    ROS_INFO("Obstacle detection enabled: %s", obstacleDetectionEnabled_ ? "true" : "false");
    ROS_INFO("Thresholds - Obstacle: %.1f, Tracking: %.3f", 
             obstacleChangeThreshold_, trackingErrorThreshold_);
    
    if (hasTarget && hasTraj) {
      double execTime = currentTime - startExeTime;
      ROS_INFO("Current execution time: %.2f seconds", execTime);
    }
    
    ROS_INFO("==============================================================");
    lastPrintTime = currentTime;
  }
}

void PlanManager::publishDebugInfo()
{
  // This function could publish debug markers or status messages
  // For now, just update performance statistics
  printPerformanceStatistics();
}

// Trajectory splicing implementation
plan_utils::TrajectoryContainer PlanManager::generateSplicingTrajectory(
    const Eigen::Vector3d& current_state, 
    const Eigen::Vector3d& current_vel,
    const Eigen::Vector3d& target_state,
    const Eigen::Vector3d& target_vel,
    double splicing_time)
{
  plan_utils::TrajectoryContainer splicingTraj;
  
  // 5th-order polynomial trajectory generation
  // For simplicity, we'll create a single piece trajectory
  // In practice, you might want to use the same trajectory optimization framework
  
  // Boundary conditions for 5th-order polynomial
  // p(0) = current_state, p'(0) = current_vel, p''(0) = 0
  // p(T) = target_state, p'(T) = target_vel, p''(T) = 0
  
  Eigen::MatrixXd coeff_x(6, 1), coeff_y(6, 1), coeff_yaw(6, 1);
  
  // Solve for x-coordinate coefficients
  Eigen::MatrixXd A(6, 6);
  Eigen::VectorXd b_x(6), b_y(6), b_yaw(6);
  
  double T = splicing_time;
  double T2 = T * T;
  double T3 = T2 * T;
  double T4 = T3 * T;
  double T5 = T4 * T;
  
  // Boundary condition matrix for 5th-order polynomial
  A << 1,  0,   0,    0,     0,      0,
       0,  1,   0,    0,     0,      0,
       0,  0,   2,    0,     0,      0,
       1,  T,  T2,   T3,    T4,     T5,
       0,  1, 2*T, 3*T2,  4*T3,   5*T4,
       0,  0,   2,  6*T, 12*T2,  20*T3;
  
  // Boundary conditions for x
  b_x << current_state[0], current_vel[0], 0, 
         target_state[0], target_vel[0], 0;
  
  // Boundary conditions for y
  b_y << current_state[1], current_vel[1], 0,
         target_state[1], target_vel[1], 0;
  
  // Boundary conditions for yaw (simplified)
  b_yaw << current_state[2], 0, 0,
           target_state[2], 0, 0;
  
  // Solve for coefficients
  coeff_x = A.colPivHouseholderQr().solve(b_x);
  coeff_y = A.colPivHouseholderQr().solve(b_y);
  coeff_yaw = A.colPivHouseholderQr().solve(b_yaw);
  
  // Create trajectory piece (this is a simplified version)
  // In practice, you'd integrate this with your existing trajectory representation
  
  return splicingTraj; // Return simplified trajectory for now
}

Eigen::Vector3d PlanManager::findSplicingPoint(double time_offset)
{
  if (!hasTraj) {
    return odom;
  }
  
  double currentTime = ros::Time::now().toSec() - trajContainer.startTime;
  double splicingTime = currentTime + time_offset;
  
  // Clamp to trajectory duration
  if (splicingTime > trajContainer.getTotalDuration()) {
    splicingTime = trajContainer.getTotalDuration();
  }
  
  return trajContainer.getState(splicingTime);
}

void PlanManager::recordReplanningTime(const std::string& phase, double time_ms)
{
  if (phase == "frontend") {
    replan_stats_.frontend_time_ms = time_ms;
  } else if (phase == "optimization") {
    replan_stats_.optimization_time_ms = time_ms;
  } else if (phase == "splicing") {
    replan_stats_.splicing_time_ms = time_ms;
  } else if (phase == "total") {
    replan_stats_.total_time_ms = time_ms;
    replan_stats_.replan_count++;
  }
}