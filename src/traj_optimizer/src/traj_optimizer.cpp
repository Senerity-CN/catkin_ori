#include <traj_optimizer/traj_optimizer.h>
// using namespace std;

/* main planning API */
bool PolyTrajOptimizer::OptimizeTrajectory(
    const std::vector<Eigen::MatrixXd> &iniStates, const std::vector<Eigen::MatrixXd> &finStates,
    std::vector<Eigen::MatrixXd> &initInnerPts, const Eigen::VectorXd &initTs,
    std::vector<std::vector<Eigen::MatrixXd>> &hPoly_container, std::vector<int> singuls, double help_eps)
{

  trajnum = initInnerPts.size();
  epis = help_eps;
  cfgHs_container = hPoly_container;
  iniState_container = iniStates;
  finState_container = finStates;
  singul_container = singuls;
  variable_num_ = 0;
  jerkOpt_container.clear();
  piece_num_container.clear();
  jerkOpt_container.resize(trajnum);
  piece_num_container.resize(trajnum);
  double final_cost;

  if (initTs.size() != trajnum)
  {
    ROS_ERROR("initTs.size()!=trajnum");
    return false;
  }
  if (initTs.minCoeff() < mini_T)
  {
    ROS_ERROR("mini segment T < mini_T!");
    return false;
  }
  std::cout << "initTs: " << initTs.transpose() << std::endl;

  for (int i = 0; i < trajnum; i++)
  {
    // check
    if (initInnerPts[i].cols() == 0)
    {
      ROS_ERROR("There is only a piece?");
      return false;
    }
    int piece_num_ = initInnerPts[i].cols() + 1;
    piece_num_container[i] = piece_num_;
    if (cfgHs_container[i].size() != (piece_num_ - 2) * (traj_res + 1) + 2 * (destraj_resolution_ + 1))
    {
      std::cout << "cfgHs size: " << cfgHs_container[i].size() << std::endl;
      ROS_ERROR("cfgHs size error!");
      return false;
    }
    for (int k = 0; k < (piece_num_ - 2) * (traj_res + 1) + 2 * (destraj_resolution_ + 1); k++)
    {
      cfgHs_container[i][k].topRows<2>().colwise().normalize(); // norm vector outside
    }

    // reset the start end max_vel_
    double max_vel, max_acc = 0.0;
    if (singuls[i] > 0)
    {
      max_vel = max_forward_vel;
      max_acc = max_forward_acc;
    }
    else
    {
      max_vel = max_backward_vel;
      max_acc = max_backward_acc;
    }
    if (iniState_container[i].col(1).norm() >= max_vel)
    {
      iniState_container[i].col(1) = iniState_container[i].col(1).normalized() * (max_vel - 1.0e-2);
    }
    if (finState_container[i].col(1).norm() >= max_vel)
    {
      finState_container[i].col(1) = finState_container[i].col(1).normalized() * (max_vel - 1.0e-2);
    }

    if (iniState_container[i].col(2).norm() >= max_acc)
    {
      iniState_container[i].col(2) = iniState_container[i].col(2).normalized() * (max_acc - 1.0e-2);
    }
    if (finState_container[i].col(2).norm() >= max_acc)
    {
      finState_container[i].col(2) = finState_container[i].col(2).normalized() * (max_acc - 1.0e-2);
    }

    jerkOpt_container[i].reset(piece_num_);
    variable_num_ += 2 * (piece_num_ - 1);
  }
  variable_num_ += trajnum;
  variable_num_ += 2 * (trajnum - 1);
  variable_num_ += 1 * (trajnum - 1);
  // variable_num_ += 2;
  // Waypoints + T + GearPosition + angle

  ros::Time t0 = ros::Time::now(), t1, t2;
  int restart_nums = 0, rebound_times = 0;
  bool flag_force_return, flag_still_occ, flag_success;
  Eigen::VectorXd x;
  x.resize(variable_num_);
  int offset = 0;
  for (int i = 0; i < trajnum; i++)
  {
    memcpy(x.data() + offset, initInnerPts[i].data(), initInnerPts[i].size() * sizeof(x[0]));
    offset += initInnerPts[i].size();
  }
  Eigen::Map<Eigen::VectorXd> Vt(x.data() + offset, initTs.size());
  RealT2VirtualT(initTs, Vt);

  offset += initTs.size();
  for (int i = 0; i < trajnum - 1; i++)
  {
    memcpy(x.data() + offset, finState_container[i].col(0).data(), 2 * sizeof(x[0]));
    offset += 2;
  }
  Eigen::Map<Eigen::VectorXd> angles(x.data() + offset, trajnum - 1);
  for (int i = 0; i < trajnum - 1; i++)
  {
    Eigen::Vector2d gearv = finState_container[i].col(1);
    angles[i] = std::atan2(gearv[1], gearv[0]);
  }

  /*for debug*/
  // offset +=  trajnum-1;
  // memcpy(x.data()+offset,finState_container[trajnum-1].col(1).data(), 2 * sizeof(x[0]));

  lbfgs::lbfgs_parameter_t lbfgs_params;
  lbfgs_params.mem_size = memsize; // 128
  lbfgs_params.past = past;        // 3
  lbfgs_params.g_epsilon = 1.0e-16;
  // lbfgs_params.g_epsilon = 0.1;
  lbfgs_params.min_step = 1.0e-32;
  lbfgs_params.delta = delta;
  lbfgs_params.max_iterations = 12000;

  if (angles.size() > 0)
    std::cout << "init angles: " << angles.transpose() << std::endl;

  /* ---------- prepare ---------- */
  iter_num_ = 0;
  flag_force_return = false;
  force_stop_type_ = DONT_STOP;
  flag_still_occ = false;
  flag_success = false;
  /* ---------- optimize ---------- */
  t1 = ros::Time::now();
  std::cout << "begin to optimize!\n";
  startT = ros::Time::now().toSec();
  int result = lbfgs::lbfgs_optimize(
      x,
      final_cost,
      PolyTrajOptimizer::costFunctionCallback,
      NULL,
      NULL,
      this,
      lbfgs_params);

  t2 = ros::Time::now();

  double time_ms = (t2 - t1).toSec() * 1000;
  double total_time_ms = (t2 - t0).toSec() * 1000;
  if (angles.size() > 0)
    std::cout << "Fin angles: " << angles.transpose() << std::endl;

  /* ---------- get result and check collision ---------- */
  if (result == lbfgs::LBFGS_CONVERGENCE ||
      result == lbfgs::LBFGS_CANCELED ||
      result == lbfgs::LBFGS_STOP || result == lbfgs::LBFGSERR_MAXIMUMITERATION)
  {
    flag_force_return = false;
    flag_success = true;
    printf("\033[32miter=%d,time(ms)=%5.3f,total_t(ms)=%5.3f,cost=%5.3f\n\033[0m", iter_num_, time_ms, total_time_ms, final_cost);
  }
  else if (result == lbfgs::LBFGSERR_MAXIMUMLINESEARCH)
  {
    printf("\033[32miter=%d,time(ms)=%5.3f,total_t(ms)=%5.3f,cost=%5.3f\n\033[0m", iter_num_, time_ms, total_time_ms, final_cost);
    // ROS_WARN("Lbfgs: The line-search routine reaches the maximum number of evaluations.");
    flag_force_return = false;
    flag_success = true;
  }
  else
  {
    printf("\033[31m[PolyTrajOptimizer]iter=%d,time(ms)=%5.3f, error.\n\033[0m", iter_num_, time_ms);
    ROS_WARN("Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
  }

  // initInnerPts  = ctrl_points_;
  // ros::shutdown();
  if (final_cost >= 50000.0)
  {
    ROS_ERROR("optimization fails! cost is too high!");
    flag_success = false;
  }
  return flag_success;
}

/* callbacks by the L-BFGS optimizer */
double PolyTrajOptimizer::costFunctionCallback(void *func_data, const Eigen::VectorXd &x, Eigen::VectorXd &grad)
{

  double total_smcost = 0.0, total_timecost = 0.0, penalty_cost = 0.0;
  PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);
  // double nowtime = ros::Time::now().toSec();
  // if(nowtime-opt->startT > 0.1){
  //   return;
  // }
  int offset = 0;

  std::vector<Eigen::Map<const Eigen::MatrixXd>> P_container;
  std::vector<Eigen::Map<Eigen::MatrixXd>> gradP_container;
  ;

  for (int trajid = 0; trajid < opt->trajnum; trajid++)
  {
    Eigen::Map<const Eigen::MatrixXd> P(x.data() + offset, 2, opt->piece_num_container[trajid] - 1);
    Eigen::Map<Eigen::MatrixXd> gradP(grad.data() + offset, 2, opt->piece_num_container[trajid] - 1);
    offset += 2 * (opt->piece_num_container[trajid] - 1);
    gradP.setZero();
    P_container.push_back(P);
    gradP_container.push_back(gradP);
  }
  Eigen::Map<const Eigen::VectorXd> t(x.data() + offset, opt->trajnum);
  Eigen::Map<Eigen::VectorXd> gradt(grad.data() + offset, opt->trajnum);
  offset += opt->trajnum;
  Eigen::VectorXd T(opt->trajnum);
  Eigen::VectorXd gradT(opt->trajnum);
  gradT.setZero();
  opt->VirtualT2RealT(t, T);
  std::vector<double> trajtimes;
  trajtimes.push_back(0.0);
  // T(i) is sum time of i-segment traj
  for (int trajid = 0; trajid < opt->trajnum; trajid++)
  {
    trajtimes.push_back(T[trajid]);
  }
  // Ini/Fin Gear Pos
  std::vector<Eigen::Map<const Eigen::MatrixXd>> Gear_container;
  std::vector<Eigen::Map<Eigen::MatrixXd>> gradGear_container;
  for (int trajid = 0; trajid < opt->trajnum - 1; trajid++)
  {
    Eigen::Map<const Eigen::MatrixXd> Gear(x.data() + offset, 2, 1);
    Eigen::Map<Eigen::MatrixXd> gradGear(grad.data() + offset, 2, 1);
    offset += 2;
    gradGear.setZero();
    Gear_container.push_back(Gear);
    gradGear_container.push_back(gradGear);
  }
  //
  Eigen::Map<const Eigen::VectorXd> Angles(x.data() + offset, opt->trajnum - 1);
  Eigen::Map<Eigen::VectorXd> gradAngles(grad.data() + offset, opt->trajnum - 1);
  gradAngles.setZero();

  /*for relax end v?*/
  // offset += opt->trajnum-1;
  // Eigen::Map<const Eigen::MatrixXd> endV (x.data()+offset, 2, 1);
  // Eigen::Map<Eigen::MatrixXd> gradendV (grad.data()+offset, 2, 1);
  // gradendV.setZero();

  // Eigen::VectorXd gradt;
  for (int trajid = 0; trajid < opt->trajnum; trajid++)
  {
    double smoo_cost;
    Eigen::VectorXd obs_surround_feas_qvar_costs(3);
    obs_surround_feas_qvar_costs.setZero();

    Eigen::MatrixXd IniS, FinS;
    IniS = opt->iniState_container[trajid];
    FinS = opt->finState_container[trajid];

    if (trajid > 0)
    {
      double theta = Angles[trajid - 1];
      IniS.col(0) = Gear_container[trajid - 1];
      IniS.col(1) = Eigen::Vector2d(-opt->non_sinv * cos(theta), -opt->non_sinv * sin(theta));
    }
    if (trajid < opt->trajnum - 1)
    {
      double theta = Angles[trajid];
      FinS.col(0) = Gear_container[trajid];
      FinS.col(1) = Eigen::Vector2d(opt->non_sinv * cos(theta), opt->non_sinv * sin(theta));
    }
    // relax end v?
    //  if(trajid == opt->trajnum - 1){
    //    FinS.col(1) = endV;
    //  }

    opt->jerkOpt_container[trajid].generate(P_container[trajid], T[trajid] / opt->piece_num_container[trajid], IniS, FinS);
    opt->jerkOpt_container[trajid].initSmGradCost(); // Smoothness cost
    smoo_cost = opt->jerkOpt_container[trajid].getTrajJerkCost();
    opt->addPVAGradCost2CT(obs_surround_feas_qvar_costs, trajid, trajtimes[trajid]); // Time int cost
    // Get gradT gradC
    total_smcost += smoo_cost;
    penalty_cost += obs_surround_feas_qvar_costs.sum();
    // std::cout<<"Trajid: "<<trajid<<" penalty: "<<obs_surround_feas_qvar_costs.transpose()<<std::endl;
  }

  for (int trajid = 0; trajid < opt->trajnum; trajid++)
  {
    double time_cost = 0.0;
    Eigen::Matrix<double, 2, 3> gradIni, gradFin;
    opt->jerkOpt_container[trajid].calGrads_PT(); // gdt gdp gdhead gdtail
    // waypoint
    gradP_container[trajid] = opt->jerkOpt_container[trajid].get_gdP();
    // init Fin
    gradIni = opt->jerkOpt_container[trajid].get_gdHead();
    gradFin = opt->jerkOpt_container[trajid].get_gdTail();
    if (opt->GearOpt)
    {
      if (trajid > 0)
      {
        double theta = Angles[trajid - 1];
        gradGear_container[trajid - 1] += gradIni.col(0);
        gradAngles[trajid - 1] += gradIni.col(1).transpose() * Eigen::Vector2d(opt->non_sinv * sin(theta), -opt->non_sinv * cos(theta));
      }
      if (trajid < opt->trajnum - 1)
      {
        double theta = Angles[trajid];
        gradGear_container[trajid] += gradFin.col(0);
        gradAngles[trajid] += gradFin.col(1).transpose() * Eigen::Vector2d(-opt->non_sinv * sin(theta), opt->non_sinv * cos(theta));
      }
    }

    // if(trajid ==  opt->trajnum-1){
    //   gradendV += gradFin.col(1);
    // }

    opt->VirtualTGradCost(T[trajid], t[trajid], opt->jerkOpt_container[trajid].get_gdT() / opt->piece_num_container[trajid], gradt[trajid], time_cost);

    // gradt[trajid] = 0.0;

    total_timecost += time_cost;
    // std::cout<<"gradp: \n"<<gradP_container[trajid] <<std::endl;
    // std::cout<<"gradeigen p: \n"<<opt->jerkOpt_container[trajid].get_gdP()<<std::endl;
  }

  opt->iter_num_ += 1;
  // std::cout << "grad angle : "<< gradAngles.transpose() << "\n";
  // std::cout << "angle : "<< Angles.transpose() << "\n";
  // std::cout <<"endV: " <<endV.transpose()<<"\n";

  return total_smcost + total_timecost + penalty_cost;
}

int PolyTrajOptimizer::earlyExitCallback(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
{
  PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

  return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
}

/* mappings between real world time and unconstrained virtual time */
template <typename EIGENVEC>
void PolyTrajOptimizer::RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT)
{
  for (int i = 0; i < RT.size(); ++i)
  {
    VT(i) = RT(i) > 1.0 + mini_T
                ? (sqrt(2.0 * RT(i) - 1.0 - 2 * mini_T) - 1.0)
                : (1.0 - sqrt(2.0 / (RT(i) - mini_T) - 1.0));
  }
}

template <typename EIGENVEC>
void PolyTrajOptimizer::VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT)
{
  for (int i = 0; i < VT.size(); ++i)
  {
    RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0) + mini_T
                        : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0) + mini_T;
  }
}

template <typename EIGENVEC, typename EIGENVECGD>
void PolyTrajOptimizer::VirtualTGradCost(
    const Eigen::VectorXd &RT, const EIGENVEC &VT,
    const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
    double &costT)
{
  for (int i = 0; i < VT.size(); ++i)
  {
    double gdVT2Rt;
    if (VT(i) > 0)
    {
      gdVT2Rt = VT(i) + 1.0;
    }
    else
    {
      double denSqrt = (0.5 * VT(i) - 1.0) * VT(i) + 1.0;
      gdVT2Rt = (1.0 - VT(i)) / (denSqrt * denSqrt);
    }

    gdVT(i) = (gdRT(i) + wei_time_) * gdVT2Rt;
  }

  costT = RT.sum() * wei_time_;
}
void PolyTrajOptimizer::VirtualTGradCost(const double &RT, const double &VT, const double &gdRT, double &gdVT, double &costT)
{
  double gdVT2Rt;
  if (VT > 0)
  {
    gdVT2Rt = VT + 1.0;
  }
  else
  {
    double denSqrt = (0.5 * VT - 1.0) * VT + 1.0;
    gdVT2Rt = (1.0 - VT) / (denSqrt * denSqrt);
  }

  gdVT = (gdRT + wei_time_) * gdVT2Rt;
  costT = RT * wei_time_;
}

void PolyTrajOptimizer::addPVAGradCost2CT(Eigen::VectorXd &costs, const int trajid, const double trajtime)
{

  // output gradT gradC
  int N = piece_num_container[trajid];
  Eigen::Vector2d outerNormal;
  Eigen::Vector2d sigma, dsigma, ddsigma, dddsigma, ddddsigma;
  double vel2_reci, vel2_reci_e, vel3_2_reci_e, acc2, cur2, cur, phi_dot;
  double latacc2;
  Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
  double s1, s2, s3, s4, s5;
  double step, alpha;
  Eigen::Matrix<double, 6, 2> gradViolaPc, gradViolaVc, gradViolaAc, gradViolaLatAc, gradViolaKc, gradViolaKLc, gradViolaKRc, gradViolaPhidotLc, gradViolaPhidotRc;
  double gradViolaPt, gradViolaVt, gradViolaAt, gradViolaLatAt, gradViolaKt, gradViolaKLt, gradViolaKRt, gradViolaPhidotLt, gradViolaPhidotRt;
  double violaPos, violaVel, violaAcc, violaLatAcc, violaCur, violaCurL, violaCurR, violaDynamicObs, violaPhidotL, violaPhidotR;
  double violaPosPenaD, violaVelPenaD, violaAccPenaD, violaLatAccPenaD, violaCurPenaD, violaCurPenaDL, violaCurPenaDR, violaDynamicObsPenaD, violaPhidotPenaDL, violaPhidotPenaDR;
  double violaPosPena, violaVelPena, violaAccPena, violaLatAccPena, violaCurPena, violaCurPenaL, violaCurPenaR, violaDynamicObsPena, violaPhidotPenaL, violaPhidotPenaR;
  double phidot_denominator, phidot_nominator;

  double approxcur2, approxviolaCur, approxviolaCurPenaD, approxviolaCurPena;
  Eigen::Matrix<double, 6, 2> gradapproxViolaKc;
  double gradapproxViolaKt;

  std::vector<Eigen::MatrixXd> cfgHs = cfgHs_container[trajid];
  int singul_ = singul_container[trajid];
  double max_vel, max_cur, max_acc;
  if (singul_ > 0)
  {
    max_vel = max_forward_vel;
    max_cur = max_forward_cur;
    max_acc = max_forward_acc;
  }
  else
  {
    max_vel = max_backward_vel;
    max_cur = max_backward_cur;
    max_acc = max_backward_acc;
  }

  double omg;
  int i_dp = 0; // the index of constrain points
  costs.setZero();
  double z_h0, z_h1, z_h2, z_h3, z_h4;
  double z1, z2, z3;
  double n1, n2, n3, n4, n5, n6;
  Eigen::Matrix2d ego_R, help_R;
  /*debug*/
  cos_points.clear();
  debug_hPolys.clear();
  key_points.clear();
  std::vector<int> cosindex;
  // int innerLoop;
  double t = 0;

  // debug
  double velcost = 0.0;
  double acccost = 0.0;
  double latacccost = 0.0;
  double curcost = 0.0;
  double phidotcost = 0.0;
  int pointid = -1;

  for (int i = 0; i < N; ++i)
  {
    int K;
    if (i == 0 || i == N - 1)
    {
      K = destraj_resolution_;
    }
    else
    {
      K = traj_res;
    }
    const Eigen::Matrix<double, 6, 2> &c = jerkOpt_container[trajid].getCoeffs().block<6, 2>(i * 6, 0);
    step = jerkOpt_container[trajid].getDt() / K; // T_i /k
    s1 = 0.0;
    // innerLoop = K;
    for (int j = 0; j <= K; ++j)
    {
      s2 = s1 * s1;
      s3 = s2 * s1;
      s4 = s2 * s2;
      s5 = s4 * s1;
      beta0 << 1.0, s1, s2, s3, s4, s5;
      beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
      beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
      beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
      beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120 * s1;
      alpha = 1.0 / K * j;

      // update s1 for the next iteration
      s1 += step;
      pointid++;

      sigma = c.transpose() * beta0;
      dsigma = c.transpose() * beta1;
      ddsigma = c.transpose() * beta2;
      dddsigma = c.transpose() * beta3;
      ddddsigma = c.transpose() * beta4;

      // ctrl_points_.col(i_dp) = sigma;
      omg = (j == 0 || j == K) ? 0.5 : 1.0;

      // some help values

      z_h0 = dsigma.norm();
      z_h1 = ddsigma.transpose() * dsigma;
      z_h2 = dddsigma.transpose() * dsigma;
      z_h3 = ddsigma.transpose() * B_h * dsigma;

      n1 = z_h0;
      n2 = n1 * n1;
      n3 = n2 * n1;
      n4 = n2 * n2;
      n5 = n3 * n2;
      n6 = n3 * n3;

      z1 = dddsigma.transpose() * B_h * dsigma;
      z2 = ddsigma.transpose() * B_h * dsigma;
      z3 = dsigma.transpose() * ddsigma;

      if (j != K || (j == K && i == N - 1))
      {
        ++i_dp;
      }

      // add cost z_h0 = ||v||
      if (z_h0 < 1e-4 || (j == 0 && i == 0) || (i == N - 1 && j == K))
      {
        continue;
      }
      // avoid siguality

      vel2_reci = 1.0 / (z_h0 * z_h0);
      vel2_reci_e = 1.0 / (z_h0 * z_h0 + epis);
      vel3_2_reci_e = vel2_reci_e * sqrt(vel2_reci_e);
      z_h0 = 1.0 / z_h0;

      z_h4 = z_h1 * vel2_reci;
      violaVel = 1.0 / vel2_reci - max_vel * max_vel;
      acc2 = z_h1 * z_h1 * vel2_reci;
      latacc2 = z_h3 * z_h3 * vel2_reci;
      cur2 = z_h3 * z_h3 * (vel2_reci_e * vel2_reci_e * vel2_reci_e);
      cur = z_h3 * vel3_2_reci_e;
      violaAcc = acc2 - max_acc * max_acc;
      violaLatAcc = latacc2 - max_latacc_ * max_latacc_;

      phidot_denominator = n6 + L_ * L_ * z2 * z2;
      phidot_nominator = L_ * (n3 * z1 - 3 * z2 * z3 * n1);
      phi_dot = phidot_nominator / phidot_denominator; // S/M

      //@hzc: add feasibility with curvature
      violaCur = cur2 - max_cur * max_cur;
      violaCurL = cur - max_cur;
      violaCurR = -cur - max_cur;
      violaPhidotL = phi_dot - max_phidot_;
      violaPhidotR = -phi_dot - max_phidot_;

      ego_R << dsigma(0), -dsigma(1),
          dsigma(1), dsigma(0);
      ego_R = singul_ * ego_R * z_h0;

      Eigen::Matrix2d temp_a, temp_v;
      temp_a << ddsigma(0), -ddsigma(1),
          ddsigma(1), ddsigma(0);
      temp_v << dsigma(0), -dsigma(1),
          dsigma(1), dsigma(0);
      Eigen::Matrix2d R_dot = singul_ * (temp_a * z_h0 - temp_v * vel2_reci * z_h0 * z_h1);

      for (auto le : vec_le_)
      {
        Eigen::Vector2d bpt = sigma + ego_R * le;

        Eigen::Matrix2d temp_l_Bl;
        temp_l_Bl << le(0), -le(1),
            le(1), le(0);

        int corr_k = cfgHs[pointid].cols();

        for (int k = 0; k < corr_k; k++)
        {
          outerNormal = cfgHs[pointid].col(k).head<2>();
          violaPos = outerNormal.dot(bpt - cfgHs[pointid].col(k).tail<2>());

          if (violaPos > 0)
          {
            positiveSmoothedL1(violaPos, violaPosPena, violaPosPenaD);

            gradViolaPc = beta0 * outerNormal.transpose() +
                          beta1 * outerNormal.transpose() * (singul_ * temp_l_Bl * z_h0 - ego_R * le * dsigma.transpose() * vel2_reci);

            gradViolaPt = alpha * outerNormal.transpose() * (dsigma + R_dot * le);

            jerkOpt_container[trajid].get_gdC().block<6, 2>(i * 6, 0) += omg * step * wei_obs_ * violaPosPenaD * gradViolaPc;
            jerkOpt_container[trajid].get_gdT() += omg * wei_obs_ * (violaPosPenaD * gradViolaPt * step + violaPosPena / K);

            costs(0) += omg * step * wei_obs_ * violaPosPena; // cost is the same
          }
        }
      }

      // ---------------------surrounding vehicle avoidance
      double gradt, grad_prev_t, costp;
      Eigen::Vector2d gradp, gradp2;

      if (violaVel > 0.0)
      {
        positiveSmoothedL1(violaVel, violaVelPena, violaVelPenaD);

        gradViolaVc = 2.0 * beta1 * dsigma.transpose(); // 6*2
        gradViolaVt = 2.0 * alpha * z_h1;               // 1*1
        jerkOpt_container[trajid].get_gdC().block<6, 2>(i * 6, 0) += omg * step * wei_feas_ * violaVelPenaD * gradViolaVc;
        jerkOpt_container[trajid].get_gdT() += omg * wei_feas_ * (violaVelPenaD * gradViolaVt * step + violaVelPena / K);
        costs(2) += omg * step * wei_feas_ * violaVelPena;
        velcost += omg * step * wei_feas_ * violaVelPena;
      }

      if (violaAcc > 0.0)
      {
        positiveSmoothedL1(violaAcc, violaAccPena, violaAccPenaD);
        gradViolaAc = 2.0 * beta1 * (z_h4 * ddsigma.transpose() - z_h4 * z_h4 * dsigma.transpose()) +
                      2.0 * beta2 * z_h4 * dsigma.transpose(); // 6*2
        gradViolaAt = 2.0 * alpha * (z_h4 * (ddsigma.squaredNorm() + z_h2) - z_h4 * z_h4 * z_h1);
        jerkOpt_container[trajid].get_gdC().block<6, 2>(i * 6, 0) += omg * step * wei_feas_ * violaAccPenaD * gradViolaAc;
        jerkOpt_container[trajid].get_gdT() += omg * wei_feas_ * (violaAccPenaD * gradViolaAt * step + violaAccPena / K);
        costs(2) += omg * step * wei_feas_ * violaAccPena;
        acccost += omg * step * wei_feas_ * violaAccPena;
      }

      /*violaCurL = cur-max_cur_;
      violaCurR = -cur-max_cur_;*/

      if (violaCurL > 0.0)
      {
        positiveSmoothedL1(violaCurL, violaCurPenaL, violaCurPenaDL);
        //@hzc
        gradViolaKLc = beta1 * (vel3_2_reci_e * ddsigma.transpose() * B_h - 3 * vel3_2_reci_e * vel2_reci_e * z_h3 * dsigma.transpose()) + beta2 * vel3_2_reci_e * dsigma.transpose() * B_h.transpose(); // 6*2
        gradViolaKLt = alpha * vel3_2_reci_e * (dddsigma.transpose() * B_h * dsigma - 3 * vel2_reci_e * z_h3 * z_h1);
        jerkOpt_container[trajid].get_gdC().block<6, 2>(i * 6, 0) += omg * step * wei_feas_ * 10.0 * violaCurPenaDL * gradViolaKLc;
        jerkOpt_container[trajid].get_gdT() += omg * wei_feas_ * 10.0 * (violaCurPenaDL * gradViolaKLt * step + violaCurPenaL / K);
        costs(2) += omg * step * wei_feas_ * 10.0 * violaCurPenaL;
        curcost += omg * step * wei_feas_ * 10.0 * violaCurPenaL;
      }
      if (violaCurR > 0.0)
      {
        positiveSmoothedL1(violaCurR, violaCurPenaR, violaCurPenaDR);
        //@hzc
        gradViolaKRc = -(beta1 * (vel3_2_reci_e * ddsigma.transpose() * B_h - 3 * vel3_2_reci_e * vel2_reci_e * z_h3 * dsigma.transpose()) + beta2 * vel3_2_reci_e * dsigma.transpose() * B_h.transpose()); // 6*2
        gradViolaKRt = -(alpha * vel3_2_reci_e * (dddsigma.transpose() * B_h * dsigma - 3 * vel2_reci_e * z_h3 * z_h1));
        jerkOpt_container[trajid].get_gdC().block<6, 2>(i * 6, 0) += omg * step * wei_feas_ * 10.0 * violaCurPenaDR * gradViolaKRc;
        jerkOpt_container[trajid].get_gdT() += omg * wei_feas_ * 10.0 * (violaCurPenaDR * gradViolaKRt * step + violaCurPenaR / K);
        costs(2) += omg * step * wei_feas_ * 10.0 * violaCurPenaR;
        curcost += omg * step * wei_feas_ * 10.0 * violaCurPenaR;
      }
    }
    t += jerkOpt_container[trajid].getDt();
  }
}

/// viola*, viola*Pena, viola*PenaD
void PolyTrajOptimizer::positiveSmoothedL1(const double &x, double &f, double &df)
{
  const double pe = 1.0e-4;
  const double half = 0.5 * pe;
  const double f3c = 1.0 / (pe * pe);
  const double f4c = -0.5 * f3c / pe;
  const double d2c = 3.0 * f3c;
  const double d3c = 4.0 * f4c;

  if (x < pe)
  {
    f = (f4c * x + f3c) * x * x * x;
    df = (d3c * x + d2c) * x * x;
  }
  else
  {
    f = x - half;
    df = 1.0;
  }
  //       df = x * x;
  // f = df *x;
  // df *= 3.0;
  return;
}
void PolyTrajOptimizer::positiveSmoothedL3(const double &x, double &f, double &df)
{
  df = x * x;
  f = df * x;
  df *= 3.0;

  return;
}

void PolyTrajOptimizer::getBoundPts(Eigen::Vector2d &position, double angle,
                                    std::vector<Eigen::Vector2d> &BoundVertices)
{

  BoundVertices.clear();

  double cos_theta = cos(angle);
  double sin_theta = sin(angle);

  double c_x = position(0) + veh_param_.d_cr() * cos_theta; // the vehicle model is based on the rear wheel
  double c_y = position(1) + veh_param_.d_cr() * sin_theta;
  double d_wx = veh_param_.width() / 2 * sin_theta;
  double d_wy = veh_param_.width() / 2 * cos_theta;
  double d_lx = veh_param_.length() / 2 * cos_theta;
  double d_ly = veh_param_.length() / 2 * sin_theta;
  // Counterclockwise from left-front vertex
  BoundVertices.push_back(Eigen::Vector2d(c_x - d_wx + d_lx, c_y + d_wy + d_ly));
  BoundVertices.push_back(Eigen::Vector2d(c_x - d_wx - d_lx, c_y - d_ly + d_wy));
  BoundVertices.push_back(Eigen::Vector2d(c_x + d_wx - d_lx, c_y - d_wy - d_ly));
  BoundVertices.push_back(Eigen::Vector2d(c_x + d_wx + d_lx, c_y + d_ly - d_wy));
}

/* helper functions */
void PolyTrajOptimizer::setParam(ros::NodeHandle nh, std::shared_ptr<Config> cfg_)
{
  wei_obs_ = 1000.0;
  half_margin = 0.1;
  wei_feas_ = 1000.0;
  GearOpt = true;

  traj_res = cfg_->traj_res;
  destraj_resolution_ = traj_res;
  wei_time_ = cfg_->wei_time_;
  max_forward_vel = cfg_->vmax;
  max_backward_vel = cfg_->vmax;
  max_forward_cur = cfg_->kmax;
  max_backward_cur = cfg_->kmax;
  max_forward_acc = cfg_->accRatemax;
  max_backward_acc = cfg_->accRatemax;
  max_latacc_ = cfg_->latAccmax;
  non_sinv = cfg_->non_siguav;
  memsize = cfg_->mem_size;
  past = cfg_->past;
  delta = cfg_->delta;
  mini_T = cfg_->mini_T;

  B_h << 0, -1,
      1, 0;

  veh_param_.set_width(veh_param_.width() + 2 * half_margin);
  veh_param_.set_length(veh_param_.length() + 2 * half_margin);

  double half_wid = 0.5 * veh_param_.width();
  double half_len = 0.5 * veh_param_.length();

  L_ = veh_param_.wheel_base();
  lz_set_.push_back(Eigen::Vector2d(half_len, half_wid));
  lz_set_.push_back(Eigen::Vector2d(half_len, -half_wid));
  lz_set_.push_back(Eigen::Vector2d(-half_len, -half_wid));
  lz_set_.push_back(Eigen::Vector2d(-half_len, half_wid));

  Eigen::Vector2d le_1, le_2, le_3, le_4; // vertexs of the ego car in the body frame
  Eigen::Vector2d lo_1, lo_2, lo_3, lo_4; // vertexs of the surround car in the body frame
  vec_le_.clear();
  vec_lo_.clear();
  le_1 << veh_param_.d_cr() + veh_param_.length() / 2.0, veh_param_.width() / 2.0;
  le_2 << veh_param_.d_cr() + veh_param_.length() / 2.0, -veh_param_.width() / 2.0;
  le_3 << veh_param_.d_cr() - veh_param_.length() / 2.0, -veh_param_.width() / 2.0;
  le_4 << veh_param_.d_cr() - veh_param_.length() / 2.0, veh_param_.width() / 2.0;
  lo_1 = le_1;
  lo_2 = le_2;
  lo_3 = le_3;
  lo_4 = le_4;

  // attention here! These vectors store one more of the vertexs! The vertexs are stored Clockwise！
  vec_le_.push_back(le_1);
  vec_le_.push_back(le_2);
  vec_le_.push_back(le_3);
  vec_le_.push_back(le_4);
  vec_le_.push_back(le_1); // !!!!!
  vec_lo_.push_back(lo_1);
  vec_lo_.push_back(lo_2);
  vec_lo_.push_back(lo_3);
  vec_lo_.push_back(lo_4);
  vec_lo_.push_back(lo_1); // !!!!!

  number_of_hyperplanes_of_ego_car_ = vec_le_.size() - 1;
  number_of_hyperplanes_of_surround_car_ = vec_lo_.size() - 1;
}

void PolyTrajOptimizer::displayPoints()
{
  visualization_msgs::Marker mk;
  mk.header.frame_id = "map";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::LINE_STRIP;
  mk.action = visualization_msgs::Marker::DELETE;

  debug_pub.publish(mk);
  geometry_msgs::Point pt;
  std_msgs::ColorRGBA pc;

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.w = 1.0;
  mk.scale.x = 0.1;

  pc.r = 0.5;
  pc.g = 0;
  pc.b = 0;
  pc.a = 0.6;

  for (int i = 0; i < ctrl_points_.cols(); i++)
  {

    pt.x = ctrl_points_(0, i);
    pt.y = ctrl_points_(1, i);
    if (isnan(pt.x) || isnan(pt.y) || abs(pt.x) >= 100 || abs(pt.y) >= 100)
      return;
    pt.z = 0.1;

    mk.points.push_back(pt);
    mk.colors.push_back(pc);
  }

  debug_pub.publish(mk);
}
void PolyTrajOptimizer::displayKeyPoints()
{
  visualization_msgs::Marker mk;
  mk.header.frame_id = "map";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;

  debug_key_points_pub.publish(mk);
  geometry_msgs::Point pt;
  std_msgs::ColorRGBA pc;

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.w = 1.0;
  mk.scale.x = 0.4;
  mk.scale.y = 0.4;
  mk.scale.z = 0.4;

  pc.r = 0.0;
  pc.g = 0;
  pc.b = 1.0;
  pc.a = 0.6;

  for (int i = 0; i < key_points.size(); i++)
  {

    pt.x = key_points[i][0];
    pt.y = key_points[i][1];
    if (isnan(pt.x) || isnan(pt.y) || abs(pt.x) >= 100 || abs(pt.y) >= 100)
      return;
    pt.z = 0.3;

    mk.points.push_back(pt);
    mk.colors.push_back(pc);
  }

  debug_key_points_pub.publish(mk);
}

void PolyTrajOptimizer::displayCosPoints()
{
  visualization_msgs::Marker mk;
  mk.header.frame_id = "map";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.scale.x = 0.1;
  mk.action = visualization_msgs::Marker::DELETE;

  debug_pub1.publish(mk);
  geometry_msgs::Point pt;
  std_msgs::ColorRGBA pc;

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.w = 1.0;
  mk.scale.x = 0.1;

  pc.r = 0.0;
  pc.g = 0;
  pc.b = 0;
  pc.a = 0.6;
  // std::cout<<"11111111111111111111111111111111111111\n";
  for (int i = 0; i < cos_points.size(); i++)
  {
    //  std::cout<<"cos: "<<cos_points[i].transpose()<<std::endl;
    pt.x = cos_points[i][0];
    pt.y = cos_points[i][1];
    if (isnan(pt.x) || isnan(pt.y) || abs(pt.x) >= 100 || abs(pt.y) >= 100)
      return;
    pt.z = 0.1;

    mk.points.push_back(pt);
    mk.colors.push_back(pc);
  }

  debug_pub1.publish(mk);
}
