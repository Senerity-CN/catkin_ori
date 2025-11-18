#ifndef SE2TRAJOPT.HPP
#define SE2TRAJOPT.HPP
#include "se2Polytraj.hpp"
#include "lbfgs.hpp"
#include "geoutils2d.hpp"
#include <tools/gridmap.hpp>
#include <tools/config.hpp>
// using namespace std;
/*to do list
  vel constraint 
  lon acc constraint
  lat acc constraint
  yaw dot constraint
*/
namespace se2Plan
{
class Se2TrajOpt{
    private:
        
        int piecenum2d, piecenum1d;
        double mini_T = 0.0;
        double wei_time_ = 500.0;
        double kmax = 0.45;
        double vmax = 3.0;
        double latAccmax = 3.0;
        double lonAccmax = 3.0;
        double accRatemax = 8.0;
        double kdotmax = 5.0;
        int traj_res = 16;
        Eigen::MatrixXd iniState2d_, finState2d_;
        Eigen::VectorXd iniState1d_, finState1d_;
        std::vector<Eigen::MatrixXd> cfgHs_;
        std::vector<Eigen::Vector2d> conpts;
        // = {Eigen::Vector2d(3.455, 0.95), 
        //                                        Eigen::Vector2d(3.455, -0.95),
        //                                        Eigen::Vector2d(-1.425, 0.95),
        //                                        Eigen::Vector2d(-1.425, -0.95)};
        // std::vector<Eigen::Vector2d> conpts = {Eigen::Vector2d(3.0, 0.7), 
        // Eigen::Vector2d(3.0, -0.7),
        // Eigen::Vector2d(-1.4, 0.7),
        // Eigen::Vector2d(-1.4, -0.7)};
        /*lots of alm param*/
        double scaling_wf_ = 1.0;
        Eigen::VectorXd scaling_eqwc_, scaling_uneqwc_;
        double scaling_wf_min_ = 0.01, scaling_wc_min_ = 0.01;
        int eqc_num = 0, neqc_num = 0, totalc_num = 0;
        Eigen::VectorXd lambda, mu, hx, gx;
        double rho = 1.0, rho_max =1000.0;
        double gamma = 1;
        double cons_eps_ = 0.20;
        Eigen::VectorXd gradx;
        double almTotalcost;


    public:
        // se2Plan::MinJerkOpt<3> jerkOpt;
        se2Plan::MinJerkOpt1d yawJerkOpt;
        se2Plan::MinJerkOpt2d posJerkOpt;
        

        inline int OptimizeSe2Trajectory(
            const Eigen::MatrixXd &iniState2d, const Eigen::MatrixXd &finState2d, const Eigen::MatrixXd &initInnerPts2d, 
            const Eigen::VectorXd &iniState1d, const Eigen::VectorXd &finState1d, const Eigen::VectorXd &initInnerPts1d, 
            const double & totalt, const std::vector<Eigen::MatrixXd> &hPolys, std::shared_ptr<Config> config_)
        {
            /*setparam*/
            mini_T = config_->mini_T;
            wei_time_ = config_->wei_time_;
            kmax = config_->kmax;
            vmax = config_->vmax;
            latAccmax = config_->latAccmax;
            lonAccmax = config_->lonAccmax;
            accRatemax = config_->accRatemax;
            kdotmax = config_->kdotmax;
            traj_res = config_->traj_res;
            rho = config_->rho;
            rho_max = config_->rho_max;
            gamma = config_->gamma;
            cons_eps_ = config_->cons_eps_;
            conpts = config_->conpts;
            

            //totalt is total time
            iniState2d_ = iniState2d;
            finState2d_ = finState2d;
            iniState1d_ = iniState1d;
            finState1d_ = finState1d;
            cfgHs_ = hPolys;
            piecenum2d = initInnerPts2d.cols() + 1;
            piecenum1d = initInnerPts1d.size() + 1;
            posJerkOpt.reset(piecenum2d);
            yawJerkOpt.reset(piecenum1d);
            ROS_WARN("begin to optimize Se2Trajectory");
            std::cout << "piecenum2d: " << piecenum2d << " piecenum1d: "<< piecenum1d << std::endl;
            int variable_num_ = 0;
            variable_num_ += 2 * (piecenum2d - 1); //wp
            variable_num_ += 1 * (piecenum1d - 1); //wp
            variable_num_ += 1; //t 
            /*set scaing*/
            eqc_num = piecenum2d * traj_res * 1;
            neqc_num = piecenum2d * traj_res * 5; // cur vel lat  lon accRate 
            //add corridor constraints 
            for(const auto hpoly : hPolys){
                neqc_num += conpts.size() * hpoly.cols();
            }
            totalc_num = eqc_num + neqc_num;
            scaling_wf_ = 1.0;
            scaling_eqwc_.resize(eqc_num); scaling_eqwc_.setConstant(1.0);
            scaling_uneqwc_.resize(totalc_num); scaling_uneqwc_.setConstant(1.0);
            hx.resize(eqc_num); lambda.resize(eqc_num);
            hx.setZero();       lambda.setZero();
            gx.resize(neqc_num); mu.resize(neqc_num);
            gx.setZero();        mu.setZero(); 
            Eigen::VectorXd x;
            x.resize(variable_num_);
            int offset = 0;
            memcpy(x.data()+offset,initInnerPts2d.data(), initInnerPts2d.size() * sizeof(x[0]));
            offset += initInnerPts2d.size();
            memcpy(x.data()+offset,initInnerPts1d.data(), initInnerPts1d.size() * sizeof(x[0]));
            offset += initInnerPts1d.size();
            double vt;
            RealT2VirtualT(totalt,vt);
            x[offset] = vt;

            lbfgs::lbfgs_parameter_t lbfgs_params;
            lbfgs_params.mem_size = config_->mem_size;//128
            lbfgs_params.past = config_->past; //3 
            lbfgs_params.g_epsilon = config_->g_epsilon;
            lbfgs_params.min_step = config_->min_step;
            lbfgs_params.delta = config_->delta;
            lbfgs_params.max_iterations = config_->max_iterations;
            double final_cost;
            bool flag_success = true;

            /*ALM*/
            double t1 = ros::Time::now().toSec();
            // initScalingW(x);
            int iter = 0;
            while(true){
                int result = lbfgs::lbfgs_optimize(
                x,
                final_cost,
                Se2TrajOpt::almCostFunctionCallback,
                NULL,
                NULL,
                this,
                lbfgs_params);

                if (result == lbfgs::LBFGS_CONVERGENCE ||
                result == lbfgs::LBFGS_CANCELED ||
                result == lbfgs::LBFGS_STOP||result == lbfgs::LBFGSERR_MAXIMUMITERATION)
                {
                ROS_INFO_STREAM("se2 trajectory generation success! cost: " << final_cost );             } 
                else if (result == lbfgs::LBFGSERR_MAXIMUMLINESEARCH){
                ROS_WARN("Lbfgs: The line-search routine reaches the maximum number of evaluations.");
                }
                else
                {
                    ROS_WARN("Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
                }
                updateDualVar();
                if(ifConvergence()){
                    ROS_WARN_STREAM("Convergence! iter: "<<iter);
                    break;
                }
                iter++;
                if(iter > config_->amlMaxIter){
                    ROS_WARN("Reach Max iteration");
                    break;
                }
            }
            double t2 = ros::Time::now().toSec();
            ROS_INFO_STREAM("alm time: "<<(t2-t1) * 1000.0 << " ms" <<" jerk cost: "<<posJerkOpt.getTrajJerkCost() << " traj time:" << posJerkOpt.getTraj().getTotalDuration());
            printf("\033[32m,time(ms)=%5.3f \n\033[0m", (t2-t1) * 1000.0);

            return flag_success;
        }
        static inline double almCostFunctionCallback(void *func_data, const Eigen::VectorXd &x, Eigen::VectorXd &grad){
            double smcost = 0.0, timecost = 0.0, almcost = 0.0;
            Eigen::MatrixXd gdS2c_2d, gdS2c_1d, gdC2c_2d, gdC2c_1d, gdTotalc_2d, gdTotalc_1d;
            double gdS2t_2d, gdS2t_1d, gdC2t_2d, gdC2t_1d, gdTotalt_2d, gdTotalt_1d;
            Se2TrajOpt *opt = reinterpret_cast<Se2TrajOpt *>(func_data);
            int offset = 0;
            Eigen::Map<const Eigen::MatrixXd> P_2d(x.data()+offset, 2, opt->piecenum2d - 1);
            Eigen::Map<Eigen::MatrixXd>gradP_2d(grad.data()+offset, 2, opt->piecenum2d - 1);
            gradP_2d.setZero();
            offset += 2 * (opt->piecenum2d - 1);
            Eigen::Map<const Eigen::MatrixXd> P_1d(x.data()+offset, 1, opt->piecenum1d - 1);
            Eigen::Map<Eigen::MatrixXd>gradP_1d(grad.data()+offset, 1, opt->piecenum1d - 1);
            gradP_1d.setZero();
            offset += opt->piecenum1d - 1;
            const double t = x[offset]; //total virtual time
            double gradt, T;
            opt->VirtualT2RealT(t, T);
            /*pos traj mini jerk*/
            opt->posJerkOpt.generate(P_2d,T / opt->piecenum2d, opt->iniState2d_, opt->finState2d_);
            opt->posJerkOpt.initSmGradCost(gdS2c_2d,gdS2t_2d); // Smoothness cost   
            smcost += opt->scaling_wf_ * opt->posJerkOpt.getTrajJerkCost();
            gdS2c_2d *= opt->scaling_wf_;
            gdS2t_2d *= opt->scaling_wf_;
            /*angle traj mini jerk*/
            opt->yawJerkOpt.generate(P_1d,T / opt->piecenum1d, opt->iniState1d_.transpose(), opt->finState1d_.transpose());
            opt->yawJerkOpt.initSmGradCost(gdS2c_1d,gdS2t_1d); // Smoothness cost   
            smcost += opt->scaling_wf_ * opt->yawJerkOpt.getTrajJerkCost();
            gdS2c_1d *= opt->scaling_wf_;
            gdS2t_1d *= opt->scaling_wf_;
            opt->almGradCost2CT(almcost, gdC2c_2d, gdC2t_2d, gdC2c_1d, gdC2t_1d); // Time int cost
            // opt->addPVAGradCost2CT(almcost, gdCc, gdCt);
            //Get gradT gradC
            gdTotalc_2d = gdS2c_2d + gdC2c_2d;
            gdTotalc_1d = gdS2c_1d + gdC2c_1d;
            gdTotalt_2d = gdS2t_2d + gdC2t_2d;
            gdTotalt_1d = gdS2t_1d + gdC2t_1d;
            Eigen::MatrixXd propogate_gradp_2d, propogate_gradp_1d;
            opt->posJerkOpt.calGrads_PT(gdTotalc_2d, gdTotalt_2d, propogate_gradp_2d); // gdt gdp gdhead gdtail
            gradP_2d = propogate_gradp_2d;
            opt->yawJerkOpt.calGrads_PT(gdTotalc_1d, gdTotalt_1d, propogate_gradp_1d); // gdt gdp gdhead gdtail
            gradP_1d = propogate_gradp_1d;
            timecost = opt->wei_time_* T * opt->scaling_wf_;
            double gdTotalT = 0.0;
            gdTotalT += opt->wei_time_* opt->scaling_wf_;
            gdTotalT += gdTotalt_2d/ opt->piecenum2d;
            gdTotalT += gdTotalt_1d/ opt->piecenum1d;
            // gdTotalt 
            opt->VirtualTGrad2t(t, gdTotalT, gradt);
            grad[offset] = gradt;
            opt->gradx = grad;
            opt->almTotalcost = smcost + timecost + almcost;
            return smcost + timecost + almcost;
        }
        template <typename EIGENVEC>
        void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT)
        {
            for (int i = 0; i < VT.size(); ++i)
            {
            RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0) + mini_T
                                : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0) + mini_T;
            }
        }
        void VirtualT2RealT(const  double & VT, double &RT){
            
            
            RT = VT > 0.0 ? ((0.5 * VT + 1.0) * VT + 1.0) + mini_T
                                : 1.0 / ((0.5 * VT - 1.0) * VT + 1.0) + mini_T;
        }
        template <typename EIGENVEC>
        inline void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT)
        {
        for (int i = 0; i < RT.size(); ++i)
            {
                VT(i) = RT(i) > 1.0 + mini_T 
              ? (sqrt(2.0 * RT(i) - 1.0 - 2 * mini_T) - 1.0)
              : (1.0 - sqrt(2.0 / (RT(i)-mini_T) - 1.0));
            }
        }
        inline void RealT2VirtualT(const double &RT, double &VT)
        {
        
            VT = RT > 1.0 + mini_T 
            ? (sqrt(2.0 * RT - 1.0 - 2 * mini_T) - 1.0)
            : (1.0 - sqrt(2.0 / (RT-mini_T) - 1.0));
        }
        template <typename EIGENVEC, typename EIGENVECGD>
        void VirtualTGradCost(const Eigen::VectorXd &RT, const EIGENVEC &VT,const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,double &costT)
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
        void VirtualTGradCost(const double &RT, const double &VT, const double &gdRT, double &gdVT, double& costT){
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
        void VirtualTGrad2t(const double &VT, const double &gdRT, double &gdVT){
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
            gdVT = gdRT * gdVT2Rt;
        }      
        double expC2(double t) {
            return t > 0.0 ? ((0.5 * t + 1.0) * t + 1.0)
                 : 1.0 / ((0.5 * t - 1.0) * t + 1.0);
        }
        double logC2(double T) {
            return T > 1.0 ? (sqrt(2.0 * T - 1.0) - 1.0) : (1.0 - sqrt(2.0 / T - 1.0));
        }
        void forwardT(const Eigen::Ref<const Eigen::VectorXd>& t, const double& sT, Eigen::Ref<Eigen::VectorXd> vecT) {
            int M = t.size();
            for (int i = 0; i < M; ++i) {
                vecT(i) = expC2(t(i));
            }
            vecT(M) = 0.0;
            vecT /= 1.0 + vecT.sum();
            vecT(M) = 1.0 - vecT.sum();
            vecT *= sT;
            return;
        }
        void backwardT(const Eigen::Ref<const Eigen::VectorXd>& vecT, Eigen::Ref<Eigen::VectorXd> t) {
            int M = t.size();
            t = vecT.head(M) / vecT(M);
            for (int i = 0; i < M; ++i) {
            t(i) = logC2(vecT(i));
            }
            return;
        }
        void almGradCost2CT(double &cost, Eigen::MatrixXd& gdC2c_2d, double& gdC2t_2d, Eigen::MatrixXd& gdC2c_1d, double& gdC2t_1d){
            // output gradT gradC 
            int N = piecenum2d;
            double yaw, dyaw, ddyaw, dddyaw, ddddyaw, latAcc, lonAcc, vnorm, cur;
            Eigen::Vector2d pos, vel, acc, jerk, snap;
            double gradYaw, gradDyaw, gradDdyaw, gradDddyaw;
            Eigen::Vector2d gradPos, gradVel, gradAcc, gradJerk;
            Eigen::Matrix<double, 6, 1> beta0_2d, beta1_2d, beta2_2d, beta3_2d, beta4_2d;
            Eigen::Matrix<double, 6, 1> beta0_1d, beta1_1d, beta2_1d, beta3_1d, beta4_1d;
            Eigen::Matrix2d rotR, drotR;
            Eigen::Vector2d outerNormal;
            Eigen::Matrix<double, 6, 1> t_2d, t_1d;
            double step, alpha, eqf;
            double vioCur, vioVel, vioLonAcc, vioLatAcc, vioAcc, vioCor;
            int yawPieceId,corId = -1;
            double baset = 0.0;
            t_2d(0) = 1.0;
            t_1d(0) = 1.0;
            
            
            gdC2c_2d.resize(6 * piecenum2d, 2);
            gdC2c_2d.setZero();
            gdC2t_2d = 0.0;
            gdC2c_1d.resize(6 * piecenum1d, 1);
            gdC2c_1d.setZero();
            gdC2t_1d = 0.0;
            int eqc_idx = 0;
            int uneqc_idx = 0;


            for (int i = 0; i < N; ++i)
            {
                const Eigen::Matrix<double, 6, 2> &c_2d = posJerkOpt.getCoeffs().block<6, 2>(i * 6, 0);
                step = posJerkOpt.getDt() / traj_res; // T_i /k
                // innerLoop = traj_res;
                for (int j = 1; j <= traj_res; ++j)
                {
                    /*analyse pos*/
                    t_2d(1) = step * j;
                    t_2d(2) = t_2d(1) * t_2d(1);
                    t_2d(3) = t_2d(2) * t_2d(1);
                    t_2d(4) = t_2d(2) * t_2d(2);
                    t_2d(5) = t_2d(4) * t_2d(1);
                    beta0_2d  = t_2d;
                    beta1_2d << 0.0, 1.0, 2.0 * t_2d(1), 3.0 * t_2d(2), 4.0 * t_2d(3), 5.0 * t_2d(4);
                    beta2_2d << 0.0, 0.0, 2.0, 6.0 * t_2d(1), 12.0 * t_2d(2), 20.0 * t_2d(3);
                    beta3_2d << 0.0, 0.0, 0.0, 6.0, 24.0 * t_2d(1), 60.0 * t_2d(2);
                    beta4_2d << 0.0, 0.0, 0.0, 0.0, 24.0, 120 * t_2d(1);
                    pos = c_2d.transpose() * beta0_2d;
                    vel = c_2d.transpose() * beta1_2d;
                    acc = c_2d.transpose() * beta2_2d;
                    jerk = c_2d.transpose() * beta3_2d;
                    snap = c_2d.transpose() * beta4_2d;
                    gradPos.setZero();
                    gradVel.setZero();
                    gradAcc.setZero();
                    gradJerk.setZero();
                    alpha = 1.0 / traj_res * j;
                    /*analyse yaw*/
                    double t_now = t_2d(1) + baset;
                    //locate the pieceid of yaw polytraj
                    yawPieceId = int((t_now) / yawJerkOpt.getDt());
                    if(yawPieceId >= piecenum1d){
                        yawPieceId = piecenum1d - 1;
                    }
                    const Eigen::Matrix<double, 6, 1> &c_1d = yawJerkOpt.getCoeffs().block<6, 1>(yawPieceId * 6, 0);
                    t_1d(1) = t_now - yawPieceId * yawJerkOpt.getDt();
                    t_1d(2) = t_1d(1) * t_1d(1);
                    t_1d(3) = t_1d(2) * t_1d(1);
                    t_1d(4) = t_1d(2) * t_1d(2);
                    t_1d(5) = t_1d(4) * t_1d(1);
                    beta0_1d  = t_1d;
                    beta1_1d << 0.0, 1.0, 2.0 * t_1d(1), 3.0 * t_1d(2), 4.0 * t_1d(3), 5.0 * t_1d(4);
                    beta2_1d << 0.0, 0.0, 2.0, 6.0 * t_1d(1), 12.0 * t_1d(2), 20.0 * t_1d(3);
                    beta3_1d << 0.0, 0.0, 0.0, 6.0, 24.0 * t_1d(1), 60.0 * t_1d(2);
                    beta4_1d << 0.0, 0.0, 0.0, 0.0, 24.0, 120 * t_1d(1);
                    yaw = c_1d.transpose() * beta0_1d;
                    dyaw = c_1d.transpose() * beta1_1d;
                    ddyaw= c_1d.transpose() * beta2_1d;
                    dddyaw = c_1d.transpose() * beta3_1d;
                    ddddyaw = c_1d.transpose() * beta4_1d;
                    gradYaw = 0.0;
                    gradDyaw = 0.0;
                    gradDdyaw = 0.0;
                    gradDddyaw = 0.0;
                    corId ++;
                    /*analyse state*/
                    lonAcc = acc[0] * cos(yaw) + acc[1] * sin(yaw);
                    latAcc = -acc[0] * sin(yaw) + acc[1] * cos(yaw);
                    vnorm = vel.norm();
                    cur = dyaw / (sqrt(vnorm * vnorm + RESDELTA));
                    rotR << cos(yaw), -sin(yaw),
                            sin(yaw),  cos(yaw);
                    drotR << -sin(yaw), -cos(yaw),
                              cos(yaw), -sin(yaw);
                    /*motion model constraint*/
                    {
                        eqf = vel.dot(Eigen::Vector2d(sin(yaw),-cos(yaw)));
                        double swc = scaling_eqwc_[eqc_idx];
                        double lbd = lambda[eqc_idx];
                        cost +=  swc * eqf * (lbd + 0.5 * rho * swc * eqf);
                        hx[eqc_idx] = eqf;
                        Eigen::Vector2d gradViolaMv;
                        double  gradViolaMtheta; 
                        gradViolaMv = Eigen::Vector2d(sin(yaw),-cos(yaw)); 
                        gradViolaMtheta = vel.transpose()* Eigen::Vector2d(cos(yaw),sin(yaw));
                        gradVel += addAlmGradPro(swc, lbd, eqf, gradViolaMv);
                        gradYaw += addAlmGradPro(swc, lbd, eqf, gradViolaMtheta);
                        eqc_idx ++;
                    }
                    /*curvature constraint*/ 
                    {
                        double curs = 10.0;
                        vioCur = curs*(cur*cur - kmax*kmax);
                        double swc = scaling_uneqwc_[uneqc_idx];
                        double tmu = mu[uneqc_idx];
                        gx[uneqc_idx] = vioCur;
                        if(tmu + rho * swc * vioCur > 0){
                            cost += swc * vioCur * (tmu + 0.5 * rho * swc * vioCur);
                            double temp = sqrt(vnorm * vnorm + RESDELTA);
                            Eigen::Vector2d gradViolaCv;
                            double  gradViolaCdtheta; 
                            gradViolaCv = curs*(-2) * cur * dyaw * vel / (temp * temp * temp);
                            gradViolaCdtheta = curs*2 * cur / (temp);
                            gradVel += addAlmGradPro(swc, tmu, vioCur, gradViolaCv);
                            gradDyaw += addAlmGradPro(swc, tmu, vioCur, gradViolaCdtheta);
                        }
                        else{
                            cost += -0.5 * tmu * tmu / rho;
                        }
                        uneqc_idx++;
                    }


                    /*vel constraint*/
                    {
                        vioVel = vel.transpose() * vel - vmax * vmax;
                        double swc = scaling_uneqwc_[uneqc_idx];
                        double tmu = mu[uneqc_idx];
                        gx[uneqc_idx] = vioVel;
                        if(tmu + rho * swc * vioVel > 0){
                            cost += swc * vioVel * (tmu + 0.5 * rho * swc * vioVel);
                            Eigen::Vector2d gradViolaVv;
                            gradViolaVv = 2 * vel;
                            gradVel += addAlmGradPro(swc, tmu, vioVel, gradViolaVv);
                        }
                        else{
                            cost += -0.5 * tmu * tmu / rho;
                        }
                        uneqc_idx++;
                    }
                    /*lon acc constraint*/
                    {
                        vioLonAcc = lonAcc * lonAcc - lonAccmax * lonAccmax;
                        double swc = scaling_uneqwc_[uneqc_idx];
                        double tmu = mu[uneqc_idx];
                        gx[uneqc_idx] = vioLonAcc;
                        if(tmu + rho * swc * vioLonAcc > 0){
                            cost += swc * vioLonAcc * (tmu + 0.5 * rho * swc * vioLonAcc);
                            Eigen::Vector2d gradViolaLonAa;
                            double  gradViolaLonAtheta; 
                            gradViolaLonAa = 2 * lonAcc * Eigen::Vector2d(cos(yaw),sin(yaw));
                            gradViolaLonAtheta = 2 * lonAcc * latAcc;
                            gradAcc += addAlmGradPro(swc, tmu, vioLonAcc, gradViolaLonAa);
                            gradYaw += addAlmGradPro(swc, tmu, vioLonAcc, gradViolaLonAtheta);

                        }
                        else{
                            cost += -0.5 * tmu * tmu / rho;
                        }
                        uneqc_idx++;
                    }
                    /*lat acc constraint*/
                    {
                        vioLatAcc = latAcc * latAcc - latAccmax * latAccmax;
                        double swc = scaling_uneqwc_[uneqc_idx];
                        double tmu = mu[uneqc_idx];
                        gx[uneqc_idx] = vioLatAcc;
                        if(tmu + rho * swc * vioLatAcc > 0){
                            cost += swc * vioLatAcc * (tmu + 0.5 * rho * swc * vioLatAcc);
                            Eigen::Vector2d gradViolaLatAa;
                            double  gradViolaLatAtheta; 
                            gradViolaLatAa = 2 * latAcc * Eigen::Vector2d(-sin(yaw),cos(yaw));
                            gradViolaLatAtheta = -2 * latAcc * lonAcc;
                            gradAcc += addAlmGradPro(swc, tmu, vioLatAcc, gradViolaLatAa);
                            gradYaw += addAlmGradPro(swc, tmu, vioLatAcc, gradViolaLatAtheta);

                        }
                        else{
                            cost += -0.5 * tmu * tmu / rho;
                        }
                        uneqc_idx++;
                    }
                    /*acc constraint*/
                    {
                        vioAcc = acc.dot(acc) - accRatemax * accRatemax;
                        double swc = scaling_uneqwc_[uneqc_idx];
                        double tmu = mu[uneqc_idx];
                        gx[uneqc_idx] = vioAcc;
                        if(tmu + rho * swc * vioAcc > 0){
                            cost += swc * vioAcc * (tmu + 0.5 * rho * swc * vioAcc);
                            Eigen::Vector2d gradViolaAa;
                            gradViolaAa = 2 * acc;
                            gradAcc += addAlmGradPro(swc, tmu, vioAcc, gradViolaAa);
                        }
                        else{
                            cost += -0.5 * tmu * tmu / rho;
                        }
                        uneqc_idx++;
                    }
                    /*dotcur constraint*/
                    // {
                    //     double v2norm = sqrt(vel.transpose()*vel + RESDELTA);
                    //     double v2squr = vel.transpose()*vel + RESDELTA;
                    //     double kdot = (ddyaw*v2squr - vel.dot(acc)*dyaw)/(v2squr * v2norm);
                    //     double viokdot = kdot * kdot - kdotmax * kdotmax;
                    //     double swc = scaling_uneqwc_[uneqc_idx];
                    //     double tmu = mu[uneqc_idx];
                    //     gx[uneqc_idx] = viokdot;
                    //     if(tmu + rho * swc * viokdot > 0){
                    //         cost += swc * viokdot * (tmu + 0.5 * rho * swc * viokdot);
                    //         double gradViolaKdotdyaw = (-vel.dot(acc)) / (v2squr * v2norm);
                    //         double gradViolaKdotddyaw = 1 / v2norm;
                    //         Eigen::Vector2d gradViolaKdotvel = (2 * ddyaw * vel - dyaw * acc) / (v2squr * v2norm) - 
                    //                                 (3 * (ddyaw*v2squr - vel.dot(acc)*dyaw)*vel)/(v2squr * v2squr * v2norm);
                    //         Eigen::Vector2d gradViolaKdotacc = -(dyaw * vel)/(v2squr * v2norm);
                    //         gradViolaKdotdyaw = 2 * kdot * gradViolaKdotdyaw;
                    //         gradViolaKdotddyaw = 2 * kdot * gradViolaKdotddyaw;
                    //         gradViolaKdotvel = 2 * kdot * gradViolaKdotvel;
                    //         gradViolaKdotacc = 2 * kdot * gradViolaKdotacc;
                    //         gradDyaw += addAlmGradPro(swc, tmu, viokdot, gradViolaKdotdyaw);
                    //         gradDdyaw += addAlmGradPro(swc, tmu, viokdot, gradViolaKdotddyaw);
                    //         gradVel += addAlmGradPro(swc, tmu, viokdot, gradViolaKdotvel);
                    //         gradAcc += addAlmGradPro(swc, tmu, viokdot, gradViolaKdotacc);

                    //     }
                    //     else{
                    //         cost += -0.5 * tmu * tmu / rho;
                    //     }
                    //     uneqc_idx++;
                    // }
                    /*static corridor constraint*/
                    {
                        for(const auto conpt : conpts){
                            Eigen::Vector2d bpt = pos + rotR * conpt;
                            int corr_k = cfgHs_[corId].cols();
                            for(int k = 0; k < corr_k; k ++){
                                outerNormal = cfgHs_[corId].col(k).head<2>();
                                vioCor = outerNormal.dot(bpt - cfgHs_[corId].col(k).tail<2>());
                                double swc = scaling_uneqwc_[uneqc_idx];
                                double tmu = mu[uneqc_idx];
                                gx[uneqc_idx] = vioCor;
                                if(tmu + rho * swc * vioCor > 0){
                                    cost += swc * vioCor * (tmu + 0.5 * rho * swc * vioCor);
                                    Eigen::Vector2d gradViolaScP = outerNormal;
                                    double gradViolaSctheta = outerNormal.transpose() * drotR * conpt;
                                    gradPos += addAlmGradPro(swc, tmu, vioCor, gradViolaScP);
                                    gradYaw += addAlmGradPro(swc, tmu, vioCor, gradViolaSctheta);
                                }
                                else{
                                    cost += -0.5 * tmu * tmu / rho;
                                }
                                uneqc_idx++;
                                //to do
                            }
                        }
                    }
                    gdC2c_2d.block<6, 2>(i * 6, 0) += beta0_2d * gradPos.transpose() +
                                                    beta1_2d * gradVel.transpose() +
                                                    beta2_2d * gradAcc.transpose() +
                                                    beta3_2d * gradJerk.transpose();
                    gdC2t_2d += (gradPos.dot(vel) +
                                 gradVel.dot(acc) +
                                 gradAcc.dot(jerk) +
                                 gradJerk.dot(snap)) *
                                    alpha;
                    gdC2c_1d.block<6, 1>(yawPieceId * 6, 0) += beta0_1d * gradYaw +
                                                               beta1_1d * gradDyaw +
                                                               beta2_1d * gradDdyaw +
                                                               beta3_1d * gradDddyaw;

                    gdC2t_1d += -(gradYaw * dyaw +
                                 gradDyaw * ddyaw +
                                 gradDdyaw * dddyaw +
                                 gradDddyaw * ddddyaw) * yawPieceId;
                    gdC2t_2d += (gradYaw * dyaw +
                                 gradDyaw * ddyaw +
                                 gradDdyaw * dddyaw +
                                 gradDddyaw * ddddyaw) * (i+alpha);
                }

                baset +=  posJerkOpt.getDt();
            }
        }        
        void positiveSmoothedL1(const double &x, double &f, double &df)
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
                return;
        }
        void positiveSmoothedL3(const double &x, double &f, double &df){
            df = x * x;
            f = df *x;
            df *= 3.0;
        

            return ;
        }

        //dense 
        void activationSmoothed(const double &x, double &f, double &df){
            double mu = 0.01;
            double mu4_1 = 1.0/(mu*mu*mu*mu);
            if(x<-mu){
                df = 0;
                f = 0;
            }
            else if(x<0){
                double y=x+mu;
                double y2 = y*y;
                df = y2 * (mu-2*x)*mu4_1;
                f = 0.5 * y2 * y * (mu-x) * mu4_1;
            }
            else if(x<mu){
                double y = x-mu;
                double y2 = y*y;
                df = y2*(mu+2*x)*mu4_1;
                f = 0.5*y2*y*(mu-x)*mu4_1+1;
            }
            else{
                df = 0;
                f = 1; 
            }
            return ;
        }

        
        /*
        void calConsGrad2CT(std::vector<Eigen::MatrixXd>& gdEqCons2c, std::vector<double>& gdEqCons2t, std::vector<Eigen::MatrixXd>& gdUnEqCons2c, std::vector<double>& gdUnEqCons2t){

            // output gradT gradC 
            int N = piecenum;
            double yaw, latAcc, lonAcc, vnorm, cur;
            Eigen::Vector3d sigma, dsigma, ddsigma, dddsigma, ddddsigma;
            Eigen::Vector3d GradSigma, GradDsigma, GradDdsigma, GradDddsigma;
            Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
            double s1, s2, s3, s4, s5;
            double step, alpha,eqf;
            double vioModel, vioCur, vioVel, vioLonAcc, vioLatAcc;
            gdEqCons2c.resize(eqc_num);
            gdEqCons2t.resize(eqc_num);
            gdUnEqCons2c.resize(neqc_num);
            gdUnEqCons2t.resize(neqc_num);
            Eigen::MatrixXd gdCc;
            double gdCt;
            gdCc.resize(6 * piecenum, 3);
            int eqc_idx = 0;
            int uneqc_idx = 0;
            for (int i = 0; i < N; ++i)
            {
                const Eigen::Matrix<double, 6, 3> &c = jerkOpt.getCoeffs().block<6, 3>(i * 6, 0);
                step = jerkOpt.getDt() / traj_res; // T_i /k
                s1 = 0.0;
                // innerLoop = traj_res;
                for (int j = 1; j <= traj_res; ++j)
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
                    alpha = 1.0 / traj_res * j;
                    s1 += step;//update s1 for the next iteration
                    sigma = c.transpose() * beta0;
                    dsigma = c.transpose() * beta1;
                    ddsigma = c.transpose() * beta2;
                    dddsigma = c.transpose() * beta3;
                    ddddsigma = c.transpose() * beta4;
                    GradSigma.setZero();
                    GradDsigma.setZero();
                    GradDdsigma.setZero();
                    GradDddsigma.setZero();
                    yaw = sigma[2];
                    lonAcc = ddsigma[0] * cos(yaw) + ddsigma[1] * sin(yaw);
                    latAcc = -ddsigma[0] * sin(yaw) + ddsigma[1] * cos(yaw);
                    vnorm = dsigma.head(2).norm();
                    cur = dsigma[2] / (sqrt(vnorm * vnorm + RESDELTA));

                    double debugpenalty = 0.0;
                    double debuggradt = 0.0;
                    Eigen::Matrix<double, 6, 3> debuggradC;
                    debuggradC.setZero();
                    {
                        eqf = dsigma.head(2).dot(Eigen::Vector2d(sin(yaw),-cos(yaw)));
                        gdCc.setZero();
                        gdCt = 0.0;
                        {
                            //calculate constraint w.r.t coef and t
                            Eigen::Vector2d gradViolaMv;
                            double  gradViolaMtheta; 
                            gradViolaMv = Eigen::Vector2d(sin(yaw),-cos(yaw)); 
                            gradViolaMtheta =  dsigma.head(2).transpose()* Eigen::Vector2d(cos(yaw),sin(yaw));
                            double gradviolaMt = alpha * (gradViolaMv.dot(ddsigma.head(2)) + gradViolaMtheta * dsigma[2]);
                            gdCc.block<6, 2>(i * 6, 0) = beta1 * gradViolaMv.transpose();
                            gdCc.block<6, 1>(i * 6, 2) = beta0 * gradViolaMtheta;
                            gdCt = gradviolaMt;
                        }
                        gdEqCons2c[eqc_idx] = gdCc;
                        gdEqCons2t[eqc_idx] = gdCt;
                        eqc_idx++;
                    }
                    {
                        vioCur = cur*cur - kmax*kmax;
                        gdCc.setZero();
                        gdCt = 0.0;
                        {
                            //calculate constraint w.r.t coef and t
                            double temp = sqrt(vnorm * vnorm + RESDELTA);
                            Eigen::Vector2d gradViolaCv;
                            double  gradViolaCdtheta; 
                            gradViolaCv = (-2) * cur * dsigma[2] * dsigma.head(2) / (temp * temp * temp);
                            gradViolaCdtheta = 2 * cur / (temp);
                            double gradviolaCt = alpha * (gradViolaCv.dot(ddsigma.head(2)) + gradViolaCdtheta * ddsigma[2]);
                            gdCc.block<6, 2>(i * 6, 0) = beta1 * gradViolaCv.transpose();
                            gdCc.block<6, 1>(i * 6, 2) = beta1 * gradViolaCdtheta;
                            gdCt = gradviolaCt;            
                        }
                        gdUnEqCons2c[uneqc_idx] = gdCc;
                        gdUnEqCons2t[uneqc_idx] = gdCt;
                        uneqc_idx++;
                    }
                    {
                        vioVel = dsigma.head(2).transpose() * dsigma.head(2) - vmax * vmax;
                        gdCc.setZero();
                        gdCt = 0.0;
                        {
                            Eigen::Vector2d gradViolaVv;
                            gradViolaVv = 2 * dsigma.head(2);
                            double gradviolaVt = alpha * (gradViolaVv.dot(ddsigma.head(2)));
                            gdCc.block<6, 2>(i * 6, 0) = beta1 * gradViolaVv.transpose();
                            gdCt = gradviolaVt;
                        }
                        gdUnEqCons2c[uneqc_idx] = gdCc;
                        gdUnEqCons2t[uneqc_idx] = gdCt;
                        uneqc_idx++;
                    }
                    {
                        vioLonAcc = lonAcc * lonAcc - lonAccmax * lonAccmax;
                        gdCc.setZero();
                        gdCt = 0.0;
                        {
                            Eigen::Vector2d gradViolaLonAa;
                            double  gradViolaLonAtheta; 
                            gradViolaLonAa = 2 * lonAcc * Eigen::Vector2d(cos(yaw),sin(yaw));
                            gradViolaLonAtheta = 2 * lonAcc * latAcc;
                            double gradviolaLonAt = alpha * (gradViolaLonAa.dot(dddsigma.head(2)) + gradViolaLonAtheta * dsigma[2]);
                            gdCc.block<6, 2>(i * 6, 0) = beta2 * gradViolaLonAa.transpose();
                            gdCc.block<6, 1>(i * 6, 2) = beta0 * gradViolaLonAtheta;
                            gdCt = gradviolaLonAt;            
                        }
                        gdUnEqCons2c[uneqc_idx] = gdCc;
                        gdUnEqCons2t[uneqc_idx] = gdCt;
                        uneqc_idx++;
                    }
                    {
                        vioLatAcc = latAcc * latAcc - latAccmax * latAccmax;
                        gdCc.setZero();
                        gdCt = 0.0;
                        {
                            Eigen::Vector2d gradViolaLatAa;
                            double  gradViolaLatAtheta; 
                            gradViolaLatAa = 2 * latAcc * Eigen::Vector2d(-sin(yaw),cos(yaw));
                            gradViolaLatAtheta = -2 * latAcc * lonAcc;
                            double gradviolaLatAt = alpha * (gradViolaLatAa.dot(dddsigma.head(2)) + gradViolaLatAtheta * dsigma[2]);
                            gdCc.block<6, 2>(i * 6, 0) = beta2 * gradViolaLatAa.transpose();
                            gdCc.block<6, 1>(i * 6, 2) = beta0 * gradViolaLatAtheta;
                            gdCt = gradviolaLatAt; 
                        }
                        gdUnEqCons2c[uneqc_idx] = gdCc;
                        gdUnEqCons2t[uneqc_idx] = gdCt;
                        uneqc_idx++;
                    }
                    {

                    }
                }
            }
        }
        void initScalingW(const Eigen::VectorXd &xi)
        {
            scaling_wf_ = 1.0;
            scaling_eqwc_.resize(eqc_num); scaling_eqwc_.setConstant(1.0);
            scaling_uneqwc_.resize(neqc_num); scaling_eqwc_.setConstant(1.0);
            Eigen::VectorXd gradf_(xi.size()); 
            std::vector<Eigen::VectorXd> gradeqc_, graduneqc_;
            gradeqc_.resize(eqc_num); graduneqc_.resize(neqc_num);
            for(auto& it : gradeqc_){
                it.resize(xi.size());
            }
            for(auto& it : graduneqc_){
                it.resize(xi.size());
            }
            int offset = 0;
            Eigen::Map<const Eigen::MatrixXd> P(xi.data(), 3, piecenum - 1);
            offset += 3 * (piecenum - 1);
            const double t = xi[offset];
            double T;
            VirtualT2RealT(t, T);
            jerkOpt.generate(P,T / piecenum, iniState_, finState_);
            {
                Eigen::MatrixXd gdSc;
                double gdSdt;
                jerkOpt.initSmGradCost(gdSc, gdSdt);
                //VirtualTGrad2t
                Eigen::MatrixXd grads2p;
                jerkOpt.calGrads_PT(gdSc, gdSdt, grads2p); // gdt gdp gdhead gdtail
                double gradsvt;
                gdSdt += wei_time_ * piecenum;
                VirtualTGrad2t(t, gdSdt / piecenum, gradsvt);
                Eigen::Map<Eigen::MatrixXd> grads2p_(gradf_.data(), 3, piecenum - 1);
                grads2p_ = grads2p;
                gradf_[offset] = gradsvt;
            }
            {
                std::vector<Eigen::MatrixXd> gdEqCons2c, gdUnEqCons2c; 
                std::vector<double> gdEqCons2t, gdUnEqCons2t;
                calConsGrad2CT(gdEqCons2c, gdEqCons2t, gdUnEqCons2c, gdUnEqCons2t);
                for(int i = 0; i < eqc_num; i++){
                    Eigen::MatrixXd gdc = gdEqCons2c[i];
                    double gddt = gdEqCons2t[i];
                    Eigen::MatrixXd gradeqc2p;
                    jerkOpt.calGrads_PT(gdc, gddt, gradeqc2p); // gdt gdp gdhead gdtail
                    double gradeqcvt;
                    VirtualTGrad2t(t, gddt / piecenum, gradeqcvt);
                    Eigen::Map<Eigen::MatrixXd> gradeqc2p_(gradeqc_[i].data(), 3, piecenum - 1);
                    gradeqc2p_ = gradeqc2p;
                    gradeqc_[i][offset] = gradeqcvt;
                }
                for(int i = 0; i < neqc_num; i++){
                    Eigen::MatrixXd gdc = gdUnEqCons2c[i];
                    double gddt = gdUnEqCons2t[i];
                    Eigen::MatrixXd gradneqc2p;
                    jerkOpt.calGrads_PT(gdc, gddt, gradneqc2p); // gdt gdp gdhead gdtail
                    double gradneqcvt;
                    VirtualTGrad2t(t, gddt / piecenum, gradneqcvt);
                    Eigen::Map<Eigen::MatrixXd> gradneqc2p_(graduneqc_[i].data(), 3, piecenum - 1);
                    gradneqc2p_ = gradneqc2p;
                    graduneqc_[i][offset] = gradneqcvt;
                }
            }
            scaling_wf_ = std::max(scaling_wf_min_, 1.0 / std::max(1.0, gradf_.cwiseAbs().maxCoeff()));
            for (int i = 0; i < scaling_eqwc_.size(); i++)
            {
                scaling_eqwc_(i) = std::max(scaling_wc_min_, 1.0 / std::max(1.0, gradeqc_[i].cwiseAbs().maxCoeff()));
            }
            for (int i = 0; i < scaling_uneqwc_.size(); i++)
            {
                scaling_uneqwc_(i) = std::max(scaling_wc_min_, 1.0 / std::max(1.0, graduneqc_[i].cwiseAbs().maxCoeff()));
            }

        }
        */
       
        void updateDualVar(){
            /*update lambda*/
            {
                for(int i = 0; i < eqc_num; i++){
                    lambda[i] += rho * scaling_eqwc_[i] * hx[i];
                }
            }
            /*update mu*/
            {
                for(int i = 0; i < neqc_num; i++){
                    if(mu[i] + rho * scaling_uneqwc_[i] * gx[i] >= 0){
                        mu[i] += rho * scaling_uneqwc_[i] * gx[i];
                    }
                    else{
                        mu[i] = 0;
                    }
                }
            }
            /*update rho*/
            {
                rho = std::min((1 + gamma) * rho, rho_max);
            }
        }
        bool ifConvergence(){
            //to do
            Eigen::VectorXd maxgx(neqc_num), maxhx(eqc_num);
            for(int i =0 ;i < eqc_num; i++){
                // maxhx[i] = hx[i] * lambda[i];
                maxhx[i] = hx[i];
            }
            for(int i = 0; i < neqc_num; i++){
                maxgx[i] = std::max(0.0, gx[i]);
            }
            double reshx = maxhx.cwiseAbs().maxCoeff();
            double resgx = maxgx.cwiseAbs().maxCoeff();
            double resgrad = gradx.cwiseAbs().maxCoeff();
            /*hzc may ???*/
            //&& (resgrad) < prec_eps_
            std::cout << "reshx: "<<reshx << " resgx: "<<resgx << " resgrad: "<<resgrad << std::endl;
            if (std::max(reshx, resgx) < cons_eps_ )
            {
                return true;
            }
            return false;
        }
        template <typename EIGENVEC>
        EIGENVEC addAlmGradPro(const double &ws, const double &lbd, const double &cons, const EIGENVEC &grad_){
            EIGENVEC output = ws * lbd * grad_ + rho * ws * ws * cons * grad_;
            return output;
        }
        std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d>> getSe2traj(double dt){
            std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d>> arrowTraj;
            Se2Trajectory traj;
            traj.posTraj = posJerkOpt.getTraj();
            traj.angleTraj = yawJerkOpt.getTraj();
            for(double t = 0.0; t < traj.getTotalDuration()-1.0e-3; t+=dt){
                Eigen::Vector2d pos= traj.getPos(t);
                double angle = traj.getAngle(t);
                Eigen::Vector3d p1,p2;
                p1 << pos[0], pos[1], 0.0;
                p2 << pos[0] + 1.0 * cos(angle), pos[1] + 1.0 * sin(angle), 0.0;
                arrowTraj.push_back(std::pair<Eigen::Vector3d,Eigen::Vector3d>(p1,p2));
            }
            Eigen::Vector2d pos= traj.getPos(traj.getTotalDuration());
            double angle = traj.getAngle(traj.getTotalDuration());
            Eigen::Vector3d p1,p2;
            p1 << pos[0], pos[1], 0.0;
            p2 << pos[0] + 1.0 * cos(angle), pos[1] + 1.0 * sin(angle), 0.0;
            arrowTraj.push_back(std::pair<Eigen::Vector3d,Eigen::Vector3d>(p1,p2));
            return arrowTraj;
        }
        std::vector<Eigen::Vector3d> getTraj(double dt){
            //return {px,py,yaw}
            std::vector<Eigen::Vector3d> poseTraj;
            Se2Trajectory traj;
            traj.posTraj = posJerkOpt.getTraj();
            traj.angleTraj = yawJerkOpt.getTraj();
            for(double t = 0.0; t < traj.getTotalDuration()-1.0e-3; t+=dt){
                Eigen::Vector2d pos= traj.getPos(t);
                double angle = traj.getAngle(t);
                poseTraj.push_back(Eigen::Vector3d(pos[0],pos[1],0));
            }
            Eigen::Vector2d pos= traj.getPos(traj.getTotalDuration());
            double angle = traj.getAngle(traj.getTotalDuration());
            poseTraj.push_back(Eigen::Vector3d(pos[0],pos[1],0));
            return poseTraj;

        }
        Se2Trajectory getTraj(){
            Se2Trajectory traj;
            traj.posTraj = posJerkOpt.getTraj();
            traj.angleTraj = yawJerkOpt.getTraj();
            return traj;
        }
};
} 
#endif 