#ifndef COMMON_H_
#define COMMON_H_

#include <assert.h>
#include <Eigen/Eigen>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
namespace common {

class VehicleParam {
 public:
  inline double width() const { return width_; }
  inline double length() const { return length_; }
  inline double wheel_base() const { return wheel_base_; }
  inline double front_suspension() const { return front_suspension_; }
  inline double rear_suspension() const { return rear_suspension_; }
  inline double max_steering_angle() const { return max_steering_angle_; }
  inline double max_longitudinal_acc() const { return max_longitudinal_acc_; }
  inline double max_lateral_acc() const { return max_lateral_acc_; }
  inline double d_cr() const { return d_cr_; }

  inline void set_width(const double val) { width_ = val; }
  inline void set_length(const double val) { length_ = val; }
  inline void set_wheel_base(const double val) { wheel_base_ = val; }
  inline void set_front_suspension(const double val) {
    front_suspension_ = val;
  }
  inline void set_rear_suspension(const double val) { rear_suspension_ = val; }
  inline void set_max_steering_angle(const double val) {
    max_steering_angle_ = val;
  }
  inline void set_max_longitudinal_acc(const double val) {
    max_longitudinal_acc_ = val;
  }
  inline void set_max_lateral_acc(const double val) { max_lateral_acc_ = val; }
  inline void set_d_cr(const double val) { d_cr_ = val; }

  /**
   * @brief Print info
   */
  void print() const;

 private:
  double width_ = 0.5;
  double length_ = 0.8;
  double wheel_base_ = 0.6;
  double d_cr_ = 0.3;  // length between geometry center and rear axle



  double front_suspension_ = 0.93;
  double rear_suspension_ = 1.10;
  double max_steering_angle_ = 45.0;

  double max_longitudinal_acc_ = 2.0;
  double max_lateral_acc_ = 2.0;

};
struct State {
  double time_stamp{0.0};
  Eigen::Vector2d vec_position{Eigen::Vector2d::Zero()};
  double angle{0.0};  // heading angle
  double curvature{0.0};
  double velocity{0.0};
  double acceleration{0.0};
  double steer{0.0};  // steering angle
  void print() const {
    printf("State:\n");
    printf(" -- time_stamp: %lf.\n", time_stamp);
    printf(" -- vec_position: (%lf, %lf).\n", vec_position[0], vec_position[1]);
    printf(" -- angle: %lf.\n", angle);
    printf(" -- curvature: %lf.\n", curvature);
    printf(" -- velocity: %lf.\n", velocity);
    printf(" -- acceleration: %lf.\n", acceleration);
    printf(" -- steer: %lf.\n", steer);
  }

  Eigen::Vector3d ToXYTheta() const {
    return Eigen::Vector3d(vec_position(0), vec_position(1), angle);
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}


#endif