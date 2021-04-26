/* Copyright 2019 Unity-Drive Inc. All rights reserved */

#pragma once

#include <ackermann_msgs/AckermannDrive.h>

#include <Eigen/Dense>
#include <deque>

#include "common/base/trajectory.h"

namespace planning {
namespace simulation {

class MpcController {
 public:
  /**
   * @brief constructor
   */
  MpcController() = default;

  /**
   * @brief destructor
   */
  ~MpcController() = default;

  void Init();

  void set_wheel_base(const double wheel_base) { wheel_base_ = wheel_base; }

  bool CalculateAckermannDrive(const common::State& state,
                               const common::Trajectory& trajectory,
                               ackermann_msgs::AckermannDrive* control_cmd);
  /**
   * @brief reset controller
   */
  void Reset(const common::State& state);

 private:
  Eigen::MatrixXd ComputeInitialState(const common::State& ego,
                                      const common::State& ref);

  void ComputeModel(Eigen::MatrixXd* matrix_ad, Eigen::MatrixXd* matrix_bd,
                    Eigen::MatrixXd* matrix_wd, const common::State& ref_point,
                    const double dt);

  bool SolveMPCproblem(const Eigen::MatrixXd& x0,
                       const common::State& ref_points,
                       std::vector<double>* result);

  bool SolveOsqpProblem(std::vector<double>* result);

  void SetStopCommand(ackermann_msgs::AckermannDrive* control_cmd);

  void RecordControlCommand(const double speed, const double steer);

 private:
  /* controller params */
  double dt_ = 0;
  int32_t predict_steps_ = 0;
  int32_t horizon_ = 0;

  double weight_position_error_ = 0.0;
  double weight_position_error_vel_ = 0.0;

  double weight_heading_error_ = 0.0;
  double weight_heading_error_vel_ = 0.0;

  double weight_steer_ = 0.0;
  double weight_steer_vel_ = 0.0;
  double weight_steer_rate_ = 0.0;
  double weight_steer_acc_ = 0.0;

  double weight_v_ = 0.0;
  double weight_acc_ = 0.0;
  double weight_jerk_ = 0.0;

  double weight_terminal_position_error_ = 0.0;
  double weight_terminal_heading_error_ = 0.0;

  /* vehicle params */
  double wheel_base_ = 0.0;
  double speedctl_deadzone_ = 0.0;

  double max_steer_angle_ = 0.0;
  double max_steer_rate_ = 0.0;

  double max_forward_speed_ = 0.0;
  double max_inverse_speed_ = 0.0;

  double max_acceleration_ = 0.0;
  double max_deceleration_ = 0.0;

  /* qp problem description */
  const int nx_ = 5;
  const int nu_ = 2;

  int nv_ = 0;

  Eigen::MatrixXd x_min_;
  Eigen::MatrixXd x_max_;

  Eigen::MatrixXd x_min_relative_;
  Eigen::MatrixXd x_max_relative_;

  Eigen::MatrixXd u_min_;
  Eigen::MatrixXd u_max_;

  Eigen::MatrixXd P_;
  Eigen::VectorXd q_;
  Eigen::MatrixXd A_;

  Eigen::VectorXd l_;
  Eigen::VectorXd u_;

  double previous_steer_ = 0.0;
  double previous_speed_ = 0.0;
  double previous_steer_velocity_ = 0.0;
  double previous_acceleration_ = 0.0;

  std::deque<double> history_vel_;
  std::deque<double> history_steer_;
};
}  // namespace simulation
}  // namespace planning
