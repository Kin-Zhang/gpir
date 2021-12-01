/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#pragma once

#include <Eigen/Dense>
#include <deque>

#include "common/base/state.h"
#include "common/base/trajectory.h"
#include "common/utils/boxcar_filter.h"
#include "curvature_experiment/simulator/car_simulator.h"

class MpcController {
 public:
  MpcController();

  void Init();

  void set_wheel_base(const double wheel_base) { ls_ = wheel_base; }

  bool CalculateAckermannDrive(const common::State& state,
                               const common::Trajectory& trajectory,
                               Control* control_cmd);

 protected:
  void Reset(const common::State& state);

  void Discretization(const Eigen::MatrixXd& matrix_a,
                      const Eigen::MatrixXd& matrix_b);

  void RecordControlCommand(const double speed, const double steer);

  void SetStopCommand();

 private:
  const int nx_ = 3;
  const int nu_ = 2;

  double ts_ = 0.02;
  double ls_ = 0.0;
  int horizon_ = 20;
  int predict_steps_ = 0;

  double max_speed_ = 20.0;
  double min_speed_ = 0.0;
  double max_acc_ = 3.0;
  double min_acc_ = -5.0;
  double max_steer_ = 35.0 * M_PI / 180.0;
  double max_srate_ = 35.0 * M_PI / 180.0;

  Eigen::MatrixXd matrix_a_piao_;
  Eigen::MatrixXd matrix_b_piao_;
  Eigen::MatrixXd matrix_ad_;
  Eigen::MatrixXd matrix_bd_;
  Eigen::MatrixXd matrix_q_;
  Eigen::MatrixXd matrix_r_;
  Eigen::MatrixXd matrix_xmax_;
  Eigen::MatrixXd matrix_xmin_;
  Eigen::MatrixXd matrix_xmax_relative_;
  Eigen::MatrixXd matrix_xmin_relative_;
  Eigen::MatrixXd matrix_umax_;
  Eigen::MatrixXd matrix_umin_;

  common::BoxCarFilter bf_x_;
  common::BoxCarFilter bf_y_;
  common::BoxCarFilter bf_heading_;
  common::BoxCarFilter bf_steer_;

  std::vector<double> last_control_;

  std::deque<double> history_vel_;
  std::deque<double> history_steer_;
};
