/* Copyright 2020 Unity-Drive Inc. All rights reserved */
/*     Author: ChengJie (chengjie@unity-drive.com)     */

#include "planning_core/simulation/controller/mmpc_controller.h"

#include <glog/logging.h>
#include <osqp/osqp.h>

#include <algorithm>

#include "common/solver/osqp/osqp_interface.h"
#include "common/solver/osqp/osqp_sparse_matrix.h"
#include "common/utils/math.h"

namespace planning {
namespace simulation {

using Eigen::MatrixXd;

void MpcController::Init() {
  dt_ = 0.05;
  predict_steps_ = 5;
  horizon_ = 20;
  max_steer_angle_ = 50 * M_PI / 180.0;
  max_steer_rate_ = 20 * M_PI / 180.0;
  max_forward_speed_ = 20;
  max_inverse_speed_ = 0;
  max_acceleration_ = 3.0;
  max_deceleration_ = -5.0;

  double weight_position_error = 3.0;
  double weight_position_error_vel = 0;
  double weight_heading_error = 3.0;
  double weight_heading_error_vel = 0;
  double weight_steer = 5.0;
  double weight_steer_vel = 0;
  double weight_steer_rate = 2.0;
  double weight_steer_acc = 0;
  double weight_v = 5.0;
  double weight_acc = 2.0;
  double weight_jerk = 0;

  double weight_terminal_position_error = 3;
  double weight_terminal_heading_error = 3;

  /* normalize weightings */
  double postion_scale_sqr = 1.0;     // 0.5;
  double heading_scale_sqr = 1.0;     // 1.0 / Sqr(M_PI / 2.0);
  double steer_scale_sqr = 1.0;       // 1.0 / Sqr(max_steer_angle_);
  double steer_rate_scale_sqr = 1.0;  // 1.0 / Sqr(max_steer_rate_);
  double v_scale_sqr = 1.0;           // 1.0 / Sqr(max_forward_speed_);
  double a_scale_sqr = 1.0;           // 1.0 / Sqr(max_acceleration_);

  weight_position_error_ = weight_position_error * postion_scale_sqr;
  weight_position_error_vel_ =
      weight_position_error_vel * postion_scale_sqr * v_scale_sqr;
  weight_heading_error_ = weight_heading_error * heading_scale_sqr;
  weight_heading_error_vel_ =
      weight_heading_error_vel * heading_scale_sqr * v_scale_sqr;
  weight_steer_ = weight_steer * steer_scale_sqr;
  weight_steer_vel_ = weight_steer_vel * steer_scale_sqr * v_scale_sqr;
  weight_steer_rate_ = weight_steer_rate * steer_rate_scale_sqr;
  weight_steer_acc_ = weight_steer_acc;
  weight_v_ = weight_v * v_scale_sqr;
  weight_acc_ = weight_acc * a_scale_sqr;
  weight_jerk_ = weight_jerk;
  weight_terminal_position_error_ =
      weight_terminal_position_error * postion_scale_sqr;
  weight_terminal_heading_error_ =
      weight_terminal_heading_error * heading_scale_sqr;

  LOG(INFO) << "nx: " << nx_;
  LOG(INFO) << "nu: " << nu_;
  LOG(INFO) << "weight_position_error_: " << weight_position_error_;
  LOG(INFO) << "weight_position_error_vel_: " << weight_position_error_vel_;
  LOG(INFO) << "weight_heading_error_: " << weight_heading_error_;
  LOG(INFO) << "weight_heading_error_vel_: " << weight_heading_error_vel_;
  LOG(INFO) << "weight_steer_: " << weight_steer_;
  LOG(INFO) << "weight_steer_vel_: " << weight_steer_vel_;
  LOG(INFO) << "weight_steer_rate_: " << weight_steer_rate_;
  LOG(INFO) << "weight_steer_acc_: " << weight_steer_acc_;
  LOG(INFO) << "weight_v_: " << weight_v_;
  LOG(INFO) << "weight_acc_: " << weight_acc_;
  LOG(INFO) << "weight_jerk_: " << weight_jerk_;
  LOG(INFO) << "weight_terminal_position_error_: "
            << weight_terminal_position_error_;
  LOG(INFO) << "weight_terminal_heading_error_: "
            << weight_terminal_heading_error_;

  x_min_ = x_max_ = MatrixXd::Zero(nx_, 1);
  u_min_ = u_max_ = MatrixXd::Zero(nu_, 1);

  x_max_ << OSQP_INFTY, OSQP_INFTY, OSQP_INFTY, max_steer_angle_,
      max_forward_speed_;
  x_min_ << -OSQP_INFTY, -OSQP_INFTY, -OSQP_INFTY, -max_steer_angle_,
      max_inverse_speed_;
  u_max_ << max_steer_rate_, max_acceleration_;
  u_min_ << -max_steer_rate_, max_deceleration_;

  history_vel_.resize(predict_steps_, 0.0);
  history_steer_.resize(predict_steps_, 0.0);
}

bool MpcController::CalculateAckermannDrive(
    const common::State& state, const common::Trajectory& trajectory,
    ackermann_msgs::AckermannDrive* control_cmd) {
  if (trajectory.empty()) {
    LOG_EVERY_N(WARNING, 20) << "trajectory is empty";
    Reset(state);
    return false;
  }

  Eigen::Vector2d predict_pos = state.position;
  double predict_heading = state.heading;
  double predict_v = state.velocity;
  double predict_steer = state.steer;

  for (int i = 0; i < predict_steps_; ++i) {
    predict_v = history_vel_[i];
    predict_steer = history_steer_[i];
    predict_pos.x() += predict_v * std::cos(predict_heading) * dt_;
    predict_pos.y() += predict_v * std::sin(predict_heading) * dt_;
    predict_heading += predict_v / wheel_base_ * std::tan(predict_steer) * dt_;
  }
  auto ref_index = trajectory.GetNearsetIndex(predict_pos);
  if (ref_index == trajectory.size()) {
    LOG_EVERY_N(WARNING, 20) << "reached the end of trajectory";
    Reset(state);
    return false;
  }
  auto ref_state = trajectory[ref_index];
  ref_state.steer = std::atan(wheel_base_ * ref_state.kappa);
  LOG(INFO) << "wheel_base: " << wheel_base_;
  LOG(INFO) << "ref_steer: " << ref_state.steer;

  std::cout << state.DebugString() << std::endl;
  std::cout << ref_state.DebugString() << std::endl;
  MatrixXd x0 = ComputeInitialState(state, ref_state);

  std::cout << x0 << std::endl;

  x_max_relative_ = x_max_;
  x_min_relative_ = x_min_;

  x_max_relative_(3, 0) -= ref_state.steer;
  x_max_relative_(4, 0) -= ref_state.velocity;
  x_min_relative_(3, 0) -= ref_state.steer;
  x_min_relative_(4, 0) -= ref_state.velocity;

  // Eigen::VectorXd result;
  std::vector<double> result;
  if (!SolveMPCproblem(x0, ref_state, &result)) {
    SetStopCommand(control_cmd);
    Reset(state);
    LOG(ERROR) << "Fail to compute control commad";
    return false;
  }

  previous_steer_velocity_ = result[nx_ * horizon_];
  previous_acceleration_ = result[nx_ * horizon_ + 1];
  previous_steer_ += previous_steer_velocity_ * dt_;
  previous_speed_ += previous_acceleration_ * dt_;

  control_cmd->acceleration = 0.0;
  control_cmd->speed = previous_speed_;
  control_cmd->jerk = 0.0;
  control_cmd->steering_angle = previous_steer_;
  control_cmd->steering_angle_velocity = previous_steer_velocity_;
  LOG(INFO) << "speed: " << previous_speed_ << ", steer: " << previous_steer_;

  RecordControlCommand(previous_speed_, previous_steer_);
  return true;
}

MatrixXd MpcController::ComputeInitialState(const common::State& state,
                                            const common::State& ref_point) {
  MatrixXd x0 = MatrixXd::Zero(nx_, 1);

  x0(0, 0) = state.position.x() - ref_point.position.x();
  x0(1, 0) = state.position.y() - ref_point.position.y();
  x0(2, 0) = common::NormalizeAngle(state.heading - ref_point.heading);
  x0(3, 0) = previous_steer_ - ref_point.steer;
  x0(4, 0) = previous_speed_ - ref_point.velocity;

  return x0;
}

// transform to osqp description :
// min  1/2*xTPx + qTx
// wrt.  l <= Ax <= u
// reference: https://osqp.org/docs/solver/index.html
bool MpcController::SolveMPCproblem(const Eigen::MatrixXd& x0,
                                    const common::State& ref_point,
                                    std::vector<double>* result) {
  const double ts = dt_;
  /* assume N >= 2 */
  const int N = horizon_ - 1;

  int nX = nx_ * (N + 1);
  int nU = nu_ * N;
  nv_ = nU + nX;
  int constraints_num = N * nx_ + nv_;

  P_ = MatrixXd::Zero(nv_, nv_);
  q_ = MatrixXd::Zero(nv_, 1);
  A_ = MatrixXd::Zero(constraints_num, nv_);
  l_ = MatrixXd::Zero(constraints_num, 1);
  u_ = MatrixXd::Zero(constraints_num, 1);

  MatrixXd matrix_px = MatrixXd::Zero(nX, nX);
  MatrixXd matrix_pu = MatrixXd::Zero(nU, nU);

  MatrixXd matrix_qx = MatrixXd::Zero(nX, 1);
  MatrixXd matrix_qu = MatrixXd::Zero(nU, 1);

  MatrixXd matrix_a_dynamics = MatrixXd::Zero(nx_ * N, nv_);
  MatrixXd matrix_a_bounding = MatrixXd::Zero(nv_, nv_);

  MatrixXd l_dynamics = MatrixXd::Zero(nx_ * N, 1);
  MatrixXd l_bounding = MatrixXd::Zero(nv_, 1);

  MatrixXd u_dynamics = MatrixXd::Zero(nx_ * N, 1);
  MatrixXd u_bounding = MatrixXd::Zero(nv_, 1);

  MatrixXd matrix_ad, matrix_bd, matrix_wd;
  ComputeModel(&matrix_ad, &matrix_bd, &matrix_wd, ref_point, ts);
  /* step1: compute time receding model, Q and R1*/
  for (int i = 0; i < N; ++i) {
    Eigen::MatrixXd matrix_p = MatrixXd::Zero(nx_, nx_);
    Eigen::MatrixXd matrix_q = MatrixXd::Zero(nx_, 1);
    Eigen::MatrixXd matrix_r = MatrixXd::Zero(nu_, nu_);

    matrix_p(0, 0) = matrix_p(1, 1) = weight_position_error_;
    matrix_p(2, 2) = weight_heading_error_;
    matrix_p(3, 3) = weight_steer_;
    matrix_p(4, 4) = weight_v_;

    matrix_px.block(i * nx_, i * nx_, nx_, nx_) = matrix_p;
    matrix_pu.block(i * nu_, i * nu_, nu_, nu_) = matrix_r;
  }

  /* terminal weight to improve stability */
  matrix_px(nX - 5, nX - 5) = weight_terminal_position_error_;
  matrix_px(nX - 4, nX - 4) = weight_terminal_position_error_;
  matrix_px(nX - 3, nX - 3) = weight_terminal_heading_error_;
  matrix_px(nX - 2, nX - 2) = weight_steer_;
  matrix_px(nX - 1, nX - 1) = weight_v_;

  /* step3: vehicle dynamics constraints */
  const Eigen::MatrixXd I_x = MatrixXd::Identity(nx_, nx_);
  const Eigen::MatrixXd I_u = MatrixXd::Identity(nu_, nu_);
  for (int32_t i = 0; i < N; ++i) {
    matrix_a_dynamics.block(i * nx_, i * nx_, nx_, nx_) = -matrix_ad;
    matrix_a_dynamics.block(i * nx_, (i + 1) * nx_, nx_, nx_) = I_x;
    matrix_a_dynamics.block(i * nx_, nX + i * nu_, nx_, nu_) = -matrix_bd;
    l_dynamics.block(i * nx_, 0, nx_, 1) = matrix_wd;
    u_dynamics.block(i * nx_, 0, nx_, 1) = matrix_wd;
  }

  /* step4: state, control constraints */
  const double x_offset = N * nx_;
  const double u_offset = N * nu_;
  for (int i = 0; i < N + 1; ++i) {
    matrix_a_bounding.block(i * nx_, i * nx_, nx_, nx_) = I_x;
    if (i == 0) {
      u_bounding.block(i * nx_, 0, nx_, 1) = x0;
      l_bounding.block(i * nx_, 0, nx_, 1) = x0;
    } else {
      u_bounding.block(i * nx_, 0, nx_, 1) = x_max_relative_;
      l_bounding.block(i * nx_, 0, nx_, 1) = x_min_relative_;
    }
    if (i != N) {
      matrix_a_bounding.block(nX + i * nu_, nX + i * nu_, nu_, nu_) = I_u;
      u_bounding.block(nX + i * nu_, 0, nu_, 1) = u_max_;
      l_bounding.block(nX + i * nu_, 0, nu_, 1) = u_min_;
    }
  }

  /* step5: compute P, q, A, l, u */
  P_.block(0, 0, nX, nX) = 2 * matrix_px;
  P_.block(nX, nX, nU, nU) = 2 * matrix_pu;

  q_.block(0, 0, nX, 1) = matrix_qx;
  q_.block(nX, 0, nU, 1) = matrix_qu;

  A_.block(0, 0, N * nx_, nv_) = matrix_a_dynamics;
  A_.block(N * nx_, 0, nv_, nv_) = matrix_a_bounding;

  l_.block(0, 0, N * nx_, 1) = l_dynamics;
  l_.block(N * nx_, 0, nv_, 1) = l_bounding;

  u_.block(0, 0, N * nx_, 1) = u_dynamics;
  u_.block(N * nx_, 0, nv_, 1) = u_bounding;

  // return common::OsqpInterface::Solve(P_, q_, A_, l_, u_, result);
  return SolveOsqpProblem(result);
}

bool MpcController::SolveOsqpProblem(std::vector<double>* result) {
  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  const Eigen::MatrixXd& P = P_.triangularView<Eigen::Upper>();
  common::DenseToCSCMatrix(P, &P_data, &P_indices, &P_indptr);

  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
  common::DenseToCSCMatrix(A_, &A_data, &A_indices, &A_indptr);

  // Problem settings
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));

  data->n = P_.rows();
  data->m = A_.rows();
  data->P = csc_matrix(data->n, data->n, P_data.size(), P_data.data(),
                       P_indices.data(), P_indptr.data());
  data->q = q_.data();
  data->A = csc_matrix(data->m, data->n, A_data.size(), A_data.data(),
                       A_indices.data(), A_indptr.data());
  data->l = l_.data();
  data->u = u_.data();

  // Setup settings
  osqp_set_default_settings(settings);
  settings->alpha = 1.0;
  settings->eps_abs = 1.0e-05;
  settings->eps_rel = 1.0e-05;
  settings->max_iter = 5000;
  settings->polish = true;
  settings->verbose = false;

  // Setup workspace
  OSQPWorkspace* work;
  c_int exitflag = osqp_setup(&work, data, settings);

  // Solve
  bool problem_solved = true;
  osqp_solve(work);
  auto status = work->info->status_val;

  if (status < 0 || (status != 1 && status != 2)) {
    LOG(ERROR) << "failed optimization status:\t" << work->info->status;
    problem_solved = false;
  } else if (work->solution == nullptr) {
    LOG(ERROR) << "The solution from OSQP is nullptr";
    problem_solved = false;
  }

  if (problem_solved) {
    for (int i = 0; i < nv_; ++i) {
      result->emplace_back(work->solution->x[i]);
      // LOG(INFO) << work->solution->x[i];
    }
  } else {
    result = nullptr;
  }

  // Cleanup
  osqp_cleanup(work);
  c_free(data->A);
  c_free(data->P);
  c_free(data);
  c_free(settings);

  return problem_solved;
}

void MpcController::ComputeModel(MatrixXd* matrix_ad, MatrixXd* matrix_bd,
                                 MatrixXd* matrix_wd,
                                 const common::State& ref_point,
                                 const double dt) {
  Eigen::MatrixXd matrix_a = MatrixXd::Zero(nx_, nx_);
  Eigen::MatrixXd matrix_b = MatrixXd::Zero(nx_, nu_);
  Eigen::MatrixXd matrix_w = MatrixXd::Zero(nx_, 1);

  const double v_ref = ref_point.velocity;
  const double theta_ref = ref_point.heading;
  LOG(INFO) << "Heading: " << ref_point.heading;
  const double sin0_ref = std::sin(theta_ref);
  const double cos0_ref = std::cos(theta_ref);
  const double cos2_s_ref = std::pow(std::cos(ref_point.steer), 2);

  matrix_a(0, 2) = -v_ref * sin0_ref;
  matrix_a(0, 4) = cos0_ref;
  matrix_a(1, 2) = v_ref * cos0_ref;
  matrix_a(1, 4) = sin0_ref;
  matrix_a(2, 3) = v_ref / wheel_base_ / cos2_s_ref;
  matrix_a(2, 4) = std::tan(ref_point.steer) / wheel_base_;

  matrix_b(3, 0) = 1.0;
  matrix_b(4, 1) = 1.0;

  MatrixXd I = MatrixXd::Identity(nx_, nx_);
  (*matrix_ad) =
      (I - dt * matrix_a / 2.0).inverse() * (I + dt * matrix_a / 2.0);

  (*matrix_bd) = dt * matrix_b;
  (*matrix_wd) = dt * matrix_w;
}

void MpcController::RecordControlCommand(const double speed,
                                         const double steer) {
  history_vel_.push_back(speed);
  history_steer_.push_back(steer);
  if (history_vel_.size() > predict_steps_) {
    history_vel_.pop_front();
    history_steer_.pop_front();
  }
}

void MpcController::Reset(const common::State& state) {
  previous_speed_ = state.velocity;
  previous_steer_ = state.steer;
  RecordControlCommand(0.0, 0.0);
}

void MpcController::SetStopCommand(
    ackermann_msgs::AckermannDrive* control_cmd) {
  control_cmd->acceleration = 0.0;
  control_cmd->jerk = 0.0;
  control_cmd->speed = 0.0;
  control_cmd->steering_angle = 0.0;
  control_cmd->steering_angle_velocity = 0.0;
}
}  // namespace simulation
}  // namespace planning
