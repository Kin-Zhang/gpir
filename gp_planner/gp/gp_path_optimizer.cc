/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include "gp_planner/gp/gp_path_optimizer.h"

#include <glog/logging.h>

#include "common/smoothing/osqp_spline1d_solver.h"
#include "common/smoothing/osqp_spline2d_solver.h"
#include "gp_planner/gp/factors/gp_interpolate_kappa_limit_factor.h"
#include "gp_planner/gp/factors/gp_interpolate_obstacle_factor.h"
#include "gp_planner/gp/factors/gp_kappa_limit_factor.h"
#include "gp_planner/gp/factors/gp_obstacle_factor.h"
#include "gp_planner/gp/factors/gp_prior_factor.h"
#include "gp_planner/initializer/gp_initializer.h"
#include "gp_planner/thirdparty/traj_min_jerk/traj_min_jerk.hpp"
#include "gtsam/inference/Symbol.h"
#include "gtsam/nonlinear/DoglegOptimizer.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/LevenbergMarquardtParams.h"

namespace planning {

using gtsam::Vector3;
using PriorFactor3 = gtsam::PriorFactor<Vector3>;

constexpr double kEpsilon = 1.8;
constexpr double kQc = 0.1;

bool GPPathOptimizer::GenerateGPPath(const ReferenceLine& reference_line,
                                     const common::FrenetState& frenet_state,
                                     const double length, const double s,
                                     common::Trajectory* trajectory,
                                     GPPath* gp_path) {
  static auto fix_pos_cost = gtsam::noiseModel::Isotropic::Sigma(3, 0.001);
  auto pose_fix_cost2 =
      gtsam::noiseModel::Diagonal::Sigmas(Vector3(1, 0.1, 0.1));
  auto pose_fix_cost3 =
      gtsam::noiseModel::Diagonal::Sigmas(Vector3(20, 1e9, 1e9));
  const double delta_s = length / (num_of_nodes_ - 1);

  s_refs_.clear();
  if (!graph_.empty()) graph_.resize(0);
  // * init state
  // common::FrenetState frenet_state;
  // reference_line.ToFrenetState(state, &frenet_state);
  // LOG(INFO) << frenet_state.DebugString();
  // LOG(INFO) << state.DebugString();
  // frenet_state.d[0] = state.debug(0);
  // frenet_state.d[1] = state.debug(1);
  // frenet_state.d[2] = state.debug(2);
  // printf("previous: (%f, %f, %f), now (%f, %f, %f)\n", state.debug(0),
  //        state.debug(1), state.debug(2), frenet_state.d[0], frenet_state.d[1],
  //        frenet_state.d[2]);

  Vector3 x0(frenet_state.d[0], frenet_state.d[1], frenet_state.d[2]);
  Vector3 xn(0, 0, 0);
  Vector3 x_ref(0, 0, 0);

  // Eigen::Matrix3d ref_cost_max;
  // ref_cost_max << 5, 0, 0, 0, 0, 0, 0, 0, 0;

  double start_s = frenet_state.s[0];

  const int collision_check_num = 10;
  const double tau = delta_s / (collision_check_num + 1);

  graph_.reserve(num_of_nodes_);
  double kappa_r = 0.0, dkappa_r = 0.0, current_s = 0.0;
  for (int i = 0; i < num_of_nodes_; ++i) {
    current_s = start_s + i * delta_s;
    gtsam::Key key = gtsam::Symbol('x', i);

    if (i == 0) graph_.add(PriorFactor3(key, x0, fix_pos_cost));
    if (i == num_of_nodes_ - 1) graph_.add(PriorFactor3(key, xn, fix_pos_cost));

    if (i > 0) {
      gtsam::Key last_key = gtsam::Symbol('x', i - 1);
      reference_line.GetCurvature(current_s, &kappa_r, &dkappa_r);

      graph_.add(GPPriorFactor(last_key, key, delta_s, kQc));

      if (std::fabs(current_s - s) > 6) {
        graph_.add(PriorFactor3(key, x_ref, pose_fix_cost3));
      }
      graph_.add(
          GPObstacleFactor(key, sdf_, 0.01, kEpsilon, current_s, kappa_r));
      graph_.add(
          GPKappaLimitFactor(key, 0.1, kappa_r, dkappa_r, 0.35, current_s));

      for (int j = 0; j < collision_check_num; ++j) {
        graph_.add(GPInterpolateObstacleFactor(
            last_key, key, sdf_, 0.01, kEpsilon, start_s + delta_s * (i - 1),
            kQc, delta_s, tau * (j + 1), kappa_r));
        graph_.add(GPInterpolateKappaLimitFactor(last_key, key, 0.1, kQc,
                                                 delta_s, tau * (j + 1),
                                                 kappa_r, 0.0, 0.35));
        // s_refs_.emplace_back(start_s + delta_s * (i - 1) + tau * (j + 1));
      }
    }
    s_refs_.emplace_back(current_s);
  }

  std::vector<std::vector<std::pair<double, double>>> boundaries;
  std::vector<double> seeds;
  seeds.emplace_back(s);
  sdf_->mutable_occupancy_map()->SearchForVerticalBoundaries(seeds, &boundaries);
  // sdf_->mutable_occupancy_map()->SearchForVerticalBoundaries(s_refs_,
  //                                                           &boundaries);
  gtsam::Values init_values;
  std::vector<Eigen::Vector3d> resa, tmp;
  std::vector<double> l;
  GPInitializer initializer;
  // initializer.SetBoundary(boundaries);
  // if (!initializer.Solve(std::array<double, 3>{x0(0), x0(1), x0(2)},
  //                        std::array<double, 3>{xn(0), xn(1), xn(2)}, delta_s,
  //                        &resa, &l)) {
  //   LOG(ERROR) << "fail to generate gp initial path";
  //   return false;
  // }

  if (!initializer.Solve2(x0, xn, s_refs_, s, boundaries[0].front().first,
                          boundaries[0].front().second, &resa, &l)) {
    LOG(ERROR) << "FAILE";
    return false;
  }

  for (int i = 0; i < s_refs_.size(); ++i) {
    gtsam::Key key = gtsam::symbol('x', i);
    init_values.insert<gtsam::Vector3>(key, resa[i]);
  }

  // gtsam::DoglegParams param;
  // param.setDeltaInitial(10.0);
  gtsam::LevenbergMarquardtParams param;
  param.setlambdaInitial(100.0);
  param.setAbsoluteErrorTol(1e-5);
  // param.setVerbosity("ERROR");

  // gtsam::DoglegOptimizer opt(graph_, init_values, param);
  gtsam::LevenbergMarquardtOptimizer opt(graph_, init_values, param);
  auto res = opt.optimize();
  std::vector<double> l_opt;
  for (int i = 0; i < s_refs_.size(); ++i) {
    auto x = res.at<Vector3>(gtsam::Symbol('x', i));
    l_opt.emplace_back(x(0));
  }
  // reference_line.FrenetToCartesion(s_refs_, l_opt, path);

  vector_Eigen<Eigen::Vector2d> init_line;
  for (int i = 0; i < s_refs_.size(); ++i) {
    init_line.emplace_back(Eigen::Vector2d(s_refs_[i], l[i]));
  }
  // sdf_->mutable_occupancy_map()->PolyLine(init_line);

  int inter_num = 20;
  double delta_tau = delta_s / (inter_num + 1);

  *gp_path =
      GPPath(num_of_nodes_, start_s, delta_s, length, kQc, &reference_line);
  auto gp_path_nodes = gp_path->mutable_nodes();
  common::State traj_state;
  std::vector<double> t, x, v, a;
  LOG(INFO) << "res size: " << res.size();
  for (size_t i = 0; i < res.size(); ++i) {
    auto x1 = res.at<gtsam::Vector3>(gtsam::Symbol('x', i));
    // auto x2 = res.at<gtsam::Vector3>(gtsam::Symbol('x', i + 1));
    // t.emplace_back(i * delta_s + start_s);
    // x.emplace_back(x1(0));
    // v.emplace_back(x1(1));
    // a.emplace_back(x1(2));
    // reference_line.FrenetToState(t.back(), x1, &traj_state);
    // traj_state.debug = x1;
    // trajectory->emplace_back(traj_state);

    // for (int j = 0; j < inter_num; ++j) {
    //   double tau = (j + 1) * delta_tau;
    //   auto inter_x = GPInterpolator::Interpolate(x1, x2, kQc, delta_s, tau);
    //   t.emplace_back(i * delta_s + tau + start_s);
    //   x.emplace_back(inter_x(0));
    //   v.emplace_back(inter_x(1));
    //   a.emplace_back(inter_x(2));
    //   reference_line.FrenetToState(t.back(), inter_x, &traj_state);
    //   traj_state.debug = inter_x;
    //   trajectory->emplace_back(traj_state);
    // }
    gp_path_nodes->emplace_back(x1);
  }

  // for (double i = start_s; i <= start_s + length; i += 1.0) {
  //   gp_path->GetState(i, &traj_state);
  //   trajectory->emplace_back(traj_state);
  // }

  // vector_Eigen<Eigen::Vector2d> points;
  // for (int i = 0; i < x.size(); ++i) {
  //   points.emplace_back(Eigen::Vector2d(t[i], x[i]));
  // }

  // sdf_->mutable_occupancy_map()->PolyLine(points);
  // cv::imshow("path", sdf_->occupancy_map().BinaryImage());
  // cv::waitKey(50);
  return true;
}
}  // namespace planning
