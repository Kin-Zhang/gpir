/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include "gp_planner/gp/gp_path_optimizer.h"

#include <glog/logging.h>

#include <queue>

#include "common/smoothing/osqp_spline1d_solver.h"
#include "common/smoothing/osqp_spline2d_solver.h"
#include "common/utils/math.h"
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
using common::NormalizeAngle;

constexpr double kEpsilon = 1.8;
constexpr double kQc = 0.1;

bool GPPathOptimizer::GenerateGPPath(
    const ReferenceLine& reference_line,
    const common::FrenetState& initial_state, const double length,
    const std::vector<double>& obstacle_location_hint, GPPath* gp_path) {
  static auto fix_pos_cost = gtsam::noiseModel::Isotropic::Sigma(3, 0.001);
  auto pose_fix_cost2 =
      gtsam::noiseModel::Diagonal::Sigmas(Vector3(1, 0.1, 0.1));
  auto pose_fix_cost3 =
      gtsam::noiseModel::Diagonal::Sigmas(Vector3(30, 1e9, 1e9));
  const double delta_s = length / (num_of_nodes_ - 1);

  s_refs_.clear();
  if (!graph_.empty()) graph_.resize(0);

  Vector3 x0 = initial_state.d;
  Vector3 xn(0, 0, 0);
  Vector3 x_ref(0, 0, 0);

  double start_s = initial_state.s[0];

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

      // LOG(INFO) << "init v: " << initial_state.s[1];
      if (current_s > initial_state.s[1] * 3) {
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
                                                 kappa_r, dkappa_r, 0.35));
      }
    }
    s_refs_.emplace_back(current_s);
  }

  double init_kappa = reference_line.GetCurvature(initial_state.s[0]);
  double init_angle =
      std::atan2(initial_state.d[1], 1 - init_kappa * initial_state.d[0]);
  std::vector<std::vector<std::pair<double, double>>> boundaries;
  sdf_->mutable_occupancy_map()->SearchForVerticalBoundaries(
      obstacle_location_hint, &boundaries);
  std::vector<double> lb, ub;
  DecideInitialPathBoundary(
      Eigen::Vector2d(initial_state.s[0], initial_state.d[0]), init_angle,
      obstacle_location_hint, boundaries, &lb, &ub);

  GPInitializer initializer;
  vector_Eigen3d initial_path;

  if (!initializer.GenerateInitialPath(x0, xn, s_refs_, obstacle_location_hint,
                                       lb, ub, &initial_path)) {
    LOG(ERROR) << "Generate initial path for GP planner failed";
    return false;
  }

  gtsam::Values init_values;
  for (int i = 0; i < s_refs_.size(); ++i) {
    gtsam::Key key = gtsam::symbol('x', i);
    init_values.insert<gtsam::Vector3>(key, initial_path[i]);
  }

  gtsam::LevenbergMarquardtParams param;
  param.setlambdaInitial(100.0);
  param.setAbsoluteErrorTol(1e-5);
  // param.setVerbosity("ERROR");

  gtsam::LevenbergMarquardtOptimizer opt(graph_, init_values, param);
  auto gp_result = opt.optimize();
  // std::vector<double> l_opt;
  // for (int i = 0; i < s_refs_.size(); ++i) {
  //   auto x = res.at<Vector3>(gtsam::Symbol('x', i));
  //   l_opt.emplace_back(x(0));
  // }

  // vector_Eigen<Eigen::Vector2d> init_line;
  // for (int i = 0; i < s_refs_.size(); ++i) {
  //   init_line.emplace_back(Eigen::Vector2d(s_refs_[i], l[i]));
  // }

  // int inter_num = 20;
  // double delta_tau = delta_s / (inter_num + 1);

  *gp_path =
      GPPath(num_of_nodes_, start_s, delta_s, length, kQc, &reference_line);
  auto gp_path_nodes = gp_path->mutable_nodes();
  // common::State traj_state;
  // std::vector<double> t, x, v, a;
  for (size_t i = 0; i < gp_result.size(); ++i) {
    auto x1 = gp_result.at<gtsam::Vector3>(gtsam::Symbol('x', i));
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

bool GPPathOptimizer::DecideInitialPathBoundary(
    const Eigen::Vector2d& init_pos, const double init_angle,
    const std::vector<double>& obstacle_location_hint,
    const std::vector<std::vector<std::pair<double, double>>>& boundaries,
    std::vector<double>* lb, std::vector<double>* ub) {
  if (boundaries.empty()) return true;
  CHECK_EQ(obstacle_location_hint.size(), boundaries.size());

  struct PathCandidate {
    int current_idx;
    double angle;
    Eigen::Vector2d pos;
    std::vector<int> selections;
    double cost = 0;

    std::unique_ptr<PathCandidate> Expand(int selection) {
      auto expansion = std::make_unique<PathCandidate>();
      expansion->current_idx = current_idx + 1;
      expansion->selections = selections;
      expansion->selections.emplace_back(selection);
      expansion->cost = cost;
      return expansion;
    }
  };

  int max_idx = boundaries.size() - 1;
  std::vector<std::unique_ptr<PathCandidate>> resutls;

  constexpr double weight_width = 0.4;
  constexpr double weight_dis = 0.0;
  constexpr double weight_angle = 1.0;

  // use BFS to find best initial path topology
  std::queue<std::unique_ptr<PathCandidate>> bfs_queue;

  auto init_node = std::make_unique<PathCandidate>();
  init_node->pos = init_pos;
  init_node->current_idx = -1;
  bfs_queue.push(std::move(init_node));

  while (!bfs_queue.empty()) {
    auto candidate = std::move(bfs_queue.front());
    bfs_queue.pop();
    const int current_idx = candidate->current_idx;
    if (current_idx == max_idx) {
      resutls.emplace_back(std::move(candidate));
      continue;
    }
    for (int j = 0; j < boundaries[current_idx + 1].size(); ++j) {
      double width = boundaries[current_idx + 1][j].second -
                     boundaries[current_idx + 1][j].first;
      if (width < ego_width_) continue;  // physically impossible
      auto expansion = candidate->Expand(j);
      expansion->pos =
          Eigen::Vector2d(obstacle_location_hint[current_idx + 1],
                          0.5 * (boundaries[current_idx + 1][j].first +
                                 boundaries[current_idx + 1][j].second));
      auto direction_vector = expansion->pos - candidate->pos;
      expansion->angle = std::atan2(direction_vector.y(), direction_vector.x());
      expansion->cost +=
          weight_angle *
          std::fabs(NormalizeAngle(expansion->angle - candidate->angle));
      // expansion->cost +=
      //     weight_dis * (expansion->pos - candidate->pos).squaredNorm();
      if (width < ego_width_ * 2) {
        expansion->cost += weight_width * (ego_width_ * 2 - width);
      }
      // printf("(%d, %d), width: %f, cost: %f, angle diff: %f\n", current_idx + 1,
      //        j, width, expansion->cost,
      //        NormalizeAngle(expansion->angle - candidate->angle));
      bfs_queue.push(std::move(expansion));
    }
  }

  if (resutls.empty()) {
    LOG(ERROR) << "Cannot find valid initial path, the road might be blocked";
    return false;
  }

  std::sort(resutls.begin(), resutls.end(),
            [](const std::unique_ptr<PathCandidate>& p1,
               const std::unique_ptr<PathCandidate>& p2) {
              return p1->cost <= p2->cost;
            });
  // for (const auto& p : resutls) {
  //   LOG(INFO) << "cost: " << p->cost;
  // }

  const auto& best_path = resutls.front();
  for (int i = 0; i < boundaries.size(); ++i) {
    int selection = best_path->selections[i];
    if (selection == 0) {
      lb->emplace_back(boundaries[i][selection].first);
      ub->emplace_back(boundaries[i][selection].second - ego_half_width_);
    } else if (selection == boundaries[i].size()) {
      lb->emplace_back(boundaries[i][selection].first + ego_half_width_);
      ub->emplace_back(boundaries[i][selection].second);
    } else {
      lb->emplace_back(boundaries[i][selection].first + ego_half_width_);
      ub->emplace_back(boundaries[i][selection].second - ego_half_width_);
    }
  }

  return true;
}

}  // namespace planning
