/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include "gp_planner/benchmark/DL_IAPS/coarse_trajectory_generator/hybrid_a_star.h"
#include "gp_planner/benchmark/DL_IAPS/iterative_anchoring_smoother.h"
#include "gp_planner/benchmark/TDR_OBCA/distance_approach_problem.h"

using namespace apollo::planning;
using namespace apollo::common::math;

int main(int argc, char const *argv[]) {
  // IterativeAnchoringSmoother dl_iaps_smoother;
  // Eigen::MatrixXd xWS(3, 101);
  // for (int i = 0; i <= 100; ++i) {
  //   xWS(0, i) = i * 0.5;
  // }
  // std::vector<std::vector<Vec2d>> vertices{
  //     {Vec2d(5.0, 3.0), Vec2d(6.0, 3.0), Vec2d(5.5, 5.0)}};
  // DiscretizedPath result;
  // dl_iaps_smoother.Smooth(xWS, vertices, &result);

  // HybridAStar hybrid_a_star;

  // double sx = -15.0;
  // double sy = 0.0;
  // double sphi = 0.0;
  // double ex = 15.0;
  // double ey = 0.0;
  // double ephi = 0.0;
  // std::vector<std::vector<Vec2d>> obstacles_list;
  // HybridAStartResult resutl2;
  // Vec2d obstacle_vertice_a(1.0, 0.0);
  // Vec2d obstacle_vertice_b(-1.0, 0.0);
  // std::vector<Vec2d> obstacle = {obstacle_vertice_a, obstacle_vertice_b};
  // // load xy boundary into the Plan() from configuration(Independent from
  // frame) std::vector<double> XYbounds_; XYbounds_.push_back(-50.0);
  // XYbounds_.push_back(50.0);
  // XYbounds_.push_back(-50.0);
  // XYbounds_.push_back(50.0);

  // obstacles_list.emplace_back(obstacle);
  // hybrid_a_star.Plan(sx, sy, sphi, ex, ey, ephi, XYbounds_, obstacles_list,
  //                    &resutl2);

  return 0;
}
