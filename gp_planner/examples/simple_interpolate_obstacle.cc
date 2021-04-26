/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include <cstdlib>

#include "gp_planner/gp/factors/gp_interpolate_obstacle_factor.h"
#include "gp_planner/gp/factors/gp_obstacle_factor.h"
#include "gp_planner/gp/factors/gp_prior_factor.h"
#include "gp_planner/sdf/signed_distance_field_2d.h"
#include "gtsam/inference/Symbol.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/LevenbergMarquardtParams.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/nonlinear/NonlinearOptimizer.h"

using namespace gtsam;
using namespace planning;

int main(int argc, char** argv) {
  OccupancyMap grid_map;
  grid_map.set_origin(std::array<double, 2>{0, -10});
  grid_map.set_cell_number(std::array<int, 2>{800, 200});
  grid_map.set_resolution(std::array<double, 2>{0.1, 0.1});
  // grid_map.FillCircle(Eigen::Vector2d(30, 0), 1.5);
  grid_map.FillEntireRow(9.9);
  grid_map.FillEntireRow(-10);
  grid_map.FillConvexPoly(vector_Eigen<Eigen::Vector2d>{
      Eigen::Vector2d(20, 0.5), Eigen::Vector2d(25, 0.5),
      Eigen::Vector2d(25, 10), Eigen::Vector2d(20, 10)});

  // grid_map.FillConvexPoly(vector_Eigen<Eigen::Vector2d>{
  //     Eigen::Vector2d(40, 2), Eigen::Vector2d(45, 2), Eigen::Vector2d(45,
  //     -10), Eigen::Vector2d(40, -10)});

  std::shared_ptr<SignedDistanceField2D> sdf =
      std::make_shared<SignedDistanceField2D>(std::move(grid_map));
  sdf->UpdateSDF();

  const double t_total = 60;
  const int N = 20;
  const double interval = t_total / (N - 1);

  Vector3 x0(0.0, 0.0, 0.0);
  Vector3 xn(atof(argv[1]), 0.0, 0.0);

  double vel = (xn(0) - x0(0)) / t_total;

  NonlinearFactorGraph graph;

  auto pose_fix_cost = noiseModel::Isotropic::Sigma(3, 0.001);

  const double qc = 1;
  Values init_values;

  const double epsilon = 3;
  const int collision_check_num = 5;
  const double tau = interval / (collision_check_num + 1);

  for (int i = 0; i < N; ++i) {
    Key key = Symbol('x', i);
    if (i == 0) graph.add(PriorFactor<Vector3>(key, x0, pose_fix_cost));
    if (i == N - 1) graph.add(PriorFactor<Vector3>(key, xn, pose_fix_cost));

    if (i > 0) {
      Key last_key = Symbol('x', i - 1);
      graph.add(GPPriorFactor(last_key, key, interval, qc));
      graph.add(GPObstacleFactor(key, sdf, 0.1, epsilon, interval * i, 0.0));

      for (int j = 0; j < collision_check_num; ++j) {
        graph.add(GPInterpolateObstacleFactor(last_key, key, sdf, 0.01, epsilon,
                                              interval * (i - 1), qc, interval,
                                              tau * (j + 1), 0.0));
      }
    }

    Vector3 value(vel * interval * i, vel, 0.0);
    init_values.insert<Vector3>(key, value);
  }

  LevenbergMarquardtParams param;
  // param.setlambdaInitial(100.0);
  param.setVerbosity("ERROR");

  LevenbergMarquardtOptimizer opt(graph, init_values, param);

  auto t0 = std::chrono::high_resolution_clock::now();
  auto res = opt.optimize();
  auto t1 = std::chrono::high_resolution_clock::now();

  std::cout << "time consumption: "
            << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0)
                       .count() /
                   1000.0
            << " ms" << std::endl;

  std::vector<double> t, x, v;

  int inter_num = 20;
  double delta_tau = interval / inter_num;

  for (size_t i = 0; i < res.size() - 1; ++i) {
    auto x1 = res.at<Vector3>(Symbol('x', i));
    auto x2 = res.at<Vector3>(Symbol('x', i + 1));
    t.emplace_back(i * interval);
    x.emplace_back(x1(0));
    v.emplace_back(x1(1));

    for (int j = 0; j < inter_num; ++j) {
      double tau = (j + 1) * delta_tau;
      auto inter_x = GPInterpolator::Interpolate(x1, x2, qc, interval, tau);
      t.emplace_back(i * interval + tau);
      x.emplace_back(inter_x(0));
      v.emplace_back(inter_x(1));
    }
  }

  vector_Eigen<Eigen::Vector2d> points;
  for (int i = 0; i < x.size(); ++i) {
    points.emplace_back(Eigen::Vector2d(t[i], x[i]));
  }

  sdf->mutable_occupancy_map()->PolyLine(points);
  cv::imshow("sdf", sdf->esdf().ImageSec());
  cv::imshow("path", sdf->occupancy_map().BinaryImage());
  cv::waitKey(0);

  // plt::subplot(2, 1, 1);
  // plt::plot(t, x);
  // plt::subplot(2, 1, 2);
  // plt::plot(t, v);
  // plt::show();

  return 0;
}
