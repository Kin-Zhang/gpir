/* Copyright 2021 Unity-Drive Inc. All rights reserved */

// good illustration: rosrun gp_planner simple_kappa_test 2 0.2 0.3

#include <cstdlib>

#include "common/frenet/frenet_transform.h"
#include "gp_planner/gp/factors/gp_interpolate_kappa_limit_factor.h"
#include "gp_planner/gp/factors/gp_interpolate_obstacle_factor.h"
#include "gp_planner/gp/factors/gp_kappa_limit_factor.h"
#include "gp_planner/gp/factors/gp_obstacle_factor.h"
#include "gp_planner/gp/factors/gp_prior_factor.h"
#include "gp_planner/initializer/gp_initializer.h"
#include "gp_planner/sdf/signed_distance_field_2d.h"
#include "gp_planner/thirdparty/matplotlib-cpp/matplotlibcpp.h"
#include "gp_planner/thirdparty/traj_min_jerk/traj_min_jerk.hpp"
#include "gtsam/inference/Symbol.h"
#include "gtsam/nonlinear/DoglegOptimizer.h"
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
  // grid_map.FillCircle(Eigen::Vector2d(30, -0.5), 1.5);
  // grid_map.FillEntireRow(-10);
  // grid_map.FillEntireRow(9.9);
  grid_map.FillConvexPoly(vector_Eigen<Eigen::Vector2d>{
      Eigen::Vector2d(20, 0.0), Eigen::Vector2d(25, 0.0),
      Eigen::Vector2d(25, 10), Eigen::Vector2d(20, 10)});

  grid_map.FillConvexPoly(vector_Eigen<Eigen::Vector2d>{
      Eigen::Vector2d(40, 0.0), Eigen::Vector2d(45, 0.0),
      Eigen::Vector2d(45, -10), Eigen::Vector2d(40, -10)});

  std::shared_ptr<SignedDistanceField2D> sdf =
      std::make_shared<SignedDistanceField2D>(std::move(grid_map));
  sdf->UpdateSDF();

  Eigen::Vector2d grad;

  // cv::imshow("grid_map", sdf->occupancy_mat().BinaryImage());
  // cv::imshow("sdf", sdf->esdf().ImageSec());
  // cv::waitKey(0);

  const double t_total = 70;
  const int N = 20;
  const double interval = t_total / (N - 1);

  Vector3 x0(0.0, 0.0, 0.0);
  Vector3 xn(atof(argv[1]), 0, 0);

  double vel = (xn(0) - x0(0)) / t_total;

  NonlinearFactorGraph graph;
  NonlinearFactorGraph graph2;

  auto pose_fix_cost = noiseModel::Isotropic::Sigma(3, 0.001);
  // auto pose_fix_cost2 = noiseModel::Isotropic::Sigma(3, 1);
  auto pose_fix_cost2 = noiseModel::Diagonal::Sigmas(Vector3(1, 0.1, 0.1));
  const double qc = 0.1;

  Values init_values;

  const double kappa = atof(argv[2]);
  const double epsilon = 3.0;
  const int collision_check_num = 5;
  const double tau = interval / (collision_check_num + 1);
  std::vector<double> x_coords;

  std::vector<double> kappa_r, dkappa_r;

  for (int i = 0; i < N; ++i) {
    Key key = Symbol('x', i);
    if (i == 0) {
      graph.add(PriorFactor<Vector3>(key, x0, pose_fix_cost));
      graph2.add(PriorFactor<Vector3>(key, x0, pose_fix_cost));
    }
    if (i == N - 1) {
      graph.add(PriorFactor<Vector3>(key, xn, pose_fix_cost2));
      graph2.add(PriorFactor<Vector3>(key, xn, pose_fix_cost2));
    }

    if (i > 0) {
      Key last_key = Symbol('x', i - 1);
      graph.add(GPPriorFactor(last_key, key, interval, qc));
      graph.add(GPObstacleFactor(key, sdf, 0.01, epsilon, interval * i, kappa));

      graph2.add(GPPriorFactor(last_key, key, interval, qc));
      graph2.add(
          GPObstacleFactor(key, sdf, 0.01, epsilon, interval * i, kappa));
      graph2.add(GPKappaLimitFactor(key, 0.1, kappa, 0.0, atof(argv[3]),
                                    interval * i));

      for (int j = 0; j < collision_check_num; ++j) {
        graph.add(GPInterpolateObstacleFactor(last_key, key, sdf, 0.01, epsilon,
                                              interval * (i - 1), qc, interval,
                                              tau * (j + 1), kappa));
        graph2.add(GPInterpolateObstacleFactor(last_key, key, sdf, 0.01,
                                               epsilon, interval * (i - 1), qc,
                                               interval, tau * (j + 1), kappa));
        graph2.add(GPInterpolateKappaLimitFactor(last_key, key, 0.1, qc,
                                                 interval, tau * (j + 1), kappa,
                                                 0.0, atof(argv[3])));
      }
    }

    x_coords.emplace_back(interval * i);
    kappa_r.emplace_back(kappa);
    dkappa_r.emplace_back(0.0);
  }

  LOG(INFO) << "?";
  auto tt0 = std::chrono::high_resolution_clock::now();
  std::vector<std::vector<std::pair<double, double>>> boundaries;
  std::vector<double> seeds;
  seeds.emplace_back(22.5);
  seeds.emplace_back(42.5);
  sdf->mutable_occupancy_map()->SearchForVerticalBoundaries(seeds, &boundaries);

  LOG(INFO) << 1;
  Eigen::MatrixXd route(1, 2);
  route << (boundaries[0].front().first + boundaries[0].front().second) / 2.0,
      (boundaries[1].front().first + boundaries[1].front().second) / 2.0;
  min_jerk::JerkOpt jerk_opt;
  jerk_opt.reset(x0.transpose(), xn.transpose(), 3);
  jerk_opt.generate(route, Eigen::Vector3d(22.5, 20, t_total - 22.5 - 20));
  min_jerk::Trajectory traj;
  jerk_opt.getTraj(traj);
  LOG(INFO) << 2;

  std::vector<double> l;
  for (int i = 0; i < x_coords.size(); ++i) {
    gtsam::Key key = gtsam::symbol('x', i);
    Eigen::Vector3d val(traj.getPos(x_coords[i]).x(),
                        traj.getVel(x_coords[i]).x(),
                        traj.getAcc(x_coords[i]).x());
    l.emplace_back(traj.getPos(x_coords[i]).x());
    init_values.insert<gtsam::Vector3>(key, val);
  }

  // std::cout << boundaries.size() << std::endl;
  // GPInitializer initializer;
  // initializer.SetBoundary(boundaries);
  // if (!initializer.Solve(std::array<double, 3>{x0[0], x0[1], x0[2]},
  //                        std::array<double, 3>{xn[0], xn[1], xn[2]},
  //                        interval, &init_values, &l)) {
  //   std::cerr << "initializer failed" << std::endl;
  // }
  auto tt1 = std::chrono::high_resolution_clock::now();
  std::cout << "init time: "
            << std::chrono::duration_cast<std::chrono::nanoseconds>(tt1 - tt0)
                       .count() /
                   1e6
            << std::endl;

  vector_Eigen<Eigen::Vector2d> init_line;
  for (int i = 0; i < x_coords.size(); ++i) {
    init_line.emplace_back(Eigen::Vector2d(x_coords[i], l[i]));
  }
  sdf->mutable_occupancy_map()->PolyLine(init_line);

  LevenbergMarquardtParams param;
  param.setlambdaInitial(100.0);
  param.setAbsoluteErrorTol(1e-4);
  param.setVerbosity("ERROR");

  auto t0 = std::chrono::high_resolution_clock::now();
  LevenbergMarquardtOptimizer opt(graph, init_values, param);
  LevenbergMarquardtOptimizer opt2(graph2, init_values, param);
  auto res = opt.optimize();
  auto t1 = std::chrono::high_resolution_clock::now();
  auto res2 = opt2.optimize();
  auto ta2 = std::chrono::high_resolution_clock::now();

  std::cout << "time consumption: "
            << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0)
                       .count() /
                   1000.0
            << " ms"
            << "next: "
            << std::chrono::duration_cast<std::chrono::microseconds>(ta2 - t1)
                       .count() /
                   1000.0
            << " ms" << std::endl;

  std::vector<double> t, x, v, a, k;
  std::vector<double> t2, ax2, v2, a2, k2;

  int inter_num = 20;
  double delta_tau = interval / (inter_num + 1);

  for (size_t i = 0; i < res.size() - 1; ++i) {
    auto x1 = res.at<Vector3>(Symbol('x', i));
    auto x2 = res.at<Vector3>(Symbol('x', i + 1));
    auto xx1 = res2.at<Vector3>(Symbol('x', i));
    auto xx2 = res2.at<Vector3>(Symbol('x', i + 1));
    t.emplace_back(i * interval);
    x.emplace_back(x1(0));
    v.emplace_back(x1(1));
    a.emplace_back(x1(2));
    k.emplace_back(common::FrenetTransfrom::GetCurvature(
        std::array<double, 3>{x1(0), x1(1), x1(2)}, kappa, 0.0));
    t2.emplace_back(i * interval);
    ax2.emplace_back(xx1(0));
    v2.emplace_back(xx1(1));
    a2.emplace_back(xx1(2));
    k2.emplace_back(common::FrenetTransfrom::GetCurvature(
        std::array<double, 3>{xx1(0), xx1(1), xx1(2)}, kappa, 0.0));

    for (int j = 0; j < inter_num; ++j) {
      double tau = (j + 1) * delta_tau;
      auto inter_x = GPInterpolator::Interpolate(x1, x2, qc, interval, tau);
      auto inter_x2 = GPInterpolator::Interpolate(xx1, xx2, qc, interval, tau);
      t.emplace_back(i * interval + tau);
      x.emplace_back(inter_x(0));
      v.emplace_back(inter_x(1));
      a.emplace_back(inter_x(2));
      double cur = common::FrenetTransfrom::GetCurvature(
          std::array<double, 3>{inter_x(0), inter_x(1), inter_x(2)}, kappa,
          0.0);
      if (fabs(cur) > 1) {
        LOG(INFO) << inter_x(0) << ", " << inter_x(1) << ", "
                  << ", " << inter_x(2) << ", " << cur;
      }
      k.emplace_back(cur);
      t2.emplace_back(i * interval + tau);
      ax2.emplace_back(inter_x2(0));
      v2.emplace_back(inter_x2(1));
      a2.emplace_back(inter_x2(2));
      k2.emplace_back(common::FrenetTransfrom::GetCurvature(
          std::array<double, 3>{inter_x2(0), inter_x2(1), inter_x2(2)}, kappa,
          0.0));
    }
  }

  vector_Eigen<Eigen::Vector2d> points, points2;
  for (int i = 0; i < x.size(); ++i) {
    points.emplace_back(Eigen::Vector2d(t[i], x[i]));
    points2.emplace_back(Eigen::Vector2d(t[i], ax2[i]));
  }

  cv::imshow("init", sdf->occupancy_map().BinaryImage());
  sdf->mutable_occupancy_map()->PolyLine(points);
  cv::imshow("no", sdf->occupancy_map().BinaryImage());
  sdf->mutable_occupancy_map()->PolyLine(points2);
  cv::imshow("with", sdf->occupancy_map().BinaryImage());
  // cv::imshow("sdf", sdf->esdf().ImageSec());
  cv::waitKey(0);
  // sdf->mutable_esdf()->ShowGradientFeild();

  namespace plt = matplotlibcpp;
  plt::subplot(4, 1, 1);
  plt::named_plot("nokappa", t, x);
  plt::plot(t, ax2);
  plt::axis("equal");
  plt::subplot(4, 1, 2);
  plt::named_plot("no", t, v);
  plt::plot(t, v2);
  plt::subplot(4, 1, 3);
  plt::named_plot("no", t, a);
  plt::plot(t, a2);
  plt::subplot(4, 1, 4);
  plt::named_plot("no", t, k);
  plt::plot(t, k2);
  plt::show();

  return 0;
}
