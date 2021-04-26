/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include "gp_planner/gp/factors/gp_prior_factor.h"
#include "gtsam/inference/Symbol.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/LevenbergMarquardtParams.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/nonlinear/NonlinearOptimizer.h"
#include "matplot/matplot.h"

using namespace gtsam;
using namespace planning;

int main() {
  const double t_total = 50;
  const int N = 50;
  const double interval = t_total / (N - 1);

  Vector3 x0(0.0, 0.0, 0.0);
  Vector3 xn(2.0, 0.0, 0.0);

  double vel = (xn(0) - x0(0)) / t_total;

  NonlinearFactorGraph graph;

  auto pose_fix_cost = noiseModel::Isotropic::Sigma(3, 0.001);

  const double qc = 1;

  Values init_values;

  for (int i = 0; i < N; ++i) {
    Key key = Symbol('x', i);
    if (i == 0) graph.add(PriorFactor<Vector3>(key, x0, pose_fix_cost));
    if (i == N - 1) graph.add(PriorFactor<Vector3>(key, xn, pose_fix_cost));

    if (i > 0) {
      Key last_key = Symbol('x', i - 1);
      graph.add(GPPriorFactor(last_key, key, interval, qc));
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

  // ConstVelocityGPInterpolator interpolator(interval, Qc);

  namespace plt = matplot;
  std::vector<double> t, x, v;

  int inter_num = 5;
  double delta_tau = interval / inter_num;

  for (size_t i = 0; i < res.size(); ++i) {
    auto x1 = res.at<Vector3>(Symbol('x', i));
    // auto x2 = res.at<Vector3>(Symbol('x', i + 1));
    t.emplace_back(i * interval);
    x.emplace_back(x1(0));
    v.emplace_back(x1(1));

    // for (int j = 0; j < 5; ++j) {
    //   double tau = (j + 1) * delta_tau;
    //   auto inter_x = interpolator.Interpolate(x1, x2, tau);
    //   t.emplace_back(i * interval + tau);
    //   x.emplace_back(inter_x(0));
    //   v.emplace_back(inter_x(1));
    // }
  }

  // plt::subplot(2, 1, 1);
  plt::plot(t, x);
  // plt::subplot(2, 1, 2);
  // plt::plot(t, v);
  plt::show();

  return 0;
}
