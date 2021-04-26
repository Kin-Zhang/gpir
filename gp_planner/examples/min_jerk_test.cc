/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include <iostream>

#include "gp_planner/thirdparty/matplotlib-cpp/matplotlibcpp.h"
#include "gp_planner/thirdparty/traj_min_jerk/traj_min_jerk.hpp"

namespace plt = matplotlibcpp;
using namespace std;

int main(int argc, char *argv[]) {
  Eigen::MatrixXd route(1, 4);
  route << 0, 5, 5, 5;
  Eigen::VectorXd ts(3);
  ts << 3, 2, 2;

  Eigen::Matrix<double, 1, 3> is, fs;
  is << 0, 1, 0;
  fs << 5, 0, 0;

  auto a = route.block(0, 1, 1, 0);

  min_jerk::Trajectory traj;
  min_jerk::JerkOpt jerk_opt;
  jerk_opt.reset(is, fs, route.cols() - 1);
  jerk_opt.generate(route.block(0, 1, 1, route.cols() - 2), ts);
  jerk_opt.getTraj(traj);

  double total = ts.sum();
  double t_now = 0;
  std::vector<double> t, s;
  while (t_now < total) {
    t.emplace_back(t_now);
    s.emplace_back(traj.getPos(t_now)(0));
    t_now += 0.1;
  }

  cout << traj.getMaxVelRate() << endl;

  plt::plot(t, s);
  plt::show();

  return 0;
}
