/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include <iostream>

#include "gp_planner/gp/interpolator/gp_interpolator.h"
#include "gp_planner/gp/model/white_noise_on_jerk_model_1d.h"
#include "matplot/matplot.h"

using namespace planning;
using namespace std;

int main(int argc, char const *argv[]) {
  const double t = 0.4;
  const double qc = 1.0;
  Eigen::Matrix3d Q, Q_inv;
  Q << 1 / 2. * std::pow(t, 5) * qc, 1 / 8. * std::pow(t, 4) * qc,
      1 / 6. * std::pow(t, 3) * qc, 1 / 8. * std::pow(t, 4) * qc,
      1 / 3. * std::pow(t, 3) * qc, 1 / 2. * std::pow(t, 2) * qc,
      1 / 6. * std::pow(t, 3) * qc, 1 / 2. * std::pow(t, 2) * qc, t * qc;
  Q_inv << 720 * std::pow(t, -5) / qc, -360 * std::pow(t, -4) / qc,
      60 * std::pow(t, -3) / qc, -360 * std::pow(t, -4) / qc,
      192 * std::pow(t, -3) / qc, -36 * std::pow(t, -2) / qc,
      60 * std::pow(t, -3) / qc, -36 * std::pow(t, -2) / qc, 9 / t / qc;

  cout << Q << std::endl;
  cout << WhiteNoiseOnJerkModel1D::Q(qc, t) << endl;

  cout << "\n" << Q_inv << endl;
  cout << WhiteNoiseOnJerkModel1D::QInverse(qc, t) << endl;


  cout << WhiteNoiseOnJerkModel1D::Phi(0.5) << endl;

  Eigen::Vector3d x1(2, 0, 0);
  Eigen::Vector3d x2(5, 0, 0);

  const double interval = 10;
  const double step = interval / 100;

  vector<double> t_plot, x;
  for (int i = 0; i < 101; ++i) {
    auto tmp = GPInterpolator::Interpolate(x1, x2, qc, interval, step * i);
    t_plot.emplace_back(i * step);
    // std::cout << tmp << std::endl;
    x.emplace_back(tmp(0));
  }

  namespace plt = matplot;
  plt::plot(t_plot, x, "*");
  plt::show();

  return 0;
}
