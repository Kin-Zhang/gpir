/* Copyright 2019 Unity-Drive Inc. All rights reserved */
/*     Author: ChengJie (chengjie@unity-drive.com)     */

#include "common/smoothing/osqp_spline2d_solver.h"

#include "common/utils/math.h"
#include "common/math/matplotlibcpp.h"

using namespace udrive::common;
using math::ComputeCurvature;
using math::NormalDistribution;

int main(int argc, char const* argv[]) {
  Eigen::VectorXd a(6), b(6);
  a << 0, 1.5, 0.4, -0.03, -0.02, 0.01;
  b << 0.5, 1.5, 0.4, -0.03, 0.05, 0.01;

  auto spline = [](const Eigen::VectorXd& a, const double t) -> double {
    return a[0] + a[1] * t + a[2] * t * t + a[3] * pow(t, 3) +
           a[4] * pow(t, 4) + a[5] * pow(t, 5);
  };

  auto spline_1st = [](const Eigen::VectorXd& a, const double t) -> double {
    return a[1] + 2 * a[2] * t + 3 * a[3] * pow(t, 2) + 4 * a[4] * pow(t, 3) +
           5 * a[5] * pow(t, 4);
  };

  auto spline_2st = [](const Eigen::VectorXd& a, const double t) -> double {
    return 2 * a[2] + 6 * a[3] * t + 12 * a[4] * pow(t, 2) +
           20 * a[4] * pow(t, 3);
  };

  std::vector<double> t_knots{0, 5.0};

  OsqpSpline2dSolver osqp_spline2d_solver(t_knots, 5);

  auto mutable_kernel = osqp_spline2d_solver.mutable_kernel();
  auto mutable_constraint = osqp_spline2d_solver.mutable_constraint();

  mutable_kernel->Add2dThirdOrderDerivativeMatrix(0.5);

  std::vector<double> t_coord;
  vector_Eigen<Eigen::Vector2d> ref_ft;
  std::vector<double> ref_x, ref_y;
  std::vector<double> lat_tol, lon_tol;
  std::vector<double> ref_theta;
  for (int i = 0; i < 5.0 / 0.25; ++i) {
    const double param = i * 0.25;
    t_coord.emplace_back(i * 0.25);
    double x1, y1;
    if (i == 0 || i == 19) {
      x1 = spline(a, param);
      y1 = spline(b, param);
    } else {
      x1 = spline(a, param) + NormalDistribution(0, 0.1);
      y1 = spline(b, param) + NormalDistribution(0, 0.1);
    }
    double theta = std::atan2(spline_1st(b, param), spline_1st(a, param));
    std::cout << theta / 3.14 * 180 << std::endl;
    ref_ft.emplace_back(Eigen::Vector2d(x1, y1));
    ref_x.emplace_back(x1);
    ref_y.emplace_back(y1);
    ref_theta.emplace_back(theta);
    lat_tol.emplace_back(0.20);
    lon_tol.emplace_back(0.20);
  }

  mutable_kernel->Add2dReferenceLineKernelMatrix(t_coord, ref_ft, 0.5);

  mutable_constraint->Add2dThirdDerivativeSmoothConstraint();

  mutable_constraint->Add2dPointConstraint(
      0, Eigen::Vector2d(spline(a, 0), spline(b, 0)));
  mutable_constraint->Add2dPointDerivativeConstraint(
      0, Eigen::Vector2d(spline_1st(a, 0), spline_1st(b, 0)));

  double t_end = t_coord.back();
  mutable_constraint->Add2dPointConstraint(
      t_end, Eigen::Vector2d(spline(a, t_end), spline(b, t_end)));
  mutable_constraint->Add2dPointDerivativeConstraint(
      t_end, Eigen::Vector2d(spline_1st(a, t_end), spline_1st(b, t_end)));

  mutable_constraint->Add2dStationLateralBoundary(t_coord, ref_ft, ref_theta,
                                                  lon_tol, lat_tol);

  auto res = osqp_spline2d_solver.Solve();
  auto res_spline = osqp_spline2d_solver.spline();

  std::vector<double> res_x, res_y;
  std::vector<double> ref_kappa, res_kappa;

  for (auto const t : t_coord) {
    res_x.emplace_back(res_spline.x(t));
    res_y.emplace_back(res_spline.y(t));

    ref_kappa.emplace_back(ComputeCurvature(spline_1st(a, t), spline_2st(a, t),
                                            spline_1st(b, t),
                                            spline_2st(b, t)));
    res_kappa.emplace_back(ComputeCurvature(
        res_spline.DerivativeX(t), res_spline.SecondDerivativeX(t),
        res_spline.DerivativeY(t), res_spline.SecondDerivativeY(t)));
  }

  namespace plt = matplotlibcpp;
  plt::plot(ref_x, ref_y, "b-o");
  plt::plot(res_x, res_y, "r-o");
  plt::axis("equal");
  plt::figure();
  plt::plot(t_coord, ref_kappa, "b-o");
  plt::plot(t_coord, res_kappa, "r-o");
  plt::show();

  return 0;
}
