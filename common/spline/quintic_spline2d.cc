/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include "common/spline/quintic_spline2d.h"

#include <glog/logging.h>

#include <algorithm>
#include <iomanip>

#include "common/solver/qp_problem.h"
#include "common/utils/math.h"
#include "common/utils/timer.h"

namespace common {

Eigen::Vector2d QuinticSpline2D::pos(const double s) const {
  double s_normalized;
  int index = NormalizeParam(s, &s_normalized);
  return Eigen::Vector2d(polys_[index][0].f(s_normalized),
                         polys_[index][1].f(s_normalized));
}

Eigen::Vector2d QuinticSpline2D::vel(const double s) const {
  double s_normalized;
  int index = NormalizeParam(s, &s_normalized);
  return Eigen::Vector2d(polys_[index][0].df(s_normalized),
                         polys_[index][1].df(s_normalized));
}

Eigen::Vector2d QuinticSpline2D::acc(const double s) const {
  double s_normalized;
  int index = NormalizeParam(s, &s_normalized);
  return Eigen::Vector2d(polys_[index][0].ddf(s_normalized),
                         polys_[index][1].ddf(s_normalized));
}

Eigen::Vector2d QuinticSpline2D::jerk(const double s) const {
  double s_normalized;
  int index = NormalizeParam(s, &s_normalized);
  return Eigen::Vector2d(polys_[index][0].dddf(s_normalized),
                         polys_[index][1].dddf(s_normalized));
}

double QuinticSpline2D::theta(const double s) const {
  auto d = vel(s);
  return std::atan2(d.y(), d.x());
}

double QuinticSpline2D::kappa(const double s) const {
  double s_normalized;
  int index = NormalizeParam(s, &s_normalized);

  double dx = polys_[index][0].df(s_normalized);
  double ddx = polys_[index][0].ddf(s_normalized);
  double dy = polys_[index][1].df(s_normalized);
  double ddy = polys_[index][1].ddf(s_normalized);

  return Curvature(dx, ddx, dy, ddy);
}

double QuinticSpline2D::dkappa(const double s) const {
  double s_normalized;
  int index = NormalizeParam(s, &s_normalized);

  const double dx = polys_[index][0].df(s_normalized);
  const double ddx = polys_[index][0].df(s_normalized);
  const double dddx = polys_[index][0].dddf(s_normalized);
  const double dy = polys_[index][1].df(s_normalized);
  const double ddy = polys_[index][1].ddf(s_normalized);
  const double dddy = polys_[index][1].dddf(s_normalized);

  return CurvatureDerivative(dx, ddx, dddx, dy, ddy, dddy);
}

void QuinticSpline2D::GetCurvatureInfo(const double s, double* kappa,
                                       double* dkappa) {
  double s_normalized;
  int index = NormalizeParam(s, &s_normalized);

  const double dx = polys_[index][0].df(s_normalized);
  const double ddx = polys_[index][0].df(s_normalized);
  const double dddx = polys_[index][0].dddf(s_normalized);
  const double dy = polys_[index][1].df(s_normalized);
  const double ddy = polys_[index][1].ddf(s_normalized);
  const double dddy = polys_[index][1].dddf(s_normalized);

  (*kappa) = Curvature(dx, ddx, dy, ddy);
  (*dkappa) = CurvatureDerivative(dx, ddx, dddx, dy, ddy, dddy);
}

bool QuinticSpline2D::PolyFit(const std::vector<double>& knots,
                              const vector_Eigen<Eigen::Vector2d>& refs,
                              const std::vector<double>& paras) {
  knots_ = knots;
  return Optimize(refs, paras);
  // return OptimizeDense(refs, paras);
}

int QuinticSpline2D::FindIndex(const double s) const {
  auto upper_bound = std::upper_bound(knots_.begin(), knots_.end(), s);
  return std::min<int>(upper_bound - knots_.begin() - 1, knots_.size() - 2);
}

int QuinticSpline2D::NormalizeParam(const double para,
                                    double* normalized_para) const {
  int index = FindIndex(para);
  (*normalized_para) =
      (para - knots_[index]) / (knots_[index + 1] - knots_[index]);
  return index;
}

void QuinticSpline2D::GetPowers(const double s,
                                std::array<double, 6>* powers) const {
  (*powers)[0] = 1.0;
  for (int i = 1; i < 6; ++i) {
    (*powers)[i] = s * (*powers)[i - 1];
  }
}

bool QuinticSpline2D::Optimize(const vector_Eigen<Eigen::Vector2d>& refs,
                               const std::vector<double>& paras) {
  CHECK_EQ(refs.size(), paras.size());

  Timer timer;

  int num_of_segments = knots_.size() - 1;
  int num_of_samples = refs.size();
  total_params_ = dim_ * num_of_segments * num_of_params_;

  ColSparseMatrix A(dim_ * num_of_samples, total_params_);
  Eigen::VectorXd b(A.rows());

  int row_offset = num_of_samples;
  int col_offset = num_of_params_ * num_of_segments;

  // * reference term
  int idx_col, idx_row;
  double para_normalized = 0.0;
  for (int i = 0; i < num_of_samples; ++i) {
    int index = NormalizeParam(paras[i], &para_normalized);
    GetPowers(para_normalized, &powers_);
    for (int j = 0; j < num_of_params_; j++) {
      idx_col = index * num_of_params_ + j;
      A.insert(i, idx_col) = powers_[j];
      A.insert(row_offset + i, col_offset + idx_col) = powers_[j];
      b(i) = refs[i].x();
      b(row_offset + i) = refs[i].y();
    }
  }

  // * equality constarint: continuous to 3th order derivative
  static std::vector<std::vector<double>> coeffs{
      {1, 1, 1, 1, 1, 1}, {1, 2, 3, 4, 5}, {2, 6, 12, 20}, {6, 24, 60}};

  int order_of_derivative = coeffs.size();
  int num_of_connections = knots_.size() - 2;
  int constraint_offset = order_of_derivative * num_of_connections;

  ColSparseMatrix C(dim_ * constraint_offset, total_params_);
  Eigen::VectorXd l = Eigen::VectorXd::Zero(C.rows());
  Eigen::VectorXd u = l;

  for (int i = 0; i < num_of_connections; ++i) {
    for (int j = 0; j < order_of_derivative; ++j) {
      idx_row = order_of_derivative * i + j;
      for (int k = 0; k < coeffs[j].size(); ++k) {
        idx_col = i * num_of_params_ + j + k;
        C.insert(idx_row, idx_col) = coeffs[j][k];
        C.insert(idx_row + constraint_offset, idx_col + col_offset) =
            coeffs[j][k];
      }
      idx_col = (i + 1) * num_of_params_ + j;
      C.insert(idx_row, idx_col) = -coeffs[j][0];
      C.insert(idx_row + constraint_offset, idx_col + col_offset) =
          -coeffs[j][0];
    }
  }

  // * weight
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> W(dim_ * num_of_samples);
  W.diagonal() = Eigen::VectorXd::Ones(dim_ * num_of_samples);

  // * result
  Eigen::VectorXd x;
  if (!QPProblem::Solve(A, b, W, 1e-5, C, l, u, &x)) {
    LOG(ERROR) << "Fail to optimize spline";
    return false;
  }

  polys_.resize(num_of_segments);
  QuinticCoefficients x_coeff, y_coeff;
  for (int i = 0; i < num_of_segments; ++i) {
    for (int j = 0; j < num_of_params_; ++j) {
      x_coeff[j] = x(i * num_of_params_ + j);
      y_coeff[j] = x(col_offset + i * num_of_params_ + j);
    }
    polys_[i][0].set_coeff(x_coeff);
    polys_[i][1].set_coeff(y_coeff);
  }

  timer.End("QuinticSpline2D optimization");
  return true;
}

bool QuinticSpline2D::OptimizeDense(const vector_Eigen<Eigen::Vector2d>& refs,
                                    const std::vector<double>& paras) {
  CHECK_EQ(refs.size(), paras.size());

  Timer timer;

  int num_of_segments = knots_.size() - 1;
  int num_of_samples = refs.size();
  total_params_ = dim_ * num_of_segments * num_of_params_;

  Eigen::MatrixXd A =
      Eigen::MatrixXd::Zero(dim_ * num_of_samples, total_params_);
  Eigen::VectorXd b(A.rows());

  int row_offset = num_of_samples;
  int col_offset = num_of_params_ * num_of_segments;

  // * reference term
  int idx_col, idx_row;
  double para_normalized = 0.0;
  for (int i = 0; i < num_of_samples; ++i) {
    int index = NormalizeParam(paras[i], &para_normalized);
    GetPowers(para_normalized, &powers_);
    for (int j = 0; j < num_of_params_; j++) {
      idx_col = index * num_of_params_ + j;
      A(i, idx_col) = powers_[j];
      A(row_offset + i, col_offset + idx_col) = powers_[j];
      b(i) = refs[i].x();
      b(row_offset + i) = refs[i].y();
    }
  }

  // * equality constarint: continuous to 3th order derivative
  static std::vector<std::vector<double>> coeffs{
      {1, 1, 1, 1, 1, 1}, {1, 2, 3, 4, 5}, {2, 6, 12, 20}, {6, 24, 60}};

  int order_of_derivative = coeffs.size();
  int num_of_connections = knots_.size() - 2;
  int constraint_offset = order_of_derivative * num_of_connections;

  Eigen::MatrixXd C =
      Eigen::MatrixXd::Zero(dim_ * constraint_offset, total_params_);
  Eigen::VectorXd l = Eigen::VectorXd::Zero(C.rows());
  Eigen::VectorXd u = l;

  for (int i = 0; i < num_of_connections; ++i) {
    for (int j = 0; j < order_of_derivative; ++j) {
      idx_row = order_of_derivative * i + j;
      for (int k = 0; k < coeffs[j].size(); ++k) {
        idx_col = i * num_of_params_ + j + k;
        C(idx_row, idx_col) = coeffs[j][k];
        C(idx_row + constraint_offset, idx_col + col_offset) = coeffs[j][k];
      }
      idx_col = (i + 1) * num_of_params_ + j;
      C(idx_row, idx_col) = -coeffs[j][0];
      C(idx_row + constraint_offset, idx_col + col_offset) = -coeffs[j][0];
    }
  }

  // * weight
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> W(dim_ * num_of_samples);
  W.diagonal() = Eigen::VectorXd::Ones(dim_ * num_of_samples);

  // * result
  Eigen::VectorXd x;
  if (!QPProblem::Solve(A, b, W, 1e-4, C, l, u, &x)) {
    LOG(ERROR) << "Fail to optimize spline";
    return false;
  }

  polys_.resize(num_of_segments);
  QuinticCoefficients x_coeff, y_coeff;
  for (int i = 0; i < num_of_segments; ++i) {
    for (int j = 0; j < num_of_params_; ++j) {
      x_coeff[j] = x(i * num_of_params_ + j);
      y_coeff[j] = x(col_offset + i * num_of_params_ + j);
    }
    polys_[i][0].set_coeff(x_coeff);
    polys_[i][1].set_coeff(y_coeff);
  }

  timer.End("QuinticSpline2D optimization");
  return true;
}
void QuinticSpline2D::print() {
  int seg = 0;
  for (const auto& poly : polys_) {
    std::cout << seg++ << ".\nx: ";
    poly[0].print();
    std::cout << "y: ";
    poly[1].print();
  }
}
}  // namespace common
