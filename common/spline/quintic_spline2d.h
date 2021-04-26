/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#pragma once

#include <Eigen/Core>
#include <array>
#include <vector>

#include "common/base/type.h"
#include "common/spline/quintic_polynomial.h"

namespace common {

class QuinticSpline2D {
 public:
  QuinticSpline2D() = default;

  Eigen::Vector2d pos(const double s) const;
  Eigen::Vector2d vel(const double s) const;
  Eigen::Vector2d acc(const double s) const;
  Eigen::Vector2d jerk(const double s) const;

  double theta(const double s) const;
  double kappa(const double s) const;
  double dkappa(const double s) const;

  bool PolyFit(const std::vector<double>& knots,
               const vector_Eigen<Eigen::Vector2d>& refs,
               const std::vector<double>& paras);
  void GetCurvatureInfo(const double s, double* kappa, double* dkappa);

  void print();

 private:
  bool Optimize(const vector_Eigen<Eigen::Vector2d>& refs,
                const std::vector<double>& paras);

  bool OptimizeDense(const vector_Eigen<Eigen::Vector2d>& refs,
                     const std::vector<double>& paras);

  int FindIndex(const double s) const;
  int NormalizeParam(const double para, double* normalized_para) const;
  void GetPowers(const double s, std::array<double, 6>* powers) const;

 private:
  const int dim_ = 2;
  const int num_of_params_ = 6;
  int total_params_ = 0;

  Eigen::Vector2d origin_;
  std::array<double, 6> powers_;

  std::vector<std::array<QuinticPolynomial, 2>> polys_;

  // * e.g. *------*------* knots = 3
  std::vector<double> knots_;
  std::vector<double> knots_inv_;
};

}  // namespace common
