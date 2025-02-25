

#pragma once

#include <glog/logging.h>
#include <Eigen/Core>
#include <algorithm>
#include <iostream>
#include <vector>

#include "common/smoothing/affine_constraint.h"

namespace common {
/**
 * @class Spline1dConstraint
 * @brief specify one-dimension polynomial spline constraint in matrix form
 */
class Spline1dConstraint {
 public:
  Spline1dConstraint() = default;
  Spline1dConstraint(const std::vector<double>& x_knots, const uint32_t order);

  bool AddConstraint(const Eigen::MatrixXd constraint,
                     const std::vector<double>& lower_bound,
                     const std::vector<double>& upper_bound);

  bool AddBoundary(const std::vector<double>& x,
                   const std::vector<double>& lower_bound,
                   const std::vector<double>& upper_bound);

  bool AddDerivativeBoundary(const std::vector<double>& x,
                             const std::vector<double>& lower_bound,
                             const std::vector<double>& upper_bound);

  bool AddSecondDerivativeBoundary(const std::vector<double>& x,
                                   const std::vector<double>& lower_bound,
                                   const std::vector<double>& upper_bound);

  bool AddThirdDerivativeBoundary(const std::vector<double>& x,
                                  const std::vector<double>& lower_bound,
                                  const std::vector<double>& upper_bound);

  bool AddPointConstraint(const double x, const double fx);
  bool AddPointDerivativeConstraint(const double x, const double dfx);
  bool AddPointSecondDerivativeConstraint(const double x, const double ddfx);
  bool AddPointThirdDerivativeConstraint(const double x, const double dddfx);

  bool AddSmoothConstraint();
  bool AddDerivativeSmoothConstraint();
  bool AddSecondDerivativeSmoothConstraint();
  bool AddThirdDerivativeSmoothConstraint();

  const AffineConstraint& affine_constraint() const;
  const Eigen::MatrixXd& constraint_matrix() const;
  const std::vector<double>& lower_bound() const;
  const std::vector<double>& upper_bound() const;

 private:
  size_t FindSegStartIndex(const double x) const;

 private:
  AffineConstraint affine_constraint_;
  std::vector<double> x_knots_;

  uint32_t spline_order_ = 0;
  uint32_t spline_param_num_ = 0;
  uint32_t columns_;
};
}  // namespace common
