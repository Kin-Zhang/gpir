/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include "gp_planner/gp/factors/gp_lat_acc_limit_factor.h"

namespace planning {

gtsam::Vector GPLatAccLimitFactor::evaluateError(
    const gtsam::Vector3& d1, const gtsam::Vector3& d2,
    boost::optional<gtsam::Matrix&> H1,
    boost::optional<gtsam::Matrix&> H2) const {
  gtsam::Matrix33 J_d1, J_d2;
  auto d = gp_interpolator_.Interpolate(d1, d2, J_d1, J_d2);
  double lat_acc = d[2] * s_dot_sqr_ + d[1] * s_dot_dot_;

  double error = 0.0, grad = 0.0;
  gtsam::Matrix13 J_err = gtsam::Matrix13::Zero();
  J_err(0, 0) = 0.0;
  J_err(0, 1) = s_dot_dot_;
  J_err(0, 2) = s_dot_sqr_;
  error = penalty_.EvaluatePoly(lat_acc, &grad);
  // printf("lat_acc: %f, error %f")

  if (H1 || H2) {
    (*H1) = grad * J_err * J_d1;
    (*H2) = grad * J_err * J_d2;
  }

  return gtsam::Vector1(error);
}
}  // namespace planning
