/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include "gp_planner/gp/factors/gp_kappa_limit_factor.h"

#include "gp_planner/gp/utils/curvature_utils.h"
#include "gp_planner/gp/utils/gp_utils.h"

namespace planning {

gtsam::Vector GPKappaLimitFactor::evaluateError(
    const gtsam::Vector3& x, boost::optional<gtsam::Matrix&> H) const {
  // double err = 0.0;
  // if (H)
  //   err = GPUtils::HingeKappaLimitLoss(x, kappa_r_, dkappa_r_, kappa_limit_,
  //   H);
  // else
  //   err = GPUtils::HingeKappaLimitLoss(x, kappa_r_, dkappa_r_, kappa_limit_);
  // return gtsam::Vector1(err);
  double gradient = 0.0, kappa = 0.0;
  if (H) {
    kappa = CurvatureUtils::GetKappaAndJacobian(x, kappa_r_, dkappa_r_, H);
  } else {
    kappa = CurvatureUtils::GetKappaAndJacobian(x, kappa_r_, dkappa_r_);
  }
  double error = penalty_.EvaluateHinge(kappa, &gradient);
  if (H) (*H) *= gradient;
  return gtsam::Vector1(error);
}
}  // namespace planning
