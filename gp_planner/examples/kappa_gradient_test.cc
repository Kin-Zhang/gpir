/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include "gp_planner/gp/utils/curvature_utils.h"
#include "gp_planner/gp/utils/penalty_function.h"

using namespace planning;

int main(int argc, char const *argv[]) {
  const double kappa_r = 0.3;
  const double kappa_rd = 0.0;
  PenaltyFunction penalty(0.01);

  Eigen::Vector3d d(1, 1, 0);
  gtsam::Matrix J;
  double kappa = CurvatureUtils::GetKappaAndJacobian(d, kappa_r, kappa_r, J);
  double grad = 0.0;
  double error = penalty.EvaluatePoly(kappa, &grad);
  printf("initial error: %f\n", error);

  for (int i = 0; i < 20; ++i) {
    d -= 0.05 * grad * Eigen::Vector3d(J(0, 0), J(0, 1), J(0, 2));
    double kappa2 =
        CurvatureUtils::GetKappaAndJacobian(d, kappa_r, kappa_rd, J);
    error = penalty.EvaluatePoly(kappa2, &grad);

    printf("error: %f,  %f vs %f\n", error, kappa, kappa2);
    kappa = kappa2;
  }

  return 0;
}
