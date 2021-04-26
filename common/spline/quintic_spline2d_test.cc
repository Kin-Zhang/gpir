/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include "common/spline/quintic_spline2d.h"

#include <gtest/gtest.h>

namespace common {

TEST(QuinticSpline2DTest, PolyFitCircle) {
  vector_Eigen<Eigen::Vector2d> refs;
  std::vector<double> paras;

  const double radius = 10;
  double angle = 0.0;

  while (angle < M_PI) {
    double x = std::cos(angle) * radius;
    double y = std::sin(angle) * radius;
    double arc = angle * radius;
    angle += 20.0 / 180.0 * M_PI;
    refs.emplace_back(Eigen::Vector2d(x, y));
    paras.emplace_back(arc);
  }
  std::vector<double> knots{paras.front(), paras.back()};

  QuinticSpline2D qs;
  EXPECT_TRUE(qs.PolyFit(knots, refs, paras));

  std::vector<double> x, y, x_ref, y_ref;
  const double kappa_ground_truth = 1 / radius;
  for (int i = 0; i < paras.size(); ++i) {
    double kappa = qs.kappa(paras[i]);
    double dkappa = qs.dkappa(paras[i]);
    EXPECT_NEAR(kappa, kappa_ground_truth, 0.015);
    EXPECT_NEAR(dkappa, 0.0, 0.015);
  }
}

}  // namespace common
