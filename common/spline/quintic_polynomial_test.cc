/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include "common/spline/quintic_polynomial.h"

#include <gtest/gtest.h>

#include <cmath>

using std::pow;

TEST(QuinticPolynomialTest, basic) {
  std::array<double, 6> coeff{1, 2e-1, 3e-2, 4e-3, 5e-4, 6e-5};
  auto f = [&coeff](const double s) {
    return coeff[0] + coeff[1] * s + coeff[2] * pow(s, 2) +
           coeff[3] * pow(s, 3) + coeff[4] * pow(s, 4) + coeff[5] * pow(s, 5);
  };
  auto df = [&coeff](const double s) {
    return coeff[1] + 2 * coeff[2] * s + 3 * coeff[3] * pow(s, 2) +
           4 * coeff[4] * pow(s, 3) + 5 * coeff[5] * pow(s, 4);
  };
  auto ddf = [&coeff](const double s) {
    return 2 * coeff[2] + 6 * coeff[3] * s + 12 * coeff[4] * pow(s, 2) +
           20 * coeff[5] * pow(s, 3);
  };
  auto dddf = [&coeff](const double s) {
    return 6 * coeff[3] + 24 * coeff[4] * s + 60 * coeff[5] * pow(s, 2);
  };

  common::QuinticPolynomial qp(coeff);
  double s = -50;
  for (int i = 0; i < 10000; ++i) {
    s += 0.01;
    EXPECT_NEAR(qp.f(s), f(s), 1e-5);
    EXPECT_NEAR(qp.df(s), df(s), 1e-5);
    EXPECT_NEAR(qp.ddf(s), ddf(s), 1e-5);
    EXPECT_NEAR(qp.dddf(s), dddf(s), 1e-5);
  }
}