/* Copyright 2019 Unity-Drive Inc. All rights reserved */
/*     Author: ChengJie (chengjie@unity-drive.com)     */

#include <gtest/gtest.h>

#include "common/smoothing/spline1d_constraint.h"

namespace common {

TEST(Spline1dConstraint, add_boundary) {
  std::vector<double> x_knots = {0.0, 1.0};
  int32_t spline_order = 5;
  Spline1dConstraint constraint(x_knots, spline_order);

  std::vector<double> x_coord = {0.0, 0.5, 1.0};
  std::vector<double> lower_bound = {1.0, 1.0, 1.0};
  std::vector<double> upper_bound = {5.0, 5.0, 5.0};

  constraint.AddBoundary(x_coord, lower_bound, upper_bound);
  const auto res_mat = constraint.affine_constraint().constraint_matrix();
  const auto res_lower_bound = constraint.affine_constraint().lower_bound();
  const auto res_upper_bound = constraint.affine_constraint().upper_bound();

  // clang-format off
  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(3, 6);
  ref_mat <<
       1,        0,        0,        0,        0,        0,
       1,      0.5,     0.25,    0.125,   0.0625,  0.03125,
       1,        1,        1,        1,        1,        1;
  // clang-format on

  for (size_t row = 0; row < res_mat.rows(); ++row) {
    for (size_t col = 0; col < res_mat.cols(); ++col) {
      EXPECT_DOUBLE_EQ(res_mat(row, col), ref_mat(row, col));
    }
  }

  for (size_t i = 0; i < lower_bound.size(); ++i) {
    EXPECT_DOUBLE_EQ(lower_bound[i], res_lower_bound[i]);
    EXPECT_DOUBLE_EQ(upper_bound[i], res_upper_bound[i]);
  }
}

TEST(Spline1dConstraint, add_derivative_boundary) {
  std::vector<double> x_knots = {0.0, 1.0};
  int32_t spline_order = 5;
  Spline1dConstraint constraint(x_knots, spline_order);

  std::vector<double> x_coord = {0.0, 0.5, 1.0};
  std::vector<double> lower_bound = {1.0, 1.0, 1.0};
  std::vector<double> upper_bound = {5.0, 5.0, 5.0};

  constraint.AddDerivativeBoundary(x_coord, lower_bound, upper_bound);
  const auto res_mat = constraint.affine_constraint().constraint_matrix();
  const auto res_lower_bound = constraint.affine_constraint().lower_bound();
  const auto res_upper_bound = constraint.affine_constraint().upper_bound();

  // clang-format off
  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(3, 6);
  ref_mat <<
      0,       1,       0,       0,       0,       0,
      0,       1,       1,    0.75,     0.5,  0.3125,
      0,       1,       2,       3,       4,       5;
  // clang-format on

  for (size_t row = 0; row < res_mat.rows(); ++row) {
    for (size_t col = 0; col < res_mat.cols(); ++col) {
      EXPECT_DOUBLE_EQ(res_mat(row, col), ref_mat(row, col));
    }
  }

  for (size_t i = 0; i < lower_bound.size(); ++i) {
    EXPECT_DOUBLE_EQ(lower_bound[i], res_lower_bound[i]);
    EXPECT_DOUBLE_EQ(upper_bound[i], res_upper_bound[i]);
  }
}

TEST(Spline1dConstraint, add_second_derivative_boundary) {
  std::vector<double> x_knots = {0.0, 1.0};
  int32_t spline_order = 5;
  Spline1dConstraint constraint(x_knots, spline_order);

  std::vector<double> x_coord = {0.0, 0.5, 1.0};
  std::vector<double> lower_bound = {1.0, 1.0, 1.0};
  std::vector<double> upper_bound = {5.0, 5.0, 5.0};

  constraint.AddSecondDerivativeBoundary(x_coord, lower_bound, upper_bound);
  const auto res_mat = constraint.affine_constraint().constraint_matrix();
  const auto res_lower_bound = constraint.affine_constraint().lower_bound();
  const auto res_upper_bound = constraint.affine_constraint().upper_bound();

  // clang-format off
  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(3, 6);
  ref_mat <<
      0,    0,    2,    0,    0,    0,
      0,    0,    2,    3,    3,  2.5,
      0,    0,    2,    6,   12,   20;
  // clang-format on

  for (size_t row = 0; row < res_mat.rows(); ++row) {
    for (size_t col = 0; col < res_mat.cols(); ++col) {
      EXPECT_DOUBLE_EQ(res_mat(row, col), ref_mat(row, col));
    }
  }

  for (size_t i = 0; i < lower_bound.size(); ++i) {
    EXPECT_DOUBLE_EQ(lower_bound[i], res_lower_bound[i]);
    EXPECT_DOUBLE_EQ(upper_bound[i], res_upper_bound[i]);
  }
}

TEST(Spline1dConstraint, add_third_derivative_boundary) {
  std::vector<double> x_knots = {0.0, 1.0};
  int32_t spline_order = 5;
  Spline1dConstraint constraint(x_knots, spline_order);

  std::vector<double> x_coord = {0.0, 0.5, 1.0};
  std::vector<double> lower_bound = {1.0, 1.0, 1.0};
  std::vector<double> upper_bound = {5.0, 5.0, 5.0};

  constraint.AddThirdDerivativeBoundary(x_coord, lower_bound, upper_bound);
  const auto res_mat = constraint.affine_constraint().constraint_matrix();
  const auto res_lower_bound = constraint.affine_constraint().lower_bound();
  const auto res_upper_bound = constraint.affine_constraint().upper_bound();

  // clang-format off
  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(3, 6);
  ref_mat <<
      0,    0,    0,    6,   0,     0,
      0,    0,    0,    6,   12,   15,
      0,    0,    0,    6,   24,   60;
  // clang-format on

  for (size_t row = 0; row < res_mat.rows(); ++row) {
    for (size_t col = 0; col < res_mat.cols(); ++col) {
      EXPECT_DOUBLE_EQ(res_mat(row, col), ref_mat(row, col));
    }
  }

  for (size_t i = 0; i < lower_bound.size(); ++i) {
    EXPECT_DOUBLE_EQ(lower_bound[i], res_lower_bound[i]);
    EXPECT_DOUBLE_EQ(upper_bound[i], res_upper_bound[i]);
  }
}

TEST(Spline1dConstraint, add_smooth_constraint_01) {
  std::vector<double> x_knots = {0.0, 1.0, 2.0};
  int32_t spline_order = 5;
  Spline1dConstraint constraint(x_knots, spline_order);

  constraint.AddSmoothConstraint();
  const auto res_mat = constraint.affine_constraint().constraint_matrix();
  const auto res_lower_bound = constraint.affine_constraint().lower_bound();
  const auto res_upper_bound = constraint.affine_constraint().upper_bound();

  // clang-format off
  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(1, 12);
  ref_mat << 1, 1, 1, 1, 1, 1, -1, -0, -0, -0, -0, -0;
  // clang-format on

  for (size_t i = 0; i < res_mat.rows(); ++i) {
    for (size_t j = 0; j < res_mat.cols(); ++j) {
      EXPECT_DOUBLE_EQ(res_mat(i, j), ref_mat(i, j));
    }
  }

  for (size_t i = 0; i < res_upper_bound.size(); ++i) {
    EXPECT_DOUBLE_EQ(res_lower_bound[i], 0.0);
    EXPECT_DOUBLE_EQ(res_upper_bound[i], 0.0);
  }
}

TEST(Spline1dConstraint, add_smooth_constraint_02) {
  std::vector<double> x_knots = {0.0, 1.0, 2.0, 3.0};
  int32_t spline_order = 5;
  Spline1dConstraint constraint(x_knots, spline_order);

  constraint.AddSmoothConstraint();
  const auto res_mat = constraint.affine_constraint().constraint_matrix();
  const auto res_lower_bound = constraint.affine_constraint().lower_bound();
  const auto res_upper_bound = constraint.affine_constraint().upper_bound();

  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(2, 18);
  // clang-format off
  ref_mat <<
     1,  1,  1,  1,  1,  1, -1, -0, -0, -0, -0, -0,  0,  0,  0,  0,  0,  0,
     0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  1, -1, -0, -0, -0, -0, -0;
  // clang-format on

  for (size_t i = 0; i < res_mat.rows(); ++i) {
    for (size_t j = 0; j < res_mat.cols(); ++j) {
      EXPECT_DOUBLE_EQ(res_mat(i, j), ref_mat(i, j));
    }
  }

  for (size_t i = 0; i < res_upper_bound.size(); ++i) {
    EXPECT_DOUBLE_EQ(res_lower_bound[i], 0.0);
    EXPECT_DOUBLE_EQ(res_upper_bound[i], 0.0);
  }
}

TEST(Spline1dConstraint, add_derivative_smooth_constraint) {
  std::vector<double> x_knots = {0.0, 1.0, 2.0, 3.0};
  int32_t spline_order = 3;
  Spline1dConstraint constraint(x_knots, spline_order);

  constraint.AddDerivativeSmoothConstraint();
  const auto res_mat = constraint.affine_constraint().constraint_matrix();
  const auto res_lower_bound = constraint.affine_constraint().lower_bound();
  const auto res_upper_bound = constraint.affine_constraint().upper_bound();

  // clang-format off
  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(4, 12);
  ref_mat <<
      1,  1,  1,  1, -1, -0, -0, -0,  0,  0,  0,  0,
      0,  1,  2,  3,  0, -1, -0, -0,  0,  0,  0,  0,
      0,  0,  0,  0,  1,  1,  1,  1, -1, -0, -0, -0,
      0,  0,  0,  0,  0,  1,  2,  3,  0, -1, -0, -0;
  // clang-format on

  for (size_t i = 0; i < res_mat.rows(); ++i) {
    for (size_t j = 0; j < res_mat.cols(); ++j) {
      EXPECT_DOUBLE_EQ(res_mat(i, j), ref_mat(i, j));
    }
  }

  for (size_t i = 0; i < res_upper_bound.size(); ++i) {
    EXPECT_DOUBLE_EQ(res_lower_bound[i], 0.0);
    EXPECT_DOUBLE_EQ(res_upper_bound[i], 0.0);
  }
}

TEST(Spline1dConstraint, add_second_derivative_smooth_constraint) {
  std::vector<double> x_knots = {0.0, 1.0, 2.0, 3.0};
  int32_t spline_order = 3;
  Spline1dConstraint constraint(x_knots, spline_order);

  constraint.AddSecondDerivativeSmoothConstraint();
  const auto res_mat = constraint.affine_constraint().constraint_matrix();
  const auto res_lower_bound = constraint.affine_constraint().lower_bound();
  const auto res_upper_bound = constraint.affine_constraint().upper_bound();

  // clang-format off
  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(6, 12);
  ref_mat <<
      1,  1,  1,  1, -1, -0, -0, -0,  0,  0,  0,  0,
      0,  1,  2,  3,  0, -1, -0, -0,  0,  0,  0,  0,
      0,  0,  2,  6,  0,  0, -2, -0,  0,  0,  0,  0,
      0,  0,  0,  0,  1,  1,  1,  1, -1, -0, -0, -0,
      0,  0,  0,  0,  0,  1,  2,  3,  0, -1, -0, -0,
      0,  0,  0,  0,  0,  0,  2,  6,  0,  0, -2, -0;
  // clang-format on

  for (size_t i = 0; i < res_mat.rows(); ++i) {
    for (size_t j = 0; j < res_mat.cols(); ++j) {
      EXPECT_DOUBLE_EQ(res_mat(i, j), ref_mat(i, j));
    }
  }

  for (size_t i = 0; i < res_upper_bound.size(); ++i) {
    EXPECT_DOUBLE_EQ(res_lower_bound[i], 0.0);
    EXPECT_DOUBLE_EQ(res_upper_bound[i], 0.0);
  }
}

TEST(Spline1dConstraint, add_third_derivative_smooth_constraint) {
  std::vector<double> x_knots = {0.0, 1.0, 2.0, 3.0};
  int32_t spline_order = 4;
  Spline1dConstraint constraint(x_knots, spline_order);

  constraint.AddThirdDerivativeSmoothConstraint();
  const auto res_mat = constraint.affine_constraint().constraint_matrix();
  const auto res_lower_bound = constraint.affine_constraint().lower_bound();
  const auto res_upper_bound = constraint.affine_constraint().upper_bound();

  // clang-format off
  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(8, 15);
  ref_mat <<
      1,  1,  1,  1,  1, -1, -0, -0, -0, -0,  0,  0,  0,  0,  0,
      0,  1,  2,  3,  4,  0, -1, -0, -0, -0,  0,  0,  0,  0,  0,
      0,  0,  2,  6, 12,  0,  0, -2, -0, -0,  0,  0,  0,  0,  0,
      0,  0,  0,  6, 24,  0,  0,  0, -6, -0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  1,  1,  1,  1,  1, -1, -0, -0, -0, -0,
      0,  0,  0,  0,  0,  0,  1,  2,  3,  4,  0, -1, -0, -0, -0,
      0,  0,  0,  0,  0,  0,  0,  2,  6, 12,  0,  0, -2, -0, -0,
      0,  0,  0,  0,  0,  0,  0,  0,  6, 24,  0,  0,  0, -6, -0;
  // clang-format on

  for (size_t i = 0; i < res_mat.rows(); ++i) {
    for (size_t j = 0; j < res_mat.cols(); ++j) {
      EXPECT_DOUBLE_EQ(res_mat(i, j), ref_mat(i, j));
    }
  }

  for (size_t i = 0; i < res_upper_bound.size(); ++i) {
    EXPECT_DOUBLE_EQ(res_lower_bound[i], 0.0);
    EXPECT_DOUBLE_EQ(res_upper_bound[i], 0.0);
  }
}
}  // namespace common
