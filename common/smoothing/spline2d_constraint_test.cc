/* Copyright 2019 Unity-Drive Inc. All rights reserved */
/*     Author: ChengJie (chengjie@unity-drive.com)     */

#include <gtest/gtest.h>

#include "common/smoothing/spline2d_constraint.h"

namespace common {

using Vec2d = Eigen::Vector2d;

TEST(Spline2dConstraint, add_boundary) {
  std::vector<double> t_knots = {0.0, 1.0};
  int32_t spline_order = 5;
  Spline2dConstraint constraint(t_knots, spline_order);

  std::vector<double> x_coord = {0.0, 0.5, 1.0};
  std::vector<Vec2d> lower_bound = {Vec2d(1.0, 1.0), Vec2d(1.0, 1.0),
                                    Vec2d(1.0, 1.0)};
  std::vector<Vec2d> upper_bound = {Vec2d(5.0, 5.0), Vec2d(5.0, 5.0),
                                    Vec2d(5.0, 5.0)};

  constraint.Add2dBoundary(x_coord, lower_bound, upper_bound);
  const auto res_mat = constraint.constraint_matrix();
  const auto res_lower_bound = constraint.lower_bound();
  const auto res_upper_bound = constraint.upper_bound();

  // clang-format off
  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(6, 12);
  ref_mat <<
       1,        0,        0,        0,        0,        0,        0,        0,        0,        0,        0,        0,  //NOLINT 
       1,      0.5,     0.25,    0.125,   0.0625,  0.03125,        0,        0,        0,        0,        0,        0,  //NOLINT
       1,        1,        1,        1,        1,        1,        0,        0,        0,        0,        0,        0,  //NOLINT
       0,        0,        0,        0,        0,        0,        1,        0,        0,        0,        0,        0,  //NOLINT
       0,        0,        0,        0,        0,        0,        1,      0.5,     0.25,    0.125,   0.0625,  0.03125,  //NOLINT
       0,        0,        0,        0,        0,        0,        1,        1,        1,        1,        1,        1;  //NOLINT
  // clang-format on

  for (size_t row = 0; row < res_mat.rows(); ++row) {
    for (size_t col = 0; col < res_mat.cols(); ++col) {
      EXPECT_DOUBLE_EQ(res_mat(row, col), ref_mat(row, col));
    }
  }

  for (size_t i = 0; i < lower_bound.size(); ++i) {
    EXPECT_DOUBLE_EQ(lower_bound[i].x(), res_lower_bound[i]);
    EXPECT_DOUBLE_EQ(upper_bound[i].x(), res_upper_bound[i]);
    EXPECT_DOUBLE_EQ(lower_bound[i].y(),
                     res_lower_bound[i + lower_bound.size()]);
    EXPECT_DOUBLE_EQ(upper_bound[i].y(),
                     res_upper_bound[i + lower_bound.size()]);
  }
}

TEST(Spline2dConstraint, add_sl_boundary) {
  std::vector<double> x_knots = {0.0, 1.0};
  int32_t spline_order = 3;
  Spline2dConstraint constraint(x_knots, spline_order);

  std::vector<double> t_coord = {0.0};
  std::vector<double> angle = {0.2};
  vector_Eigen<Eigen::Vector2d> ref_point;
  ref_point.emplace_back(Eigen::Vector2d(1.0, 1.0));
  std::vector<double> lateral_bound = {1.0};
  std::vector<double> longitidinal_bound = {2.0};

  constraint.Add2dStationLateralBoundary(t_coord, ref_point, angle,
                                         longitidinal_bound, lateral_bound);
  const auto mat = constraint.constraint_matrix();

  std::cout << mat << std::endl;
  const auto lower_bound = constraint.lower_bound();
  const auto upper_bound = constraint.upper_bound();

  // clang-format off
  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(2, 8);
  ref_mat <<
    -0.198669,    -0,    -0,    -0,  0.980067,     0,     0,     0,
     0.980067,     0,     0,     0,  0.198669,     0,     0,     0;
  // clang-format on

  for (int i = 0; i < mat.rows(); ++i) {
    for (int j = 0; j < mat.cols(); ++j) {
      EXPECT_NEAR(mat(i, j), ref_mat(i, j), 1e-4);
    }
  }

  for (const auto& value : lower_bound) {
    std::cout << "lower_bound: " << value << std::endl;
  }

  for (const auto& value : upper_bound) {
    std::cout << "upper_bound: " << value << std::endl;
  }
}

TEST(Spline2dConstraint, add_derivative_boundary) {
  std::vector<double> t_knots = {0.0, 1.0};
  int32_t spline_order = 5;
  Spline2dConstraint constraint(t_knots, spline_order);

  std::vector<double> x_coord = {0.0, 0.5, 1.0};
  std::vector<Vec2d> lower_bound = {Vec2d(1.0, 1.0), Vec2d(1.0, 1.0),
                                    Vec2d(1.0, 1.0)};
  std::vector<Vec2d> upper_bound = {Vec2d(5.0, 5.0), Vec2d(5.0, 5.0),
                                    Vec2d(5.0, 5.0)};

  constraint.Add2dDerivativeBoundary(x_coord, lower_bound, upper_bound);
  const auto res_mat = constraint.constraint_matrix();
  const auto res_lower_bound = constraint.lower_bound();
  const auto res_upper_bound = constraint.upper_bound();

  // clang-format off
  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(6, 12);
  ref_mat <<
       0 ,       1,        0,        0,        0,        0,        0,        0,        0,        0,        0,        0,  //NOLINT 
       0 ,       1,        1,     0.75,      0.5,   0.3125,        0,        0,        0,        0,        0,        0,  //NOLINT
       0 ,       1,        2,        3,        4,        5,        0,        0,        0,        0,        0,        0,  //NOLINT
       0,        0,        0,        0,        0,        0,        0 ,       1,        0,        0,        0,        0,  //NOLINT
       0,        0,        0,        0,        0,        0,        0 ,       1,        1,     0.75,      0.5,   0.3125,  //NOLINT
       0,        0,        0,        0,        0,        0,        0 ,       1,        2,        3,        4,        5;  //NOLINT
  // clang-format on

  for (size_t row = 0; row < res_mat.rows(); ++row) {
    for (size_t col = 0; col < res_mat.cols(); ++col) {
      EXPECT_DOUBLE_EQ(res_mat(row, col), ref_mat(row, col));
    }
  }

  for (size_t i = 0; i < lower_bound.size(); ++i) {
    EXPECT_DOUBLE_EQ(lower_bound[i].x(), res_lower_bound[i]);
    EXPECT_DOUBLE_EQ(upper_bound[i].x(), res_upper_bound[i]);
    EXPECT_DOUBLE_EQ(lower_bound[i].y(),
                     res_lower_bound[i + lower_bound.size()]);
    EXPECT_DOUBLE_EQ(upper_bound[i].y(),
                     res_upper_bound[i + lower_bound.size()]);
  }
}

TEST(Spline2dConstraint, add_second_derivative_boundary) {
  std::vector<double> t_knots = {0.0, 1.0};
  int32_t spline_order = 5;
  Spline2dConstraint constraint(t_knots, spline_order);

  std::vector<double> x_coord = {0.0, 0.5, 1.0};
  std::vector<Vec2d> lower_bound = {Vec2d(1.0, 1.0), Vec2d(1.0, 1.0),
                                    Vec2d(1.0, 1.0)};
  std::vector<Vec2d> upper_bound = {Vec2d(5.0, 5.0), Vec2d(5.0, 5.0),
                                    Vec2d(5.0, 5.0)};

  constraint.Add2dSecondDerivativeBoundary(x_coord, lower_bound, upper_bound);
  const auto res_mat = constraint.constraint_matrix();
  const auto res_lower_bound = constraint.lower_bound();
  const auto res_upper_bound = constraint.upper_bound();

  // clang-format off
  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(6, 12);
  ref_mat <<
       0,    0,    2,    0,    0,    0,    0,    0,    0,    0,    0,    0,  //NOLINT 
       0,    0,    2,    3,    3,  2.5,    0,    0,    0,    0,    0,    0,  //NOLINT
       0,    0,    2,    6,   12,   20,    0,    0,    0,    0,    0,    0,  //NOLINT
       0,    0,    0,    0,    0,    0,    0,    0,    2,    0,    0,    0,  //NOLINT
       0,    0,    0,    0,    0,    0,    0,    0,    2,    3,    3,  2.5,  //NOLINT
       0,    0,    0,    0,    0,    0,    0,    0,    2,    6,   12,   20;  //NOLINT
  // clang-format on

  for (size_t row = 0; row < res_mat.rows(); ++row) {
    for (size_t col = 0; col < res_mat.cols(); ++col) {
      EXPECT_DOUBLE_EQ(res_mat(row, col), ref_mat(row, col));
    }
  }

  for (size_t i = 0; i < lower_bound.size(); ++i) {
    EXPECT_DOUBLE_EQ(lower_bound[i].x(), res_lower_bound[i]);
    EXPECT_DOUBLE_EQ(upper_bound[i].x(), res_upper_bound[i]);
    EXPECT_DOUBLE_EQ(lower_bound[i].y(),
                     res_lower_bound[i + lower_bound.size()]);
    EXPECT_DOUBLE_EQ(upper_bound[i].y(),
                     res_upper_bound[i + lower_bound.size()]);
  }
}

TEST(Spline2dConstraint, add_third_derivative_boundary) {
  std::vector<double> t_knots = {0.0, 1.0};
  int32_t spline_order = 5;
  Spline2dConstraint constraint(t_knots, spline_order);

  std::vector<double> x_coord = {0.0, 0.5, 1.0};
  std::vector<Vec2d> lower_bound = {Vec2d(1.0, 1.0), Vec2d(1.0, 1.0),
                                    Vec2d(1.0, 1.0)};
  std::vector<Vec2d> upper_bound = {Vec2d(5.0, 5.0), Vec2d(5.0, 5.0),
                                    Vec2d(5.0, 5.0)};

  constraint.Add2dThirdDerivativeBoundary(x_coord, lower_bound, upper_bound);
  const auto res_mat = constraint.constraint_matrix();
  const auto res_lower_bound = constraint.lower_bound();
  const auto res_upper_bound = constraint.upper_bound();

  // clang-format off
  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(6, 12);
  ref_mat <<
       0,    0,    0,    6,   0,     0,    0,    0,    0,    0,    0,    0,  //NOLINT 
       0,    0,    0,    6,   12,   15,    0,    0,    0,    0,    0,    0,  //NOLINT
       0,    0,    0,    6,   24,   60,    0,    0,    0,    0,    0,    0,  //NOLINT
       0,    0,    0,    0,    0,    0,    0,    0,    0,    6,   0,     0,  //NOLINT
       0,    0,    0,    0,    0,    0,    0,    0,    0,    6,   12,   15,  //NOLINT
       0,    0,    0,    0,    0,    0,    0,    0,    0,    6,   24,   60;  //NOLINT
  // clang-format on

  for (size_t row = 0; row < res_mat.rows(); ++row) {
    for (size_t col = 0; col < res_mat.cols(); ++col) {
      EXPECT_DOUBLE_EQ(res_mat(row, col), ref_mat(row, col));
    }
  }

  for (size_t i = 0; i < lower_bound.size(); ++i) {
    EXPECT_DOUBLE_EQ(lower_bound[i].x(), res_lower_bound[i]);
    EXPECT_DOUBLE_EQ(upper_bound[i].x(), res_upper_bound[i]);
    EXPECT_DOUBLE_EQ(lower_bound[i].y(),
                     res_lower_bound[i + lower_bound.size()]);
    EXPECT_DOUBLE_EQ(upper_bound[i].y(),
                     res_upper_bound[i + lower_bound.size()]);
  }
}

TEST(Spline2dConstraint, add_smooth_constraint_01) {
  std::vector<double> t_knots = {0.0, 1.0, 2.0};
  int32_t spline_order = 5;
  Spline2dConstraint constraint(t_knots, spline_order);

  constraint.Add2dSmoothConstraint();
  const auto res_mat = constraint.constraint_matrix();
  const auto res_lower_bound = constraint.lower_bound();
  const auto res_upper_bound = constraint.upper_bound();

  // clang-format off
  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(2, 24);
  ref_mat << 1, 1, 1, 1, 1, 1, -1, -0, -0, -0, -0, -0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  //NOLINT
             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, -1, -0, -0, -0, -0, -0;  //NOLINT
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

TEST(Spline2dConstraint, add_smooth_constraint_02) {
  std::vector<double> t_knots = {0.0, 1.0, 2.0, 3.0};
  int32_t spline_order = 5;
  Spline2dConstraint constraint(t_knots, spline_order);

  constraint.Add2dSmoothConstraint();
  const auto res_mat = constraint.constraint_matrix();
  const auto res_lower_bound = constraint.lower_bound();
  const auto res_upper_bound = constraint.upper_bound();

  // clang-format off
  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(4, 3*6*2);
  ref_mat <<
      1, 1, 1, 1, 1, 1, -1, -0, -0, -0, -0, -0, 0, 0, 0, 0, 0, 0,    //NOLINT 
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,        //NOLINT

      0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, -1, -0, -0, -0, -0, -0,  //NOLINT
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,        //NOLINT

      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,        //NOLINT
      1, 1, 1, 1, 1, 1, -1, -0, -0, -0, -0, -0, 0, 0, 0, 0, 0, 0,    //NOLINT 

      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,        //NOLINT
      0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, -1, -0, -0, -0, -0, -0;  //NOLINT
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

TEST(Spline2dConstraint, add_point_angle_constraint_01) {
  std::vector<double> t_knots = {0.0, 1.0};
  uint32_t spline_order = 3;
  Spline2dConstraint constraint(t_knots, spline_order);

  double t_coord = 0.0;
  double angle = 45.0 * M_PI / 180.0;

  constraint.AddPointAngleConstraint(t_coord, angle);

  const auto mat = constraint.constraint_matrix();
  const auto lower_bound = constraint.lower_bound();
  const auto upper_bound = constraint.upper_bound();

  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(3, 8);
  // clang-format off
  ref_mat <<
    0,        1, 0, 0, 0,         0,  0,  0,
    0,        0, 0, 0, 0,         1,  0,  0,
    0, 0.707107, 0, 0, 0, -0.707107, -0, -0;
  // clang-format on

  for (int i = 0; i < mat.rows(); ++i) {
    for (int j = 0; j < mat.cols(); ++j) {
      EXPECT_NEAR(mat(i, j), ref_mat(i, j), 1e-4);
    }
  }

  for (int i = 0; i < 2; ++i) {
    EXPECT_NEAR(lower_bound[i], 0.0, 1e-4);
    EXPECT_NEAR(upper_bound[i], udrive_inf, 1e-4);
  }

  EXPECT_NEAR(lower_bound[2], 0.0, 1e-4);
  EXPECT_NEAR(upper_bound[2], 0.0, 1e-4);
}

TEST(Spline2dConstraint, add_point_angle_constraint_02) {
  std::vector<double> t_knots = {0.0, 1.0};
  uint32_t spline_order = 3;
  Spline2dConstraint constraint(t_knots, spline_order);

  double t_coord = 0.0;
  double angle = 0.0 * M_PI / 180.0;

  constraint.AddPointAngleConstraint(t_coord, angle);

  const auto mat = constraint.constraint_matrix();
  const auto lower_bound = constraint.lower_bound();
  const auto upper_bound = constraint.upper_bound();

  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(3, 8);
  // clang-format off
  ref_mat <<
    0, 1, 0, 0, 0,  0,  0,  0,
    0, 0, 0, 0, 0,  1,  0,  0,
    0, 0, 0, 0, 0, -1, -0, -0;
  // clang-format on

  for (int i = 0; i < mat.rows(); ++i) {
    for (int j = 0; j < mat.cols(); ++j) {
      EXPECT_NEAR(mat(i, j), ref_mat(i, j), 1e-4);
    }
  }

  for (int i = 0; i < 2; ++i) {
    EXPECT_NEAR(lower_bound[i], 0.0, 1e-4);
    EXPECT_NEAR(upper_bound[i], udrive_inf, 1e-4);
  }

  EXPECT_NEAR(lower_bound[2], 0.0, 1e-4);
  EXPECT_NEAR(upper_bound[2], 0.0, 1e-4);
}

TEST(Spline2dConstraint, add_point_angle_constraint_03) {
  std::vector<double> t_knots = {0.0, 1.0};
  uint32_t spline_order = 3;
  Spline2dConstraint constraint(t_knots, spline_order);

  double t_coord = 0.0;
  double angle = 135.0 * M_PI / 180.0;

  constraint.AddPointAngleConstraint(t_coord, angle);

  const auto mat = constraint.constraint_matrix();
  const auto lower_bound = constraint.lower_bound();
  const auto upper_bound = constraint.upper_bound();

  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(3, 8);
  // clang-format off
  ref_mat <<
    0,       -1, 0, 0, 0,         0,  0,  0,
    0,        0, 0, 0, 0,         1,  0,  0,
    0, 0.707107, 0, 0, 0,  0.707107,  0,  0;
  // clang-format on

  for (int i = 0; i < mat.rows(); ++i) {
    for (int j = 0; j < mat.cols(); ++j) {
      EXPECT_NEAR(mat(i, j), ref_mat(i, j), 1e-4);
    }
  }

  for (int i = 0; i < 2; ++i) {
    EXPECT_NEAR(lower_bound[i], 0.0, 1e-4);
    EXPECT_NEAR(upper_bound[i], udrive_inf, 1e-4);
  }

  EXPECT_NEAR(lower_bound[2], 0.0, 1e-4);
  EXPECT_NEAR(upper_bound[2], 0.0, 1e-4);
}

TEST(Spline2dConstraint, add_point_angle_constraint_04) {
  std::vector<double> t_knots = {0.0, 1.0};
  uint32_t spline_order = 3;
  Spline2dConstraint constraint(t_knots, spline_order);

  double t_coord = 1.0;
  double angle = 60 * M_PI / 180.0;

  constraint.AddPointAngleConstraint(t_coord, angle);

  const auto mat = constraint.constraint_matrix();
  const auto lower_bound = constraint.lower_bound();
  const auto upper_bound = constraint.upper_bound();

  Eigen::MatrixXd ref_mat = Eigen::MatrixXd::Zero(3, 8);
  // clang-format off
  ref_mat <<
    0,        1,       2,       3, 0,    0,  0,    0,
    0,        0,       0,       0, 0,    1,  2,    3,
    0, 0.866025, 1.73205, 2.59808, 0, -0.5, -1, -1.5;
  // clang-format on

  for (int i = 0; i < mat.rows(); ++i) {
    for (int j = 0; j < mat.cols(); ++j) {
      EXPECT_NEAR(mat(i, j), ref_mat(i, j), 1e-4);
    }
  }

  for (int i = 0; i < 2; ++i) {
    EXPECT_NEAR(lower_bound[i], 0.0, 1e-4);
    EXPECT_NEAR(upper_bound[i], udrive_inf, 1e-4);
  }

  EXPECT_NEAR(lower_bound[2], 0.0, 1e-4);
  EXPECT_NEAR(upper_bound[2], 0.0, 1e-4);
}
}  // namespace common
