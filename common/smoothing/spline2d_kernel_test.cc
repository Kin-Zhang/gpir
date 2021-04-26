/* Copyright 2019 Unity-Drive Inc. All rights reserved */
/*     Author: ChengJie (chengjie@unity-drive.com)     */

#include <gtest/gtest.h>

#include "common/smoothing/spline2d_kernel.h"

namespace common {

TEST(Spline2dKernel, add_derivative_kernel_matrix_01) {
  std::vector<double> t_knots = {0.0, 1.0};

  const int32_t spline_order = 5;
  const uint32_t num_params = spline_order + 1;
  const uint32_t single_n = (t_knots.size() - 1) * num_params;

  Spline2dKernel kernel(t_knots, spline_order);
  kernel.Add2dDerivativeKernelMatrix(1.0);
  Eigen::MatrixXd mat = kernel.kernel_matrix();
  Eigen::MatrixXd x_mat = mat.block(0, 0, single_n, single_n);
  Eigen::MatrixXd y_mat = mat.block(single_n, single_n, single_n, single_n);

  EXPECT_EQ(mat.rows(), mat.cols());
  EXPECT_EQ(mat.rows(), 2 * num_params * (t_knots.size() - 1));
  Eigen::MatrixXd ref_kernel_matrix = Eigen::MatrixXd::Zero(6, 6);

  // clang-format off
  ref_kernel_matrix <<
      0,       0,       0,       0,       0,       0,
      0,       1,       1,       1,       1,       1,
      0,       1, 1.33333,     1.5,     1.6, 1.66667,
      0,       1,     1.5,     1.8,       2, 2.14286,
      0,       1,     1.6,       2, 2.28571,     2.5,
      0,       1, 1.66667, 2.14286,     2.5, 2.77778;
  // clang-format on
  ref_kernel_matrix *= 2.0;

  for (int i = 0; i < x_mat.rows(); ++i) {
    for (int j = 0; j < x_mat.cols(); ++j) {
      EXPECT_NEAR(x_mat(i, j), ref_kernel_matrix(i, j), 1e-5);
      EXPECT_NEAR(y_mat(i, j), ref_kernel_matrix(i, j), 1e-5);
    }
  }

  Eigen::MatrixXd ref_g = Eigen::MatrixXd::Zero(2 * single_n, 1);
  Eigen::MatrixXd gradient = kernel.gradient();

  for (int i = 0; i < gradient.rows(); ++i) {
    for (int j = 0; j < gradient.cols(); ++j) {
      EXPECT_DOUBLE_EQ(kernel.gradient()(i, j), ref_g(i, j));
    }
  }
}

TEST(Spline2dKernel, add_derivative_kernel_matrix_02) {
  std::vector<double> t_knots = {0.0, 1.0, 2.0};

  const int32_t spline_order = 5;
  const uint32_t num_params = spline_order + 1;
  const uint32_t single_n = (t_knots.size() - 1) * num_params;

  Spline2dKernel kernel(t_knots, spline_order);
  kernel.Add2dDerivativeKernelMatrix(1.0);
  Eigen::MatrixXd mat = kernel.kernel_matrix();
  Eigen::MatrixXd x_mat = mat.block(0, 0, single_n, single_n);
  Eigen::MatrixXd y_mat = mat.block(single_n, single_n, single_n, single_n);

  EXPECT_EQ(mat.rows(), mat.cols());
  EXPECT_EQ(mat.rows(), 2 * num_params * (t_knots.size() - 1));
  Eigen::MatrixXd ref_kernel_matrix = Eigen::MatrixXd::Zero(single_n, single_n);

  // clang-format off
  ref_kernel_matrix <<
      0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,  // NOLINT
      0,       1,       1,       1,       1,       1,       0,       0,       0,       0,       0,       0,  // NOLINT
      0,       1, 1.33333,     1.5,     1.6, 1.66667,       0,       0,       0,       0,       0,       0,  // NOLINT
      0,       1,     1.5,     1.8,       2, 2.14286,       0,       0,       0,       0,       0,       0,  // NOLINT
      0,       1,     1.6,       2, 2.28571,     2.5,       0,       0,       0,       0,       0,       0,  // NOLINT
      0,       1, 1.66667, 2.14286,     2.5, 2.77778,       0,       0,       0,       0,       0,       0,  // NOLINT
      0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,       0,  // NOLINT
      0,       0,       0,       0,       0,       0,       0,       1,       1,       1,       1,       1,  // NOLINT
      0,       0,       0,       0,       0,       0,       0,       1, 1.33333,     1.5,     1.6, 1.66667,  // NOLINT
      0,       0,       0,       0,       0,       0,       0,       1,     1.5,     1.8,       2, 2.14286,  // NOLINT
      0,       0,       0,       0,       0,       0,       0,       1,     1.6,       2, 2.28571,     2.5,  // NOLINT
      0,       0,       0,       0,       0,       0,       0,       1, 1.66667, 2.14286,     2.5, 2.77778;  // NOLINT
  // clang-format on
  ref_kernel_matrix *= 2.0;

  for (int i = 0; i < x_mat.rows(); ++i) {
    for (int j = 0; j < x_mat.cols(); ++j) {
      EXPECT_NEAR(x_mat(i, j), ref_kernel_matrix(i, j), 1e-5);
      EXPECT_NEAR(y_mat(i, j), ref_kernel_matrix(i, j), 1e-5);
    }
  }

  Eigen::MatrixXd ref_g = Eigen::MatrixXd::Zero(2 * single_n, 1);
  Eigen::MatrixXd gradient = kernel.gradient();

  for (int i = 0; i < gradient.rows(); ++i) {
    for (int j = 0; j < gradient.cols(); ++j) {
      EXPECT_DOUBLE_EQ(kernel.gradient()(i, j), ref_g(i, j));
    }
  }
}

TEST(Spline2dKernel, add_derivative_kernel_matrix_03) {
  std::vector<double> t_knots = {0.0, 0.5};

  const int32_t spline_order = 5;
  const uint32_t num_params = spline_order + 1;
  const uint32_t single_n = (t_knots.size() - 1) * num_params;

  Spline2dKernel kernel(t_knots, spline_order);
  kernel.Add2dDerivativeKernelMatrix(1.0);
  Eigen::MatrixXd mat = kernel.kernel_matrix();
  Eigen::MatrixXd x_mat = mat.block(0, 0, single_n, single_n);
  Eigen::MatrixXd y_mat = mat.block(single_n, single_n, single_n, single_n);

  EXPECT_EQ(mat.rows(), mat.cols());
  EXPECT_EQ(mat.rows(), 2 * num_params * (t_knots.size() - 1));
  Eigen::MatrixXd ref_kernel_matrix = Eigen::MatrixXd::Zero(single_n, single_n);

  // clang-format off
  ref_kernel_matrix <<
      0,       0,       0,       0,       0,       0,
      0,       1,       1,       1,       1,       1,
      0,       1, 1.33333,     1.5,     1.6, 1.66667,
      0,       1,     1.5,     1.8,       2, 2.14286,
      0,       1,     1.6,       2, 2.28571,     2.5,
      0,       1, 1.66667, 2.14286,     2.5, 2.77778;
  // clang-format on
  ref_kernel_matrix *= 2.0;

  for (int i = 0; i < x_mat.rows(); ++i) {
    for (int j = 0; j < x_mat.cols(); ++j) {
      double param = std::pow(0.5, i + j - 1);
      EXPECT_NEAR(x_mat(i, j), param * ref_kernel_matrix(i, j), 1e-5);
      EXPECT_NEAR(y_mat(i, j), param * ref_kernel_matrix(i, j), 1e-5);
    }
  }

  Eigen::MatrixXd ref_g = Eigen::MatrixXd::Zero(2 * single_n, 1);
  Eigen::MatrixXd gradient = kernel.gradient();

  for (int i = 0; i < gradient.rows(); ++i) {
    for (int j = 0; j < gradient.cols(); ++j) {
      EXPECT_DOUBLE_EQ(kernel.gradient()(i, j), ref_g(i, j));
    }
  }
}

TEST(Spline2dKernel, add_second_derivative_kernel_matrix_01) {
  std::vector<double> t_knots = {0.0, 0.5};

  const int32_t spline_order = 5;
  const uint32_t num_params = spline_order + 1;
  const uint32_t single_n = (t_knots.size() - 1) * num_params;

  Spline2dKernel kernel(t_knots, spline_order);
  kernel.Add2dSecondOrderDerivativeMatrix(1.0);
  Eigen::MatrixXd mat = kernel.kernel_matrix();
  Eigen::MatrixXd x_mat = mat.block(0, 0, single_n, single_n);
  Eigen::MatrixXd y_mat = mat.block(single_n, single_n, single_n, single_n);

  EXPECT_EQ(mat.rows(), mat.cols());
  EXPECT_EQ(mat.rows(), 2 * num_params * (t_knots.size() - 1));
  Eigen::MatrixXd ref_kernel_matrix = Eigen::MatrixXd::Zero(single_n, single_n);

  // clang-format off
  ref_kernel_matrix <<
      0,       0,       0,       0,       0,       0,
      0,       0,       0,       0,       0,       0,
      0,       0,       4,       6,       8,      10,
      0,       0,       6,      12,      18,      24,
      0,       0,       8,      18,    28.8,      40,
      0,       0,      10,      24,      40, 57.1429;
  // clang-format on
  ref_kernel_matrix *= 2.0;

  for (int i = 0; i < x_mat.rows(); ++i) {
    for (int j = 0; j < x_mat.cols(); ++j) {
      const double param = std::pow(0.5, std::max(0, i + j - 3));
      EXPECT_NEAR(x_mat(i, j), param * ref_kernel_matrix(i, j), 1e-5);
      EXPECT_NEAR(y_mat(i, j), param * ref_kernel_matrix(i, j), 1e-5);
    }
  }

  Eigen::MatrixXd ref_g = Eigen::MatrixXd::Zero(2 * single_n, 1);
  Eigen::MatrixXd gradient = kernel.gradient();

  for (int i = 0; i < gradient.rows(); ++i) {
    for (int j = 0; j < gradient.cols(); ++j) {
      EXPECT_DOUBLE_EQ(kernel.gradient()(i, j), ref_g(i, j));
    }
  }
}

TEST(Spline2dKernel, add_second_derivative_kernel_matrix_02) {
  std::vector<double> t_knots = {0.0, 0.5, 1.0};

  const int32_t spline_order = 5;
  const uint32_t num_params = spline_order + 1;
  const uint32_t single_n = (t_knots.size() - 1) * num_params;

  Spline2dKernel kernel(t_knots, spline_order);
  kernel.Add2dSecondOrderDerivativeMatrix(1.0);
  Eigen::MatrixXd mat = kernel.kernel_matrix();
  Eigen::MatrixXd x_mat = mat.block(0, 0, single_n, single_n);
  Eigen::MatrixXd y_mat = mat.block(single_n, single_n, single_n, single_n);

  EXPECT_EQ(mat.rows(), mat.cols());
  EXPECT_EQ(mat.rows(), 2 * num_params * (t_knots.size() - 1));
  Eigen::MatrixXd ref_kernel_matrix = Eigen::MatrixXd::Zero(single_n, single_n);

  // clang-format off
  ref_kernel_matrix <<
     0, 0,  0,  0,    0,       0, 0, 0,  0,  0,    0,       0,
     0, 0,  0,  0,    0,       0, 0, 0,  0,  0,    0,       0,
     0, 0,  4,  6,    8,      10, 0, 0,  0,  0,    0,       0,
     0, 0,  6, 12,   18,      24, 0, 0,  0,  0,    0,       0,
     0, 0,  8, 18, 28.8,      40, 0, 0,  0,  0,    0,       0,
     0, 0, 10, 24,   40, 57.1429, 0, 0,  0,  0,    0,       0,
     0, 0,  0,  0,    0,       0, 0, 0,  0,  0,    0,       0,
     0, 0,  0,  0,    0,       0, 0, 0,  0,  0,    0,       0,
     0, 0,  0,  0,    0,       0, 0, 0,  4,  6,    8,      10,
     0, 0,  0,  0,    0,       0, 0, 0,  6, 12,   18,      24,
     0, 0,  0,  0,    0,       0, 0, 0,  8, 18, 28.8,      40,
     0, 0,  0,  0,    0,       0, 0, 0, 10, 24,   40, 57.1429;
  // clang-format on
  ref_kernel_matrix *= 2.0;

  for (int i = 0; i < x_mat.rows(); ++i) {
    for (int j = 0; j < x_mat.cols(); ++j) {
      const double param = std::pow(0.5, std::max(0, i % 6 + j % 6 - 3));
      EXPECT_NEAR(x_mat(i, j), param * ref_kernel_matrix(i, j), 1e-5);
      EXPECT_NEAR(y_mat(i, j), param * ref_kernel_matrix(i, j), 1e-5);
    }
  }

  Eigen::MatrixXd ref_g = Eigen::MatrixXd::Zero(2 * single_n, 1);
  Eigen::MatrixXd gradient = kernel.gradient();

  for (int i = 0; i < gradient.rows(); ++i) {
    for (int j = 0; j < gradient.cols(); ++j) {
      EXPECT_DOUBLE_EQ(kernel.gradient()(i, j), ref_g(i, j));
    }
  }
}

TEST(Spline2dKernel, add_third_derivative_kernel_matrix_01) {
  std::vector<double> t_knots = {0.0, 1.5};

  const int32_t spline_order = 5;
  const uint32_t num_params = spline_order + 1;
  const uint32_t single_n = (t_knots.size() - 1) * num_params;

  Spline2dKernel kernel(t_knots, spline_order);
  kernel.Add2dThirdOrderDerivativeMatrix(1.0);
  Eigen::MatrixXd mat = kernel.kernel_matrix();
  Eigen::MatrixXd x_mat = mat.block(0, 0, single_n, single_n);
  Eigen::MatrixXd y_mat = mat.block(single_n, single_n, single_n, single_n);

  EXPECT_EQ(mat.rows(), mat.cols());
  EXPECT_EQ(mat.rows(), 2 * num_params * (t_knots.size() - 1));
  Eigen::MatrixXd ref_kernel_matrix = Eigen::MatrixXd::Zero(single_n, single_n);

  // clang-format off
  ref_kernel_matrix <<
      0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,
      0,   0,   0,  36,  72, 120,
      0,   0,   0,  72, 192, 360,
      0,   0,   0, 120, 360, 720;
  // clang-format on
  ref_kernel_matrix *= 2.0;

  for (int i = 0; i < x_mat.rows(); ++i) {
    for (int j = 0; j < x_mat.cols(); ++j) {
      const double param = std::pow(1.5, std::max(0, i % 6 + j % 6 - 5));
      EXPECT_NEAR(x_mat(i, j), param * ref_kernel_matrix(i, j), 1e-5);
      EXPECT_NEAR(y_mat(i, j), param * ref_kernel_matrix(i, j), 1e-5);
    }
  }

  Eigen::MatrixXd ref_g = Eigen::MatrixXd::Zero(2 * single_n, 1);
  Eigen::MatrixXd gradient = kernel.gradient();

  for (int i = 0; i < gradient.rows(); ++i) {
    for (int j = 0; j < gradient.cols(); ++j) {
      EXPECT_DOUBLE_EQ(kernel.gradient()(i, j), ref_g(i, j));
    }
  }
}

TEST(Spline2dKernel, add_third_derivative_kernel_matrix_02) {
  std::vector<double> t_knots = {0.0, 1.5, 3.0};

  const int32_t spline_order = 5;
  const uint32_t num_params = spline_order + 1;
  const uint32_t single_n = (t_knots.size() - 1) * num_params;

  Spline2dKernel kernel(t_knots, spline_order);
  kernel.Add2dThirdOrderDerivativeMatrix(1.0);
  Eigen::MatrixXd mat = kernel.kernel_matrix();
  Eigen::MatrixXd x_mat = mat.block(0, 0, single_n, single_n);
  Eigen::MatrixXd y_mat = mat.block(single_n, single_n, single_n, single_n);

  EXPECT_EQ(mat.rows(), mat.cols());
  EXPECT_EQ(mat.rows(), 2 * num_params * (t_knots.size() - 1));
  Eigen::MatrixXd ref_kernel_matrix = Eigen::MatrixXd::Zero(6, 6);

  // clang-format off
  ref_kernel_matrix <<
      0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,
      0,   0,   0,  36,  72, 120,
      0,   0,   0,  72, 192, 360,
      0,   0,   0, 120, 360, 720;
  // clang-format on
  ref_kernel_matrix *= 2.0;

  for (int i = 0; i < x_mat.rows(); ++i) {
    for (int j = 0; j < x_mat.cols(); ++j) {
      if ((i >= 6 && j < 6) || (i < 6 && j >= 6)) {
        EXPECT_DOUBLE_EQ(x_mat(i, j), 0.0);
        EXPECT_DOUBLE_EQ(y_mat(i, j), 0.0);
      } else {
        const double param = std::pow(1.5, std::max(0, i % 6 + j % 6 - 5));
        EXPECT_NEAR(x_mat(i, j), param * ref_kernel_matrix(i % 6, j % 6), 1e-5);
        EXPECT_NEAR(y_mat(i, j), param * ref_kernel_matrix(i % 6, j % 6), 1e-5);
      }
    }
  }

  Eigen::MatrixXd ref_g = Eigen::MatrixXd::Zero(2 * single_n, 1);
  Eigen::MatrixXd gradient = kernel.gradient();

  for (int i = 0; i < gradient.rows(); ++i) {
    for (int j = 0; j < gradient.cols(); ++j) {
      EXPECT_DOUBLE_EQ(kernel.gradient()(i, j), ref_g(i, j));
    }
  }
}
}  // namespace common
