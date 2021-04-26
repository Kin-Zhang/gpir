/* Copyright 2019 Unity-Drive Inc. All rights reserved */
/*     Author: ChengJie (chengjie@unity-drive.com)     */

#include "common/smoothing/affine_constraint.h"

#include <gtest/gtest.h>

namespace common {

using std::cout;
using std::endl;

TEST(AffineConstraint, add_constraint) {
  AffineConstraint affine_constraint;

  Eigen::MatrixXd constraint_matrix_one(2, 2);
  Eigen::MatrixXd constraint_matrix_two(2, 2);
  Eigen::MatrixXd constraint_matrix(4, 2);

  constraint_matrix_one << 1, 2, 3, 4;
  constraint_matrix_two << 5, 6, 7, 8;
  constraint_matrix << 1, 2, 3, 4, 5, 6, 7, 8;

  cout << "constraint_matrix_one:\n" << constraint_matrix_one << endl;
  cout << "constraint_matrix_two:\n" << constraint_matrix_two << endl;
  cout << "constraint_matrix:\n" << constraint_matrix << endl;

  std::vector<double> lower_bound_one{-1, -2}, lower_bound_two{-3, -4},
      lower_bound{-1, -2, -3, -4};
  std::vector<double> upper_bound_one{1, 2}, upper_bound_two{3, 4},
      upper_bound{1, 2, 3, 4};

  affine_constraint.AddConstraint(constraint_matrix_one, lower_bound_one,
                                  upper_bound_one);
  affine_constraint.AddConstraint(constraint_matrix_two, lower_bound_two,
                                  upper_bound_two);

  const auto& affine_matrix = affine_constraint.constraint_matrix();

  cout << "affine matrix:\n" << affine_matrix << endl;

  for (size_t row = 0; row < affine_matrix.rows(); ++row) {
    for (size_t col = 0; col < affine_matrix.cols(); ++col) {
      EXPECT_DOUBLE_EQ(affine_matrix(row, col), constraint_matrix(row, col));
    }
  }

  for (size_t i = 0; i < affine_constraint.lower_bound().size(); ++i) {
    EXPECT_DOUBLE_EQ(lower_bound[i], affine_constraint.lower_bound()[i]);
    EXPECT_DOUBLE_EQ(upper_bound[i], affine_constraint.upper_bound()[i]);
  }
}
}  // namespace common
