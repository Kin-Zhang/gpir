/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#pragma once

#include "common/solver/osqp/osqp_interface.h"

namespace common {

class QPProblem {
 public:
  // * minimize{x}:  (Ax-b)' W (Ax-b) + w x'x
  // * s.t.           l <= Cx <= u
  // ! l and u may be modified, copy first if necessary
  static bool Solve(const ColSparseMatrix& A, const Eigen::VectorXd& b,
                    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& W,
                    const double w, const ColSparseMatrix& C,
                    Eigen::VectorXd& l, Eigen::VectorXd& u, Eigen::VectorXd* x);

  static bool Solve(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& W,
                    const double w, const Eigen::MatrixXd& C,
                    Eigen::VectorXd& l, Eigen::VectorXd u, Eigen::VectorXd* x);
};

}  // namespace common
