/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include "common/solver/qp_problem.h"

#include <glog/logging.h>

#include <iostream>

namespace common {

bool QPProblem::Solve(const ColSparseMatrix& A, const Eigen::VectorXd& b,
                      const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& W,
                      const double w, const ColSparseMatrix& C,
                      Eigen::VectorXd& l, Eigen::VectorXd& u,
                      Eigen::VectorXd* x) {
  // *  equivalent to minimize 1/2 x' (A' W A + w I) x - b' W A x
  // *  P = A' W A + w I
  // *  q = -A'W b
  CHECK_EQ(A.rows(), W.rows());

  ColSparseMatrix P =
      A.transpose() * W * A +
      w * (ColSparseMatrix)Eigen::MatrixXd::Identity(A.cols(), A.cols())
              .sparseView();
  Eigen::VectorXd q = -A.transpose() * W * b;

  // std::cout << A.toDense() << "\n\n";
  // std::cout << P.toDense() << std::endl;

  return OsqpInterface::Solve(P, q, C, l, u, x);
}

bool QPProblem::Solve(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                      const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& W,
                      const double w, const Eigen::MatrixXd& C,
                      Eigen::VectorXd& l, Eigen::VectorXd u,
                      Eigen::VectorXd* x) {
  CHECK_EQ(A.rows(), W.rows());

  Eigen::MatrixXd P =
      A.transpose() * W * A + w * Eigen::MatrixXd::Identity(A.cols(), A.cols());
  Eigen::VectorXd q = -A.transpose() * W * b;
  return OsqpInterface::Solve(P, q, C, l, u, x);
}
}  // namespace common
