/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#pragma once

#include <Eigen/Dense>

namespace planning {

class GPPathInitializer {
 public:
  GPPathInitializer() = default;

  bool GenerateInitialPath(std::vector<std::pair<double, double>> boundary,
                           std::array<double, 3> x_init,
                           std::array<double, 3> x_end, const double delta_s);

 private:
  bool Optimize();

 private:
  int num_of_point_ = 0;

  Eigen::MatrixXd P_;
  Eigen::VectorXd q_;
  Eigen::MatrixXd A_;
  Eigen::VectorXd l_;
  Eigen::VectorXd u_;
};

}  // namespace planning
