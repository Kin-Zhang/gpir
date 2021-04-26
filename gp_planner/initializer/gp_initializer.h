/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include <Eigen/Core>
#include <vector>

#pragma once

namespace planning {

class GPInitializer {
 public:
  GPInitializer() = default;
  ~GPInitializer() = default;

  void SetBoundary(
      std::vector<std::vector<std::pair<double, double>>> boundary);

  bool Solve(std::array<double, 3> x0, std::array<double, 3> xn,
             const double interval, std::vector<Eigen::Vector3d>* res,
             std::vector<double>* l);

  bool Solve2(Eigen::Vector3d x0, Eigen::Vector3d xn,
              std::vector<double> s_refs, const double s, const double lb,
              const double ub, std::vector<Eigen::Vector3d>* res,
              std::vector<double>* l);

 private:
  std::vector<std::pair<double, double>> boundary_;
};
}  // namespace planning
