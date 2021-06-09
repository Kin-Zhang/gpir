/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#pragma once

#include <Eigen/Dense>
#include <iomanip>
#include <vector>

#include "common/base/state.h"

namespace common {

class Trajectory : public std::vector<common::State> {
 public:
  Trajectory() = default;
  virtual ~Trajectory() = default;

  int GetNearsetIndex(const Eigen::Vector2d& pos) const {
    int min_index = 0;
    double dis = 0;
    double min_dis = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < size(); ++i) {
      dis = (data()[i].position - pos).squaredNorm();
      if (dis < min_dis) {
        min_index = i;
        min_dis = dis;
      }
    }
    return min_index;
  }

  const common::State& GetNearestState(const Eigen::Vector2d& pos) const {
    return data()[GetNearsetIndex(pos)];
  }
};

inline std::ostream& operator<<(std::ostream& os,
                                const Trajectory& trajectory) {
  os << "Trajectory: \n";
  for (const auto& point : trajectory) {
    os << std::fixed << std::setprecision(4) << "[" << std::left
       << "x: " << std::setw(6) << point.position.x() << ", "
       << "y: " << std::setw(6) << point.position.y() << ", "
       << "s: " << std::setw(6) << point.s << ", "
       << "v: " << std::setw(6) << point.velocity << ", "
       << "a: " << std::setw(6) << point.acceleration << "]\n";
  }
  return os;
}

}  // namespace common