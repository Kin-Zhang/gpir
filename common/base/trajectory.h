/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#pragma once

#include <Eigen/Dense>
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

}  // namespace common