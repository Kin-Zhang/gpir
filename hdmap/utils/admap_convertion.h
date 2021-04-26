/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#pragma once

#include <Eigen/Core>

#include "ad/map/point/Operation.hpp"

namespace hdmap {

class AdMapConvertion {
 public:
  static ad::map::point::ENUPoint ToEnuPoint(const Eigen::Vector2d& position);

  static Eigen::Vector2d FromEnuPoint(const ad::map::point::ENUPoint& position);
};
}  // namespace hdmap
