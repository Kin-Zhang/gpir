/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include "hdmap/utils/admap_convertion.h"

namespace hdmap {

ad::map::point::ENUPoint AdMapConvertion::ToEnuPoint(
    const Eigen::Vector2d& position) {
  return ad::map::point::createENUPoint(position.x(), position.y(), 0.0);
}

Eigen::Vector2d AdMapConvertion::FromEnuPoint(
    const ad::map::point::ENUPoint& position) {
  return Eigen::Vector2d(position.x, position.y);
}
}  // namespace hdmap
