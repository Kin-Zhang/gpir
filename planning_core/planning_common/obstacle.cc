/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include "planning_core/planning_common/obstacle.h"

namespace planning {

void Obstacle::SetBoundingBox(const double length, const double width,
                              const double height) {
  height_ = height;
  bbox_ = common::Box2D(state_.position, length, width, state_.heading, height);
}

}  // namespace planning
