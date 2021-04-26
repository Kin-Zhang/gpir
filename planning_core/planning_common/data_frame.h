/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#pragma once

#include "common/base/state.h"
#include "planning_core/planning_common/obstacle.h"

namespace planning {

struct DataFrame {
  common::State state;
  std::vector<Obstacle> obstacles;
};

}  // namespace planning
