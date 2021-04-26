/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#pragma once

#include "planning_core/navigation/navigation_map.h"

namespace planning {

class Planner {
 public:
  Planner() = default;
  virtual ~Planner() = default;

  virtual void Init() = 0;
  virtual void PlanOnce(NavigationMap* navigation_map_) = 0;
};

}  // namespace planning
