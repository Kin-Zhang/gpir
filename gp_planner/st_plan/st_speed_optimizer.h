/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#pragma once

#include <Eigen/Dense>

#include "gp_planner/gp/utils/gp_path.h"
#include "gp_planner/st_plan/st_graph.h"

namespace planning {

class StSpeedOptimizer {
 public:
  StSpeedOptimizer(/* args */);

  bool Optimize(const Eigen::Vector3d& init_s,
                const std::vector<Obstacle>& dynamic_obstacle,
                const GPPath& gp_path);

 private:
  StGraph st_graph_;
};

}  // namespace planning
