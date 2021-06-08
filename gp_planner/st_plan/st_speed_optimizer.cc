/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include 
#include "gp_planner/st_plan/st_speed_optimizer.h"

namespace planning {

bool StSpeedOptimizer::Optimize(const Eigen::Vector3d& init_s,
                                const std::vector<Obstacle>& dynamic_obstacle,
                                const GPPath& gp_path) {
  // * search for initial feasible s-t profile
  st_graph_.SetInitialState(init_s);
  st_graph_.BuildStGraph(dynamic_obstacle, gp_path);
  std::vector<StNode> st_nodes;
  if (!st_graph_.SearchWithLocalTruncation(9, &st_nodes)) {
    LOG(ERROR) << "fail to generate feasible initial s-t profile";
    return false;
  }

  // * optimize
  const auto& grid_map = st_graph_.grid_map(); 

  return true;
}

}  // namespace planning
