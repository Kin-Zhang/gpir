/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#pragma once

#include <Eigen/Dense>

#include "gp_planner/gp/utils/gp_path.h"
#include "gp_planner/sdf/signed_distance_field_2d.h"
#include "planning_core/planning_common/obstacle.h"

namespace planning {

struct StPoint {
  double s_l;
  double s_u;
  double t;

  StPoint(const double s_l, const double s_u, const double t)
      : s_l(s_l), s_u(s_u), t(t) {}
};

class StGraph {
 public:
  StGraph() = default;
  StGraph(const Eigen::Vector3d& s0);

  void BuildStGraph(const std::vector<Obstacle>& dynamic_obstacles,
                    const GPPath& gp_path);

 private:
  void GetObstacleBlockSegment(
      const Obstacle& obstacle, const GPPath& gp_path,
      std::vector<std::vector<StPoint>>* st_block_segment);

 private:
  double stamp_now_ = 0.0;
  double ego_half_length_ = 0.0;
  double safety_margin_ = 0.0;

  SignedDistanceField2D sdf_;
};

}  // namespace planning
