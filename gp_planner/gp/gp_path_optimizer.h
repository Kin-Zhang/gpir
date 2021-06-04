/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#pragma once

#include <memory>

#include "common/base/state.h"
#include "common/base/trajectory.h"
#include "gp_planner/gp/utils/gp_path.h"
#include "gp_planner/sdf/signed_distance_field_2d.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "planning_core/navigation/reference_line.h"

namespace planning {

class GPPathOptimizer {
 public:
  GPPathOptimizer() = default;
  GPPathOptimizer(std::shared_ptr<SignedDistanceField2D> sdf) : sdf_(sdf) {}

  void set_sdf(std::shared_ptr<SignedDistanceField2D> sdf) { sdf_ = sdf; }

  bool GenerateGPPath(const ReferenceLine& reference_line,
                      const common::FrenetState& frenet_state, const double length,
                      const double s, common::Trajectory* trajectory,
                      GPPath* gp_path);

 private:
  int num_of_nodes_ = 21;

  std::vector<double> s_refs_;
  gtsam::NonlinearFactorGraph graph_;
  std::shared_ptr<SignedDistanceField2D> sdf_;
};

}  // namespace planning
