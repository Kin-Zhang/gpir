/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#pragma once

#include <memory>

#include "gp_planner/gp/utils/gp_path.h"
#include "gp_planner/sdf/signed_distance_field_2d.h"
#include "gtsam/nonlinear/ISAM2.h"
#include "planning_core/navigation/reference_line.h"

namespace planning {

class GPIncrementalPathPlanner {
 public:
  GPIncrementalPathPlanner() = default;
  GPIncrementalPathPlanner(std::shared_ptr<SignedDistanceField2D> sdf)
      : sdf_(sdf){};

  bool GenerateInitialGPPath(const ReferenceLine& reference_line,
                             const common::FrenetState& initial_state,
                             const double length,
                             const std::vector<double>& obstacle_location_hint,
                             GPPath* gp_path);

  bool UpdateGPPath(const ReferenceLine& reference_line,
                    const std::vector<double>& locations,
                    const std::vector<double>& kappa_limit, GPPath* gp_path);

  void set_sdf(std::shared_ptr<SignedDistanceField2D> sdf) { sdf_ = sdf; }

 protected:
  bool DecideInitialPathBoundary(
      const Eigen::Vector2d& init_pos, const double init_angle,
      const std::vector<double>& obstacle_location_hint,
      const std::vector<std::vector<std::pair<double, double>>>& boundaries,
      std::vector<double>* lb, std::vector<double>* ub);

  int FindLocationIndex(const double s);

 private:
  gtsam::ISAM2 isam2_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values map_result_;

  int num_of_nodes_ = 21;
  double ego_width_ = 2.1;
  double ego_half_width_ = 1.0;
  const double kappa_limit_ = 0.02;
  double interval_ = 0.0;
  std::vector<double> node_locations_;
  std::shared_ptr<SignedDistanceField2D> sdf_;
};
}  // namespace planning
