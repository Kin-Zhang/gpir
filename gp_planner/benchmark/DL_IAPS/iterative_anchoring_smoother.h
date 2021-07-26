/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/*
 * @file
 */

#pragma once

#include <utility>
#include <vector>

#include "Eigen/Eigen"
#include "gp_planner/benchmark/DL_IAPS/box2d.h"
#include "gp_planner/benchmark/DL_IAPS/discretized_path.h"
#include "gp_planner/benchmark/DL_IAPS/line_segment2d.h"
#include "gp_planner/benchmark/DL_IAPS/vec2d.h"

namespace apollo {
namespace planning {
class IterativeAnchoringSmoother {
 public:
  IterativeAnchoringSmoother() = default;

  ~IterativeAnchoringSmoother() = default;

  bool Smooth(const Eigen::MatrixXd& xWS,
              const std::vector<std::vector<common::math::Vec2d>>&
                  obstacles_vertices_vec,
              DiscretizedPath* smoothed_path);

 private:
  void AdjustStartEndHeading(
      const Eigen::MatrixXd& xWS,
      std::vector<std::pair<double, double>>* const point2d);

  bool ReAnchoring(const std::vector<size_t>& colliding_point_index,
                   DiscretizedPath* path_points);

  bool GenerateInitialBounds(const DiscretizedPath& path_points,
                             std::vector<double>* initial_bounds);

  bool SmoothPath(const DiscretizedPath& raw_path_points,
                  const std::vector<double>& bounds,
                  DiscretizedPath* smoothed_path_points);

  bool SetPathProfile(const std::vector<std::pair<double, double>>& point2d,
                      DiscretizedPath* raw_path_points);

  bool CheckCollisionAvoidance(const DiscretizedPath& path_points,
                               std::vector<size_t>* colliding_point_index);

  void AdjustPathBounds(const std::vector<size_t>& colliding_point_index,
                        std::vector<double>* bounds);

  bool CheckGear(const Eigen::MatrixXd& xWS);

  // @brief: a helper function on discrete point heading adjustment
  double CalcHeadings(const DiscretizedPath& path_points, const size_t index);

 private:
  // vehicle_param
  double ego_length_ = 4.6;
  double ego_width_ = 2.1;
  double center_shift_distance_ = 0.0;

  std::vector<std::vector<common::math::LineSegment2d>>
      obstacles_linesegments_vec_;

  std::vector<size_t> input_colliding_point_index_;

  bool enforce_initial_kappa_ = true;

  // gear DRIVE as true and gear REVERSE as false
  bool gear_ = false;
};
}  // namespace planning
}  // namespace apollo
