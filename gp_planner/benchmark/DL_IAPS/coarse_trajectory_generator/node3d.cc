/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "gp_planner/benchmark/DL_IAPS/coarse_trajectory_generator/node3d.h"

namespace apollo {
namespace planning {

using apollo::common::math::Box2d;

Node3d::Node3d(double x, double y, double phi) {
  x_ = x;
  y_ = y;
  phi_ = phi;
}

Node3d::Node3d(double x, double y, double phi,
               const std::vector<double>& XYbounds) {
  CHECK_EQ(XYbounds.size(), 4U)
      << "XYbounds size is not 4, but" << XYbounds.size();

  x_ = x;
  y_ = y;
  phi_ = phi;

  x_grid_ = static_cast<int>((x_ - XYbounds[0]) / 0.3);
  y_grid_ = static_cast<int>((y_ - XYbounds[2]) / 0.3);
  phi_grid_ = static_cast<int>((phi_ - (-M_PI)) / 0.1);

  traversed_x_.push_back(x);
  traversed_y_.push_back(y);
  traversed_phi_.push_back(phi);

  index_ = ComputeStringIndex(x_grid_, y_grid_, phi_grid_);
}

Node3d::Node3d(const std::vector<double>& traversed_x,
               const std::vector<double>& traversed_y,
               const std::vector<double>& traversed_phi,
               const std::vector<double>& XYbounds) {
  CHECK_EQ(XYbounds.size(), 4U)
      << "XYbounds size is not 4, but" << XYbounds.size();
  CHECK_EQ(traversed_x.size(), traversed_y.size());
  CHECK_EQ(traversed_x.size(), traversed_phi.size());

  x_ = traversed_x.back();
  y_ = traversed_y.back();
  phi_ = traversed_phi.back();

  // XYbounds in xmin, xmax, ymin, ymax
  x_grid_ = static_cast<int>((x_ - XYbounds[0]) / 0.3);
  y_grid_ = static_cast<int>((y_ - XYbounds[2]) / 0.3);
  phi_grid_ = static_cast<int>((phi_ - (-M_PI)) / 0.1);

  traversed_x_ = traversed_x;
  traversed_y_ = traversed_y;
  traversed_phi_ = traversed_phi;

  index_ = ComputeStringIndex(x_grid_, y_grid_, phi_grid_);
  step_size_ = traversed_x.size();
}

Box2d Node3d::GetBoundingBox(const double x, const double y, const double phi) {
  double ego_length = 1.0;
  double ego_width = 0.5;
  double shift_distance = 0.0;
  Box2d ego_box(
      {x + shift_distance * std::cos(phi), y + shift_distance * std::sin(phi)},
      phi, ego_length, ego_width);
  return ego_box;
}

bool Node3d::operator==(const Node3d& right) const {
  return right.GetIndex() == index_;
}

std::string Node3d::ComputeStringIndex(int x_grid, int y_grid, int phi_grid) {
  //   return absl::StrCat(x_grid, "_", y_grid, "_", phi_grid);c
  return std::to_string(x_grid) + "_" + std::to_string(y_grid) + "_" +
         std::to_string(phi_grid);
}

}  // namespace planning
}  // namespace apollo
