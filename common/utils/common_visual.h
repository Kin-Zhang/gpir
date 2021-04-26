/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>

#pragma once

namespace common {

class CommonVisual {
 public:
  static void FillHeader(visualization_msgs::Marker* maker);

  static void StateToPose(const Eigen::Vector2d& position, const double heading,
                          geometry_msgs::Pose* pose, const double z = 0.0);

  static void PositionToPose(const Eigen::Vector2d& position,
                             geometry_msgs::Pose* pose, const double z = 0.0);

  static void PositionToPoint(const Eigen::Vector2d& position,
                              geometry_msgs::Point* point,
                              const double z = 0.0);
};

}  // namespace common
