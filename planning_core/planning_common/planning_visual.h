/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#pragma once

#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <visualization_msgs/MarkerArray.h>

#include "common/utils/color_map.h"
#include "planning_core/planning_common/obstacle.h"

namespace planning {

namespace jsk_msgs = jsk_recognition_msgs;

class PlanningVisual {
 public:
  static void BBoxToSolidCubeMarker(const common::Box2D& bbox,
                                    common::Color color,
                                    visualization_msgs::Marker* maker);

  static void BBoxToJskBBox(const common::Box2D& bbox,
                            jsk_recognition_msgs::BoundingBox* jsk_bbox,
                            const int label);

  static void ObstacleToJskBBoxArray(
      const std::vector<Obstacle>& obstacles,
      jsk_recognition_msgs::BoundingBoxArray* bbox_array);

  static void ObstacleInfoToMarkerArray(
      const std::vector<Obstacle>& obstacles,
      visualization_msgs::MarkerArray* makers);

  static void GetTrafficConeMarker(const Eigen::Vector2d& pos, const int id,
                                   visualization_msgs::Marker* marker);

  static void GetPlannerBoxMarker(const Eigen::Vector2d& pos,
                                  const double width, const double length,
                                  const double heading, common::Color color,
                                  visualization_msgs::Marker* marker);
};
}  // namespace planning
