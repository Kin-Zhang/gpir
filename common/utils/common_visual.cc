/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include "common/utils/common_visual.h"

#include <tf/tf.h>

namespace common {

void CommonVisual::FillHeader(visualization_msgs::Marker* maker) {
  maker->header.frame_id = "map";
  maker->header.stamp = ros::Time::now();
}

void CommonVisual::StateToPose(const Eigen::Vector2d& position,
                               const double heading, geometry_msgs::Pose* pose,
                               const double z) {
  pose->position.x = position.x();
  pose->position.y = position.y();
  pose->position.z = z;

  auto q = tf::createQuaternionFromYaw(heading);
  tf::quaternionTFToMsg(q, pose->orientation);
}

void CommonVisual::PositionToPose(const Eigen::Vector2d& position,
                                  geometry_msgs::Pose* pose, const double z) {
  pose->position.x = position.x();
  pose->position.y = position.y();
  pose->position.z = z;
  pose->orientation.x = 0;
  pose->orientation.y = 0;
  pose->orientation.z = 0;
  pose->orientation.w = 1;
}

void CommonVisual::PositionToPoint(const Eigen::Vector2d& position,
                                   geometry_msgs::Point* point,
                                   const double z) {
  point->x = position.x();
  point->y = position.y();
  point->z = z;
}

}  // namespace common
