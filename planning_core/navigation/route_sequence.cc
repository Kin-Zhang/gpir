/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include "planning_core/navigation/route_sequence.h"

#include "hdmap/road_network/lane_map.h"

namespace planning {

void RouteSequence::Update(const Eigen::Vector2d& position) {
  constexpr double kEpsilon = 0.1;
  auto current_lane = hdmap::LaneMap::GetLane(data()[current_index_].id());
  if (main_action() == hdmap::LaneSegmentBehavior::kKeep) {
    if (current_lane->GetArcLength(position) >
        current_lane->length() - kEpsilon) {
      if (current_index_ == size() - 1) {
        arrived_ = true;
      } else {
        ++current_index_;
      }
    }
  } else {
    if (!current_lane->IsInLane(position)) {
      ++current_index_;
    }
  }
  if (current_index_ == size() - 1) approaching_destination_ = true;
}

}  // namespace planning
