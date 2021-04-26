/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#pragma once

#include <deque>

#include "hdmap/routing/full_route.h"
#include "planning_core/navigation/route_segment.h"

namespace planning {

class RouteSequence : public std::vector<RouteSegment> {
 public:
  RouteSequence() = default;

  void Update(const Eigen::Vector2d& position);

  inline int current_index() const { return current_index_; }
  inline hdmap::LaneSegmentBehavior main_action() const;
  inline bool ApproachingDestination() { return approaching_destination_; }
  inline bool arrived() const { return arrived_; }

 private:
  int num_of_segments_ = 0;
  int current_index_ = 0;
  bool arrived_ = false;
  bool approaching_destination_ = false;
};

// inline
inline hdmap::LaneSegmentBehavior RouteSequence::main_action() const {
  return data()[current_index_].main_action();
}
}  // namespace planning
