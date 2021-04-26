/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#pragma once

#include "hdmap/routing/full_route.h"

namespace planning {

class RouteSegment {
 public:
  RouteSegment();
  RouteSegment(const hdmap::LaneSegment& lane_segment)
      : id_(lane_segment.id), length_(lane_segment.length) {}

  inline hdmap::LaneId id() const { return id_; }
  inline hdmap::LaneSegmentBehavior main_action() const;
  inline const std::vector<hdmap::LaneSegmentBehavior>& alternative_actions()
      const;

  inline void set_main_action(hdmap::LaneSegmentBehavior action);
  inline void add_alternative_actions(hdmap::LaneSegmentBehavior action);

 private:
  hdmap::LaneId id_;
  double length_ = 0.0;

  hdmap::LaneSegmentBehavior main_action_ = hdmap::LaneSegmentBehavior::kNone;
  hdmap::LaneSegmentBehavior next_action_ = hdmap::LaneSegmentBehavior::kNone;
  std::vector<hdmap::LaneSegmentBehavior> alternative_actions_;
};

inline hdmap::LaneSegmentBehavior RouteSegment::main_action() const {
  return main_action_;
}

inline const std::vector<hdmap::LaneSegmentBehavior>&
RouteSegment::alternative_actions() const {
  return alternative_actions_;
}

inline void RouteSegment::set_main_action(hdmap::LaneSegmentBehavior action) {
  main_action_ = action;
}

inline void RouteSegment::add_alternative_actions(
    hdmap::LaneSegmentBehavior action) {
  alternative_actions_.emplace_back(action);
}
}  // namespace planning
