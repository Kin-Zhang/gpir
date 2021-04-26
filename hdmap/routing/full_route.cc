/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include "hdmap/routing/full_route.h"

namespace hdmap {

std::string ToString(const LaneSegmentBehavior& behavior) {
  if (behavior == LaneSegmentBehavior::kLeftChange)
    return "left";
  else if (behavior == LaneSegmentBehavior::kRightChange)
    return "right";
  else
    return "keep";
}
}  // namespace hdmap

std::ostream& operator<<(std::ostream& os,
                         const hdmap::LaneSegmentBehavior& behavior) {
  os << hdmap::ToString(behavior);
  return os;
}