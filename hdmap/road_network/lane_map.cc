/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include "hdmap/road_network/lane_map.h"

namespace hdmap {

LaneMapType LaneMap::lane_map_;

LaneMapType* LaneMap::mutable_lane_map() { return &lane_map_; }

std::shared_ptr<Lane> LaneMap::GetLane(const LaneId& id) {
  if (lane_map_.find(id) != lane_map_.end())
    return lane_map_[id];
  else
    return nullptr;
}
}  // namespace hdmap
