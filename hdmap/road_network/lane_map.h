/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#pragma once

#include <memory>
#include <unordered_map>

#include "hdmap/road_network/lane.h"
#include "hdmap/road_network/road_network.h"

namespace hdmap {

using LaneMapType =
    std::unordered_map<LaneId, std::shared_ptr<Lane>, LaneIdHasher>;

class LaneMap {
 public:
  static std::shared_ptr<Lane> GetLane(const LaneId &id);

 private:
  static LaneMapType lane_map_;
  static LaneMapType *mutable_lane_map();

  LaneMap() = delete;
  LaneMap(const LaneMap &) = delete;
  LaneMap &operator=(const LaneMap &) = delete;

  friend class HdMapImpl;
};

}  // namespace hdmap
