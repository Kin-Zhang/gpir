/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#pragma once

#include "hdmap/road_network/road_network.h"

namespace planning {

class PlotUtils {
 public:
  static void PlotWayPoints(const std::vector<hdmap::WayPoint> &waypoints);
};

}  // namespace planning