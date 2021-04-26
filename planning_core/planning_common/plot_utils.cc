/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include "planning_core/planning_common/plot_utils.h"

#include "common/thirdparty/matplotlibcpp/matplotlibcpp.h"

namespace planning {

namespace plt = matplotlibcpp;

void PlotUtils::PlotWayPoints(const std::vector<hdmap::WayPoint>& waypoints) {
  std::vector<double> x, y;
  for (const auto& p : waypoints) {
    x.emplace_back(p.point.x());
    y.emplace_back(p.point.y());
  }
  plt::plot(x, y, "-*");
  plt::axis("equal");
  plt::show();
}

}  // namespace planning
