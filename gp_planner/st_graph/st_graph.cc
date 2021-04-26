/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include "gp_planner/st_graph/st_graph.h"

namespace planning {

void StGraph::BuildStGraph(const std::vector<Obstacle>& dynamic_obstacles,
                           const GPPath& gp_path) {
  std::vector<std::vector<std::vector<StPoint>>> st_block_segments;

  int size = dynamic_obstacles.size();
  st_block_segments.resize(size);

  for (int i = 0; i < size; ++i) {
    GetObstacleBlockSegment(dynamic_obstacles[i], gp_path,
                            &st_block_segments[i]);
  }

  auto grid_map = sdf_.mutable_occupancy_map();
  sdf_.set_origin({0.0, gp_path.start_s()});
  sdf_.set_resolution(0.1);
  sdf_.set_cell_num({80, 1000});
  vector_Eigen<Eigen::Vector2d> corners;
  corners.resize(4);
  for (const auto& st_block_segment : st_block_segments) {
    if (st_block_segment.empty()) continue;
    for (const auto& st_points : st_block_segment) {
      if (st_points.empty()) continue;
      if (st_points.size() == 1) {
        corners.clear();
        corners[0] = Eigen::Vector2d(st_points[0].t, st_points[0].s_l);
        corners[1] = Eigen::Vector2d(st_points[0].t, st_points[0].s_u);
        corners[2] = Eigen::Vector2d(st_points[0].t + 0.1, st_points[0].s_u);
        corners[3] = Eigen::Vector2d(st_points[0].t + 0.1, st_points[0].s_l);
        grid_map->FillConvexPoly(corners);
        continue;
      }
      for (size_t i = 0; i < st_points.size() - 1; ++i) {
        corners.clear();
        corners.emplace_back(st_points[i].t, st_points[i].s_l);
        corners.emplace_back(st_points[i].t, st_points[i].s_u);
        corners.emplace_back(st_points[i + 1].t, st_points[i + 1].s_u);
        corners.emplace_back(st_points[i + 1].t, st_points[i + 1].s_l);
        grid_map->FillConvexPoly(corners);
      }
    }
  }

  sdf_.UpdateSDF();
}

void StGraph::GetObstacleBlockSegment(
    const Obstacle& obstacle, const GPPath& gp_path,
    std::vector<std::vector<StPoint>>* st_block_segment) {
  const double length = obstacle.length();
  const double width = obstacle.width();
  double s_l = 0.0, s_u = 0.0;

  bool extend_from_previous = false;
  std::vector<StPoint>* current_seg = nullptr;
  for (const auto& future_point : obstacle.prediction()) {
    if (gp_path.HasOverlapWith(future_point, length, width, &s_l, &s_u)) {
      if (!extend_from_previous) {
        st_block_segment->emplace_back();
        current_seg = &st_block_segment->back();
        extend_from_previous = true;
      }
      current_seg->emplace_back(
          StPoint(s_l, s_u, future_point.stamp - stamp_now_));
    } else {
      extend_from_previous = false;
    }
  }
}
}  // namespace planning
