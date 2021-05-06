/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include "gp_planner/gp/gp_path_optimizer.h"
#include "gp_planner/st_plan/st_graph.h"
#include "gp_planner/thirdparty/matplotlib-cpp/matplotlibcpp.h"
#include "planning_core/navigation/reference_line.h"
#include "planning_core/planning_common/obstacle.h"
#include "planning_core/planning_common/vehicle_info.h"

using namespace planning;
namespace plt = matplotlibcpp;

int main(int argc, char const* argv[]) {
  google::InstallFailureSignalHandler();
  google::InitGoogleLogging(argv[0]);
  google::SetStderrLogging(google::INFO);
  // FLAGS_log_prefix = false
  FLAGS_colorlogtostderr = true;

  // prepare vehicle config
  auto vehicle_param = VehicleInfo::Instance().mutable_vehicle_param();
  vehicle_param->length = 4.79178;
  vehicle_param->width = 2.16345;
  vehicle_param->height = 1.48832;
  vehicle_param->wheel_base = 3.00464;
  vehicle_param->front_to_front_axle = 0.777665;
  vehicle_param->back_to_rear_axle = 1.00947;
  vehicle_param->rear_axle_to_center = 1.38642;
  LOG(INFO) << "vehicle param ok";

  // prepare reference_line
  std::vector<hdmap::WayPoint> waypoints;
  hdmap::WayPoint waypoint;
  for (int i = 0; i < 120; ++i) {
    waypoint.point = Eigen::Vector2d(i, 0.0);
    waypoint.s = i;
    waypoint.heading = 0.0;
    waypoints.emplace_back(waypoint);
  }
  ReferenceLine reference_line;
  reference_line.GenerateReferenceLine(waypoints);
  LOG(INFO) << "reference line ok";

  // prepare sdf
  OccupancyMap occupancy_map;
  occupancy_map.set_origin({0, -5});
  occupancy_map.set_cell_number(std::array<int, 2>{900, 100});
  occupancy_map.set_resolution({0.1, 0.1});
  auto sdf = std::make_shared<SignedDistanceField2D>(std::move(occupancy_map));
  sdf->UpdateSDF();

  GPPathOptimizer gp_path_optimizer;
  gp_path_optimizer.set_sdf(sdf);

  common::State initial_state;
  initial_state.position = Eigen::Vector2d(10.0, 0.0);
  initial_state.heading = 0.0;
  initial_state.velocity = 10.0;
  initial_state.acceleration = 0.0;
  initial_state.stamp = 0.0;

  GPPath gp_path;
  common::Trajectory trajectory;
  if (!gp_path_optimizer.GenerateGPPath(reference_line, initial_state, 90, -1,
                                        &trajectory, &gp_path)) {
    LOG(ERROR) << "gp planner failed";
    return -1;
  }
  LOG(INFO) << "gp path ok";

  // prepare obstacle
  std::vector<Obstacle> dynamic_obstacle;
  Obstacle leading_vehicle;
  Obstacle leading_vehicle2;
  auto future_points = leading_vehicle.mutable_prediction();
  auto future_points2 = leading_vehicle2.mutable_prediction();
  constexpr double inital_distance = 30;
  constexpr double leading_v = 8.0;
  constexpr double leading_v2 = 5.0;
  constexpr double initial_y = 0.0;
  constexpr double initial_y2 = .0;
  common::State future_state;
  for (double t = 0.0; t < 8.0; t += 0.2) {
    future_state.position =
        Eigen::Vector2d(-10 + leading_v * t, initial_y * (8.0 - t) / 8.0);
    future_state.heading = 0.0;
    future_state.stamp = t;
    future_points->emplace_back(future_state);
    future_state.position =
        Eigen::Vector2d(50 + leading_v2 * t, initial_y2 * (8.0 - t) / 8.0);
    future_points2->emplace_back(future_state);
  }
  leading_vehicle.SetBoundingBox(4.7, 2.1, 1.4);
  leading_vehicle.set_static(false);
  leading_vehicle2.SetBoundingBox(4.7, 2.1, 1.4);
  leading_vehicle2.set_static(false);
  dynamic_obstacle.emplace_back(leading_vehicle);
  dynamic_obstacle.emplace_back(leading_vehicle2);

  LOG(INFO) << "obstacle ok";

  StGraph st_graph;
  st_graph.SetInitialState(Eigen::Vector3d(initial_state.position.x(),
                                           initial_state.velocity,
                                           initial_state.acceleration));
  st_graph.BuildStGraph(dynamic_obstacle, gp_path);
  st_graph.LocalTopSearch(13);
  st_graph.VisualizeStGraph();

  return 0;
}
