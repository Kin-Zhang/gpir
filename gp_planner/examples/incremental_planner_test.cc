/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include "gp_planner/gp/gp_incremental_path_planner.h"
#include "gp_planner/gp/gp_path_optimizer.h"
#include "gp_planner/gp/utils/bounded_penalty_function.h"
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
  FLAGS_colorlogtostderr = true;

  BoundedPenaltyFunction penalty(0.02, 0.02 * 0.3);
  double grad;
  LOG(INFO) << penalty.limit1_ << ", " << penalty.limit2_ << ", "
            << penalty.GetPenaltyAndGradient(0.038, &grad);

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
  occupancy_map.set_cell_number(std::array<int, 2>{1100, 100});
  occupancy_map.set_resolution({0.1, 0.1});
  // occupancy_map.FillConvexPoly(
  //     vector_Eigen2d{Eigen::Vector2d(-5, -1), Eigen::Vector2d(-3, -1),
  //                    Eigen::Vector2d(-3, 0), Eigen::Vector2d(-5, 0)});
  // occupancy_map.FillConvexPoly(
  //     vector_Eigen2d{Eigen::Vector2d(5, 1), Eigen::Vector2d(5, 1),
  //                    Eigen::Vector2d(5, 1), Eigen::Vector2d(5, 1)});
  auto sdf = std::make_shared<SignedDistanceField2D>(std::move(occupancy_map));
  sdf->UpdateSDF();

  GPIncrementalPathPlanner path_planner(sdf);
  // GPPathOptimizer path_planner(sdf);

  common::State initial_state;
  initial_state.position = Eigen::Vector2d(10.0, -5);
  initial_state.heading = 0.0;
  initial_state.velocity = 15.0;
  initial_state.acceleration = 0.0;
  initial_state.stamp = 0.0;

  common::FrenetState frenet_state;
  reference_line.ToFrenetState(initial_state, &frenet_state);

  LOG(INFO) << "1";
  GPPath gp_path;
  if (!path_planner.GenerateInitialGPPath(reference_line, frenet_state, 90,
                                          std::vector<double>{10}, &gp_path)) {
    LOG(ERROR) << "gp planner failed";
    return -1;
  }
  LOG(INFO) << "gp path ok";

  // prepare obstacle
  std::vector<Obstacle> dynamic_obstacle;

  StGraph st_graph;
  std::vector<double> locations, kappa_limit;
  st_graph.SetInitialState(Eigen::Vector3d(initial_state.position.x(),
                                           initial_state.velocity,
                                           initial_state.acceleration));
  st_graph.SetReferenceSpeed(10);
  st_graph.BuildStGraph(dynamic_obstacle, gp_path);
  std::vector<StNode> st_nodes;
  st_graph.SearchWithLocalTruncation(13, &st_nodes);
  if (!st_graph.GenerateInitialSpeedProfile(gp_path)) {
    LOG(ERROR) << "generate initial speed profile failed";
    return -1;
  }
  // st_graph.VisualizeStGraph();

  for (int i = 0; i < 5; ++i) {
    common::Trajectory traj;
    st_graph.GenerateTrajectory(reference_line, gp_path, &traj);

    // recording
    std::string index = std::to_string(i);
    std::vector<double> t1, x1, y1, v1, lat_a1, lat_v1, k1;
    for (const auto& p : traj) {
      t1.emplace_back(p.stamp);
      x1.emplace_back(p.position.x());
      y1.emplace_back(p.position.y());
      lat_a1.emplace_back(p.frenet_d[2] * p.frenet_s[1] * p.frenet_s[1] +
                          p.frenet_d[1] * p.frenet_s[2]);
      lat_v1.emplace_back(p.frenet_d[1] * p.frenet_s[1]);
      k1.emplace_back(p.kappa);
    }
    plt::subplot(3, 1, 1);
    plt::named_plot(index, x1, y1);
    plt::legend();
    plt::axis("equal");
    plt::subplot(3, 1, 2);
    plt::named_plot(index, t1, lat_a1);
    plt::legend();
    plt::subplot(3, 1, 3);
    plt::named_plot(index, t1, k1);
    plt::legend();

    vector_Eigen3d frenet_s;
    if (!st_graph.IsTrajectoryFeasible(gp_path, &frenet_s)) {
      path_planner.UpdateGPPath(reference_line, frenet_s, &gp_path);
    }
  }

  // common::Trajectory traj;
  // st_graph.GenerateTrajectory(reference_line, gp_path, &traj);
  // std::vector<double> t1, x1, y1, v1, lat_a1, lat_v1, k1;
  // for (const auto& p : traj) {
  //   t1.emplace_back(p.stamp);
  //   x1.emplace_back(p.position.x());
  //   y1.emplace_back(p.position.y());
  //   lat_a1.emplace_back(p.frenet_d[2] * p.frenet_s[1] * p.frenet_s[1] +
  //                       p.frenet_d[1] * p.frenet_s[2]);
  //   lat_v1.emplace_back(p.frenet_d[1] * p.frenet_s[1]);
  //   k1.emplace_back(p.kappa);
  // }
  // plt::subplot(3, 1, 1);
  // plt::plot(x1, y1, "-*");
  // plt::axis("equal");
  // plt::subplot(3, 1, 2);
  // plt::plot(t1, lat_a1);
  // plt::subplot(3, 1, 3);
  // plt::plot(t1, k1);

  // vector_Eigen3d frenet_s;
  // if (!st_graph.IsTrajectoryFeasible(gp_path, &frenet_s)) {
  //   path_planner.UpdateGPPath(reference_line, frenet_s, &gp_path);
  //   common::Trajectory traj2;
  //   st_graph.GenerateTrajectory(reference_line, gp_path, &traj2);
  //   std::vector<double> t2, x2, y2, v2, lat_a2, lat_v2, k2;
  //   for (const auto& p : traj2) {
  //     t2.emplace_back(p.stamp);
  //     x2.emplace_back(p.position.x());
  //     y2.emplace_back(p.position.y());
  //     lat_a2.emplace_back(p.frenet_d[2] * p.frenet_s[1] * p.frenet_s[1] +
  //                         p.frenet_d[1] * p.frenet_s[2]);
  //     lat_v2.emplace_back(p.frenet_d[1] * p.frenet_s[1]);
  //     k2.emplace_back(p.kappa);
  //   }

  //   plt::subplot(3, 1, 1);
  //   plt::named_plot("up", x2, y2, "-*");
  //   plt::legend();
  //   plt::axis("equal");
  //   plt::subplot(3, 1, 2);
  //   plt::named_plot("up", t2, lat_a2);
  //   plt::legend();
  //   plt::subplot(3, 1, 3);
  //   plt::named_plot("up", t2, k2);
  //   plt::legend();
  // }

  plt::show();

  // plt::subplot(3, 1, 1);
  // plt::plot(x2, y3, "-*");
  // plt::axis("equal");
  // plt::subplot(3, 1, 2);
  // plt::plot(t1, lat_a3);
  // plt::subplot(3, 1, 3);
  // plt::plot(t1, k);
  // plt::show();
  // st_graph.SaveSnapShot("/home/udi/research/ral2021_gpir/data");

  return 0;
}
