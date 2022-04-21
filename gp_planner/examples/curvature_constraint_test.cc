/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include "common/utils/io_utils.h"
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
  for (int i = 0; i <= 120; ++i) {
    waypoint.point = Eigen::Vector2d(i, 0.0);
    waypoint.s = i;
    waypoint.heading = 0.0;
    waypoints.emplace_back(waypoint);
    // double angle = M_PI / 100.0 * i;
    // waypoint.s = angle * 5;
    // waypoint.point = 5 * Eigen::Vector2d(std::cos(angle), std::sin(angle));
    // waypoints.emplace_back(waypoint);
    // ref_x.emplace_back(waypoint.point.x());
    // ref_y.emplace_back(waypoint.point.y());
    // LOG(INFO) << "x, " << ref_x.back() << ", y " << ref_y.back() << ", s, "
    //           << waypoint.s;
  }
  ReferenceLine reference_line;
  // reference_line.GenerateReferenceLine(ref_points);
  reference_line.GenerateReferenceLine(waypoints);
  LOG(INFO) << "reference line ok";

  double plan_length = 70;

  // prepare sdf
  OccupancyMap occupancy_map;
  occupancy_map.set_origin({0, -7.5});
  occupancy_map.set_cell_number(std::array<int, 2>{1100, 150});
  occupancy_map.set_resolution({0.1, 0.1});

  occupancy_map.FillConvexPoly(
      vector_Eigen2d{Eigen::Vector2d(30, -3), Eigen::Vector2d(35, -3),
                     Eigen::Vector2d(35, 7.5), Eigen::Vector2d(30, 7.5)});
  occupancy_map.FillConvexPoly(
      vector_Eigen2d{Eigen::Vector2d(44.5, 3), Eigen::Vector2d(50, 3),
                     Eigen::Vector2d(50, -7.5), Eigen::Vector2d(44.5, -7.5)});
  occupancy_map.FillConvexPoly(
      vector_Eigen2d{Eigen::Vector2d(60, -2), Eigen::Vector2d(65, -2),
                     Eigen::Vector2d(65, 7.5), Eigen::Vector2d(60, 7.5)});
  auto sdf = std::make_shared<SignedDistanceField2D>(std::move(occupancy_map));
  cv::imshow("hah", sdf->occupancy_map().BinaryImage());
  cv::waitKey(0);
  sdf->UpdateSDF();

  std::vector<double> hint{32.5, 47.5, 62.5};
  GPIncrementalPathPlanner path_planner(sdf);
  // GPPathOptimizer path_planner(sdf);

  common::State initial_state;
  initial_state.position = Eigen::Vector2d(20.0, 0);
  initial_state.heading = 0.0;
  initial_state.velocity = 17.5;
  initial_state.acceleration = 0.0;
  initial_state.stamp = 0.0;

  common::FrenetState frenet_state;
  reference_line.ToFrenetState(initial_state, &frenet_state);
  // frenet_state.d = Eigen::Vector3d(0, 0, 0);
  // frenet_state.s = Eigen::Vector3d(10, 0, 0);

  LOG(INFO) << "1";
  GPPath gp_path;
  if (!path_planner.GenerateInitialGPPath(reference_line, frenet_state,
                                          plan_length, hint, &gp_path)) {
    LOG(ERROR) << "gp planner failed";
    return -1;
  }
  LOG(INFO) << "gp path ok";

  // prepare obstacle
  std::vector<Obstacle> dynamic_obstacle;

  StGraph st_graph;
  std::vector<double> locations, kappa_limit;
  st_graph.SetInitialState(
      Eigen::Vector3d(1, initial_state.velocity, initial_state.acceleration));
  st_graph.SetReferenceSpeed(17.5);
  st_graph.BuildStGraph(dynamic_obstacle, gp_path);
  std::vector<StNode> st_nodes;
  st_graph.SearchWithLocalTruncation(13, &st_nodes);
  if (!st_graph.GenerateInitialSpeedProfile(gp_path)) {
    LOG(ERROR) << "generate initial speed profile failed";
    return -1;
  }
  // st_graph.VisualizeStGraph();

  // std::string file_path =
  //     "/home/kin/gpir_ws/src/gpir/gp_planner/data/"
  //     "incremental_data.csv";
  // std::ofstream writer = std::ofstream(file_path, std::ios::trunc);

  // common::DotLog(writer, "iteration", "t", "x", "y", "v", "lat_a", "lat_v",
  // "k",
  //                "s", "nan");
  common::Trajectory traj;
  st_graph.GenerateTrajectory(reference_line, gp_path, &traj);

  // recording
  // std::string index = std::to_string(i);
  std::vector<double> t1, x1, y1, v1, lat_a1, lat_v1, k1, s;
  for (const auto& p : traj) {
    t1.emplace_back(p.stamp);
    x1.emplace_back(p.position.x());
    y1.emplace_back(p.position.y());
    v1.emplace_back(p.velocity);
    s.emplace_back(p.frenet_s[0]);
    lat_a1.emplace_back(p.frenet_d[2] * p.frenet_s[1] * p.frenet_s[1] +
                        p.frenet_d[1] * p.frenet_s[2]);
    lat_v1.emplace_back(p.frenet_d[1] * p.frenet_s[1]);
    k1.emplace_back(p.kappa);
    // common::DotLog(writer, i, t1.back(), x1.back(), y1.back(), v1.back(),
    //                lat_a1.back(), lat_v1.back(), k1.back(), s.back());
  }

  std::string index = "with";
  plt::subplot(4, 1, 1);
  plt::named_plot(index, x1, y1);
  plt::legend();
  plt::axis("equal");
  plt::subplot(4, 1, 2);
  plt::named_plot(index, t1, lat_a1);
  plt::legend();
  plt::subplot(4, 1, 3);
  plt::named_plot(index, t1, lat_v1);
  plt::subplot(4, 1, 4);
  plt::named_plot(index, s, k1);
  plt::legend();

  GPPath path2;
  path_planner.TmpTest(reference_line, frenet_state, plan_length, hint, &path2);
  st_graph.GenerateTrajectory(reference_line, path2, &traj);

  t1.clear(), x1.clear(), y1.clear(), v1.clear(), s.clear(), lat_a1.clear(),
      lat_v1.clear(), k1.clear();
  for (const auto& p : traj) {
    t1.emplace_back(p.stamp);
    x1.emplace_back(p.position.x());
    y1.emplace_back(p.position.y());
    v1.emplace_back(p.velocity);
    s.emplace_back(p.frenet_s[0]);
    lat_a1.emplace_back(p.frenet_d[2] * p.frenet_s[1] * p.frenet_s[1] +
                        p.frenet_d[1] * p.frenet_s[2]);
    lat_v1.emplace_back(p.frenet_d[1] * p.frenet_s[1]);
    k1.emplace_back(p.kappa);
    // common::DotLog(writer, i, t1.back(), x1.back(), y1.back(), v1.back(),
    //                lat_a1.back(), lat_v1.back(), k1.back(), s.back());
  }

  index = "without";
  plt::subplot(4, 1, 1);
  plt::named_plot(index, x1, y1);
  plt::legend();
  plt::axis("equal");
  plt::subplot(4, 1, 2);
  plt::named_plot(index, t1, lat_a1);
  plt::legend();
  plt::subplot(4, 1, 3);
  plt::named_plot(index, t1, lat_v1);
  plt::subplot(4, 1, 4);
  plt::named_plot(index, s, k1);
  plt::legend();

  plt::subplot(4, 1, 4);
  plt::plot(std::vector<double>{0, 100}, std::vector<double>{0.35, 0.35},
            "--k");

  plt::show();

  return 0;
}
