/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include <iostream>

#include "common/utils/io_utils.h"
#include "gp_planner/benchmark/DL_IAPS/coarse_trajectory_generator/hybrid_a_star.h"
#include "gp_planner/benchmark/DL_IAPS/discrete_points_math.h"
#include "gp_planner/benchmark/DL_IAPS/iterative_anchoring_smoother.h"
#include "gp_planner/benchmark/TDR_OBCA/interface.h"
#include "gp_planner/gp/gp_incremental_path_planner.h"
#include "gp_planner/gp/gp_path_optimizer.h"
#include "gp_planner/gp/utils/bounded_penalty_function.h"
#include "gp_planner/st_plan/st_graph.h"
#include "gp_planner/thirdparty/matplotlib-cpp/matplotlibcpp.h"
#include "planning_core/navigation/reference_line.h"
#include "planning_core/planning_common/obstacle.h"
#include "planning_core/planning_common/vehicle_info.h"

using namespace planning;
using namespace apollo::planning;

namespace plt = matplotlibcpp;
using apollo::common::math::Vec2d;

std::string save_path =
    "/home/kin/research/gpir_ws/src/gpir/gp_planner/data/"
    "curvature_benchmark.csv";
std::ofstream logger;

void LogPathData(std::string planner, const std::vector<double>& x,
                 const std::vector<double>& y, const std::vector<double>& s,
                 const std::vector<double>& k) {
  for (int i = 0; i < k.size(); ++i) {
    std::cout << "?" << std::endl;
    common::DotLog(logger, planner, x[i], y[i], s[i], k[i]);
  }
}

int main(int argc, char const* argv[]) {
  google::InstallFailureSignalHandler();
  google::InitGoogleLogging(argv[0]);
  google::SetStderrLogging(google::INFO);
  FLAGS_colorlogtostderr = true;

  logger = std::ofstream(save_path, std::ios::trunc);
  common::DotLog(logger, "planner", "x", "y", "s", "k");

  // prepare vehicle config
  //  notice, config of TDR_OBCA is defined inside TDR/OBCA's config.cc
  auto vehicle_param = VehicleInfo::Instance().mutable_vehicle_param();
  vehicle_param->length = BenchmarkVehicleConfig::length;
  vehicle_param->width = BenchmarkVehicleConfig::width;
  vehicle_param->height = BenchmarkVehicleConfig::height;
  vehicle_param->wheel_base = BenchmarkVehicleConfig::wheelbase;
  vehicle_param->front_to_front_axle =
      BenchmarkVehicleConfig::front_to_front_axle;
  vehicle_param->back_to_rear_axle = BenchmarkVehicleConfig::back_to_rear_axle;
  vehicle_param->rear_axle_to_center =
      BenchmarkVehicleConfig::rear_axle_to_center;

  // prepare reference_line
  std::vector<hdmap::WayPoint> waypoints;
  for (int i = 0; i <= 120; ++i) {
    hdmap::WayPoint waypoint;
    waypoint.point = Eigen::Vector2d(i, 0.0);
    waypoint.s = i;
    waypoint.heading = 0.0;
    waypoints.emplace_back(waypoint);
  }
  ReferenceLine reference_line;
  reference_line.GenerateReferenceLine(waypoints);

  double plan_length = 70;

  // prepare sdf
  OccupancyMap occupancy_map;
  occupancy_map.set_origin({0, -7.5});
  occupancy_map.set_cell_number(std::array<int, 2>{1100, 150});
  occupancy_map.set_resolution({0.1, 0.1});

  // obstacles x 3
  const size_t k_obstacle_num = 3;
  std::vector<vector_Eigen2d> obstacles{
      {Eigen::Vector2d(30, -4), Eigen::Vector2d(35, -4),
       Eigen::Vector2d(35, 7.5), Eigen::Vector2d(30, 7.5)},
      {Eigen::Vector2d(44.5, 3), Eigen::Vector2d(50, 3),
       Eigen::Vector2d(50, -7.5), Eigen::Vector2d(44.5, -7.5)},
      {Eigen::Vector2d(60, -2), Eigen::Vector2d(65, -2),
       Eigen::Vector2d(65, 7.5), Eigen::Vector2d(60, 7.5)}};

  // example: 4 obstacles, edge num are
  //   3, 2, 3, 2 = 10, with
  //      ROI_distance_approach_parking_boundary[20]
  //        10 x 2 = 20, {[x, y]x10, } (clock wise order)
  // obstacles_num_ = 4;  // 4 obstacles
  // obstacles_edges_num_.resize(4, 1);
  // obstacles_edges_num_ << 3, 2, 3, 2;
  //    第一个障碍物有2条边,第二个1条... (既几条线段连续组成)
  Eigen::MatrixXi edge_num_each_obs;
  edge_num_each_obs.resize(3, 1);
  edge_num_each_obs << 4, 4, 4;
  std::vector<double> obs_vertexs = {
      30,   -4,   35, -4,   35, 7.5, 30,   7.5,  //
      44.5, -7.5, 50, -7.5, 50, 3,   44.5, 3,    //
      60,   -2,   65, -2,   65, 7.5, 60,   7.5,  //
  };
  LOG(INFO) << "11";

  for (const auto obstacle : obstacles) {
    occupancy_map.FillConvexPoly(obstacle);
    std::vector<double> vertex_x, vertex_y;
    for (int i = 0; i <= obstacle.size(); ++i) {
      const auto& p = obstacle[i % obstacle.size()];
      vertex_x.emplace_back(p.x());
      vertex_y.emplace_back(p.y());
    }

    LOG(INFO) << "15";
    plt::subplot(2, 1, 1);
    LOG(INFO) << "16";
    plt::plot(vertex_x, vertex_y);
  }

  LOG(INFO) << "14";
  auto sdf = std::make_shared<SignedDistanceField2D>(std::move(occupancy_map));
  // cv::imshow("SDF-Rviz", sdf->occupancy_map().BinaryImage());
  // cv::waitKey(0);
  sdf->UpdateSDF();

  std::vector<double> hint{32.5, 47.5, 62.5};
  LOG(INFO) << "13";

  common::State initial_state;
  initial_state.position = Eigen::Vector2d(20.0, 0);
  initial_state.heading = 0.0;
  initial_state.velocity = 10;
  initial_state.acceleration = 0.0;
  initial_state.stamp = 0.0;

  common::FrenetState frenet_state;
  reference_line.ToFrenetState(initial_state, &frenet_state);

  LOG(INFO) << "11";
  // method GPNoCur
  GPPath path_without_curvature_constraint;
  GPIncrementalPathPlanner path_planner(sdf);
  path_planner.set_enable_curvature_constraint(false);
  path_planner.set_enable_incremental_refinement(false);
  if (!path_planner.GenerateInitialGPPath(reference_line, frenet_state,
                                          plan_length, hint,
                                          &path_without_curvature_constraint)) {
    LOG(ERROR) << "gp planner without curvature constraints -- failed";
    return -1;
  }

  HybridAStartResult initial_guess_obca;
  std::vector<common::State> samples;
  path_without_curvature_constraint.GetSamplePathPoints(0.2, &samples);

  LOG(INFO) << "12";
  std::vector<double> x, y, k, s;
  Eigen::MatrixXd xWs(3, samples.size());
  for (int i = 0; i < samples.size(); ++i) {
    xWs(0, i) = samples[i].position.x();
    xWs(1, i) = samples[i].position.y();
    xWs(2, i) = samples[i].heading;
    x.emplace_back(samples[i].position.x());
    y.emplace_back(samples[i].position.y());
    s.emplace_back(samples[i].s);
    k.emplace_back(samples[i].kappa);

    initial_guess_obca.x.emplace_back(samples[i].position.x());
    initial_guess_obca.y.emplace_back(samples[i].position.y());
    initial_guess_obca.phi.emplace_back(samples[i].heading);
    initial_guess_obca.v.emplace_back(1.);
    initial_guess_obca.a.emplace_back(0.);
    initial_guess_obca.steer.emplace_back(
        std::atan(BenchmarkVehicleConfig::wheelbase * samples[i].kappa));
    initial_guess_obca.accumulated_s.emplace_back(samples[i].s);
  }
  // LogPathData("GPNoCur", x, y, s, k);

  plt::subplot(2, 1, 1);
  plt::named_plot("G3P", x, y);
  plt::legend();
  plt::subplot(2, 1, 2);
  plt::named_plot("G3P", s, k);
  plt::legend();

  // method DL-IAPS
  std::vector<std::vector<Vec2d>> obstacles_vertices_vec;
  for (const auto& obstacle : obstacles) {
    std::vector<Vec2d> obstacle_vertices;
    for (const auto& point : obstacle) {
      obstacle_vertices.emplace_back(Vec2d(point.x(), point.y()));
    }
    obstacles_vertices_vec.emplace_back(obstacle_vertices);
  }

  IterativeAnchoringSmoother dl_iaps_smoother;
  DiscretizedPath dl_iaps_path;
  if (!dl_iaps_smoother.Smooth(xWs, obstacles_vertices_vec, &dl_iaps_path)) {
    LOG(ERROR) << "DL-IAPS smoothing failed";
    return -1;
  }

  std::vector<double> dkappa, headings;
  std::vector<std::pair<double, double>> discrete_points;
  x.clear(), y.clear(), s.clear(), k.clear();
  for (int i = 0; i < dl_iaps_path.size(); ++i) {
    discrete_points.emplace_back(
        std::make_pair(dl_iaps_path[i].x(), dl_iaps_path[i].y()));
    x.emplace_back(dl_iaps_path[i].x());
    y.emplace_back(dl_iaps_path[i].y());
  }
  DiscretePointsMath::ComputePathProfile(discrete_points, &headings, &s, &k,
                                         &dkappa);
  // LogPathData("DLIAPS", x, y, s, k);

  plt::subplot(2, 1, 1);
  plt::named_plot("DL-IAPS", x, y);
  plt::legend();
  plt::axis("equal");
  plt::subplot(2, 1, 2);
  plt::named_plot("DL-IAPS", s, k);
  plt::legend();

  // method GP-Cur
  GPPath path_with_curvature_constraint;
  path_planner.set_enable_curvature_constraint(true);
  path_planner.set_enable_incremental_refinement(true);
  path_planner.GenerateInitialGPPath(reference_line, frenet_state, plan_length,
                                     hint, &path_with_curvature_constraint);

  path_with_curvature_constraint.GetSamplePathPoints(0.2, &samples);
  x.clear(), y.clear(), s.clear(), k.clear();
  for (int i = 0; i < samples.size(); ++i) {
    x.emplace_back(samples[i].position.x());
    y.emplace_back(samples[i].position.y());
    s.emplace_back(samples[i].s);
    k.emplace_back(samples[i].kappa);
  }
  // LogPathData("GPCur", x, y, s, k);
  // logger.close();

  plt::subplot(2, 1, 1);
  plt::named_plot("G3P-Cur", x, y);
  plt::legend();
  plt::subplot(2, 1, 2);
  plt::named_plot("G3P-Cur", s, k);
  plt::legend();

  // method TDR-OBCA
  // HybridAStar hybrid_a_star;
  ObstacleContainer obs_container;
  ResultContainer tdr_obca_result;
  double XYbounds[4] = {0, 100, -15, 15};
  obs_container.AddObstacle(k_obstacle_num, edge_num_each_obs, &obs_vertexs[0]);

  const bool tdr_obca_sta = TDROBCAPlanner::DistancePlan(
      &initial_guess_obca, &obs_container, &tdr_obca_result,
      initial_state.position.x(), initial_state.position.y(),
      initial_state.heading,  //
      90.0, 0, 0,             //
      &XYbounds[0]);

  std::vector<double> opt_x, opt_y, opt_phi, opt_v, opt_a, opt_steer, opt_kappa;
  TDROBCAPlanner::DistanceGetResult(&tdr_obca_result, &obs_container, &opt_x,
                                    &opt_y, &opt_phi, &opt_v, &opt_a,
                                    &opt_steer, &opt_kappa);

  std::cout << opt_x.size() << "," << opt_kappa.size() << std::endl;
  LogPathData("TDROBCA", opt_x, opt_y, opt_x, opt_kappa);
  logger.close();

  plt::subplot(2, 1, 1);
  plt::named_plot("TDR-OBCA", opt_x, opt_y);
  plt::legend();
  plt::subplot(2, 1, 2);
  opt_x.resize(opt_kappa.size());
  plt::named_plot("TDR-OBCA", opt_x, opt_kappa);
  plt::legend();

  // plt::show
  plt::subplot(2, 1, 1);
  plt::xlim(0, 90);
  plt::subplot(2, 1, 2);
  plt::xlim(0, 90);
  plt::show();
  return 0;
}
