/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include "common/utils/io_utils.h"
#include "gp_planner/benchmark/DL_IAPS/coarse_trajectory_generator/hybrid_a_star.h"
#include "gp_planner/benchmark/DL_IAPS/discrete_points_math.h"
#include "gp_planner/benchmark/DL_IAPS/iterative_anchoring_smoother.h"
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
    "/home/udi/research/gpir_ws/src/gpir/gp_planner/data/"
    "curvature_benchmark.csv";
std::ofstream logger;

void LogPathData(std::string planner, const std::vector<double>& x,
                 const std::vector<double>& y, const std::vector<double>& s,
                 const std::vector<double>& k) {
  for (int i = 0; i < x.size(); ++i) {
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
  auto vehicle_param = VehicleInfo::Instance().mutable_vehicle_param();
  vehicle_param->length = 4.79178;
  vehicle_param->width = 2.16345;
  vehicle_param->height = 1.48832;
  vehicle_param->wheel_base = 3.00464;
  vehicle_param->front_to_front_axle = 0.777665;
  vehicle_param->back_to_rear_axle = 1.00947;
  vehicle_param->rear_axle_to_center = 1.38642;

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

  std::vector<vector_Eigen2d> obstacles{
      {Eigen::Vector2d(30, -4), Eigen::Vector2d(35, -4),
       Eigen::Vector2d(35, 7.5), Eigen::Vector2d(30, 7.5)},
      {Eigen::Vector2d(44.5, 3), Eigen::Vector2d(50, 3),
       Eigen::Vector2d(50, -7.5), Eigen::Vector2d(44.5, -7.5)},
      {Eigen::Vector2d(60, -2), Eigen::Vector2d(65, -2),
       Eigen::Vector2d(65, 7.5), Eigen::Vector2d(60, 7.5)}};

  for (const auto obstacle : obstacles) {
    occupancy_map.FillConvexPoly(obstacle);
    std::vector<double> vertex_x, vertex_y;
    for (int i = 0; i <= obstacle.size(); ++i) {
      const auto& p = obstacle[i % obstacle.size()];
      vertex_x.emplace_back(p.x());
      vertex_y.emplace_back(p.y());
    }
    plt::subplot(2, 1, 1);
    plt::plot(vertex_x, vertex_y);
  }

  auto sdf = std::make_shared<SignedDistanceField2D>(std::move(occupancy_map));
  cv::imshow("hah", sdf->occupancy_map().BinaryImage());
  cv::waitKey(0);
  sdf->UpdateSDF();

  std::vector<double> hint{32.5, 47.5, 62.5};

  common::State initial_state;
  initial_state.position = Eigen::Vector2d(20.0, 0);
  initial_state.heading = 0.0;
  initial_state.velocity = 10;
  initial_state.acceleration = 0.0;
  initial_state.stamp = 0.0;

  common::FrenetState frenet_state;
  reference_line.ToFrenetState(initial_state, &frenet_state);

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

  std::vector<common::State> samples;
  path_without_curvature_constraint.GetSamplePathPoints(0.2, &samples);

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
  }
  LogPathData("GPNoCur", x, y, s, k);

  plt::subplot(2, 1, 1);
  plt::named_plot("without", x, y);
  plt::legend();
  plt::subplot(2, 1, 2);
  plt::named_plot("without", s, k);
  plt::legend();

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
  LogPathData("DLIAPS", x, y, s, k);

  plt::subplot(2, 1, 1);
  plt::named_plot("dl-iaps", x, y);
  plt::legend();
  plt::axis("equal");
  plt::subplot(2, 1, 2);
  plt::named_plot("dl-iaps", s, k);
  plt::legend();

  GPPath path_with_curvature_constraint;
  path_planner.set_enable_curvature_constraint(true);
  path_planner.GenerateInitialGPPath(reference_line, frenet_state, plan_length,
                                     hint, &path_without_curvature_constraint);

  path_without_curvature_constraint.GetSamplePathPoints(0.2, &samples);
  x.clear(), y.clear(), s.clear(), k.clear();
  for (int i = 0; i < samples.size(); ++i) {
    x.emplace_back(samples[i].position.x());
    y.emplace_back(samples[i].position.y());
    s.emplace_back(samples[i].s);
    k.emplace_back(samples[i].kappa);
  }
  LogPathData("GPCur", x, y, s, k);
  logger.close();

  plt::subplot(2, 1, 1);
  plt::named_plot("with", x, y);
  plt::legend();
  plt::subplot(2, 1, 2);
  plt::named_plot("with", s, k);
  plt::legend();

  plt::show();
  return 0;
}
