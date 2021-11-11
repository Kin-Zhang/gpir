/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include <iostream>

#include "common/utils/io_utils.h"
#include "common/utils/math.h"
#include "common/utils/timer.h"
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
#include "tqdm/tqdm.h"

using namespace planning;
using namespace apollo::planning;

namespace plt = matplotlibcpp;
using apollo::common::math::Vec2d;
using common::RandomDouble;

static const double kappa_limit = 0.2;
static bool plot = false;

std::string save_path =
    "/home/cj/research/gpir_ws/src/gpir/gp_planner/data/"
    "curvature_benchmark-1.csv";
std::ofstream info_logger;
std::ofstream result_logger;
std::ofstream env_logger;

void LogPathData(std::string planner, const std::vector<double>& x,
                 const std::vector<double>& y, const std::vector<double>& s,
                 const std::vector<double>& k) {
  for (int i = 0; i < k.size(); ++i) {
    common::DotLog(result_logger, planner, x[i], y[i], s[i], k[i]);
  }
}

void LogEnv(const std::vector<vector_Eigen2d>& obs) {
  for (const auto& obj : obs) {
    common::DotLog(env_logger, obj[3].x(), obj[3].y(), obj[1].x(), obj[1].y());
  }
}

void LogInfo(const std::string planner, const int success, const double time) {
  common::DotLog(info_logger, planner, success, time);
}

bool IsCurvatureValiad(const double limit, const std::vector<double>& k) {
  for (const double value : k) {
    if (std::fabs(value) > limit * 1.05) {
      return false;
    }
  }
  return true;
}

int main(int argc, char const* argv[]) {
  google::InstallFailureSignalHandler();
  google::InitGoogleLogging(argv[0]);
  google::SetStderrLogging(google::ERROR);
  FLAGS_colorlogtostderr = true;

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

  const int total_trial = 1000;
  int total_num = 0;
  int g3p_cur_success = 0, dl_iaps_success = 0, tdr_obca_success = 0;

  std::string info_path =
      "/home/cj/research/gpir_ws/src/gpir/gp_planner/data/info/";
  std::string results_path =
      "/home/cj/research/gpir_ws/src/gpir/gp_planner/data/results/";
  std::string env_path =
      "/home/cj/research/gpir_ws/src/gpir/gp_planner/data/env/";

  tqdm bar;
  for (int i = 500; i < total_trial; ++i) {
    bar.progress(i, total_trial);
    OccupancyMap occupancy_map;
    occupancy_map.set_origin({0, -7.5});
    occupancy_map.set_cell_number(std::array<int, 2>{1100, 150});
    occupancy_map.set_resolution({0.1, 0.1});

    double delta_obs1_x = 1.5 * RandomDouble(0, 1);
    double delta_obs1_y = 1.5 * RandomDouble(0, 1);
    double delta_obs2_x = 1.5 * RandomDouble(0, 1);
    double delta_obs2_y = 1.5 * RandomDouble(0, 1);
    double delta_obs3_x = 1.5 * RandomDouble(0, 1);
    double delta_obs3_y = 1.5 * RandomDouble(0, 1);
    // printf("%f,%f,%f\n", delta_obs1_x, delta_obs2_x, delta_obs3_x);
    // printf("%f,%f,%f\n", delta_obs1_y, delta_obs2_y, delta_obs3_y);

    // obstacles x 3
    const size_t k_obstacle_num = 3;
    std::vector<vector_Eigen2d> obstacles{
        {Eigen::Vector2d(30 + delta_obs1_x, -4 + delta_obs1_y),
         Eigen::Vector2d(35 + delta_obs1_x, -4 + delta_obs1_y),
         Eigen::Vector2d(35 + delta_obs1_x, 7.5 + delta_obs1_y),
         Eigen::Vector2d(30 + delta_obs1_x, 7.5 + delta_obs1_y)},
        {Eigen::Vector2d(44.5 + delta_obs2_x, 3 + delta_obs2_y),
         Eigen::Vector2d(50 + delta_obs2_x, 3 + delta_obs2_y),
         Eigen::Vector2d(50 + delta_obs2_x, -7.5 + delta_obs2_y),
         Eigen::Vector2d(44.5 + delta_obs2_x, -7.5 + delta_obs2_y)},
        {Eigen::Vector2d(60 + delta_obs3_x, -2 + delta_obs3_y),
         Eigen::Vector2d(65 + delta_obs3_x, -2 + delta_obs3_y),
         Eigen::Vector2d(65 + delta_obs3_x, 7.5 + delta_obs3_y),
         Eigen::Vector2d(60 + delta_obs3_x, 7.5 + delta_obs3_y)}};

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
        30 + delta_obs1_x,   -4 + delta_obs1_y,   35 + delta_obs1_x,
        -4 + delta_obs1_y,   35 + delta_obs1_x,   7.5 + delta_obs1_y,
        30 + delta_obs1_x,   7.5 + delta_obs1_y,  //
        44.5 + delta_obs2_x, -7.5 + delta_obs2_y, 50 + delta_obs2_x,
        -7.5 + delta_obs2_y, 50 + delta_obs2_x,   3 + delta_obs2_y,
        44.5 + delta_obs2_x, 3 + delta_obs2_y,  //
        60 + delta_obs3_x,   -2 + delta_obs3_y,   65 + delta_obs3_x,
        -2 + delta_obs3_y,   65 + delta_obs3_x,   7.5 + delta_obs3_y,
        60 + delta_obs3_x,   7.5 + delta_obs3_y,  //
    };

    for (const auto obstacle : obstacles) {
      occupancy_map.FillConvexPoly(obstacle);
      std::vector<double> vertex_x, vertex_y;
      for (int i = 0; i <= obstacle.size(); ++i) {
        const auto& p = obstacle[i % obstacle.size()];
        vertex_x.emplace_back(p.x());
        vertex_y.emplace_back(p.y());
      }

      if (plot) {
        plt::subplot(2, 1, 1);
        plt::plot(vertex_x, vertex_y);
      }
    }

    auto sdf =
        std::make_shared<SignedDistanceField2D>(std::move(occupancy_map));
    // cv::imshow("SDF-Rviz", sdf->occupancy_map().BinaryImage());
    // cv::waitKey(0);
    sdf->UpdateSDF();

    std::vector<double> hint{32.5, 47.5, 62.5};

    common::State initial_state;
    initial_state.position = Eigen::Vector2d(20.0, 0);
    initial_state.heading = RandomDouble(-0.1, 0.1);
    initial_state.velocity = 10;
    initial_state.acceleration = 0.0;
    initial_state.stamp = 0.0;

    common::FrenetState frenet_state;
    reference_line.ToFrenetState(initial_state, &frenet_state);

    // method GPNoCur
    GPPath path_without_curvature_constraint;
    GPIncrementalPathPlanner path_planner(sdf);
    path_planner.set_enable_curvature_constraint(false);
    path_planner.set_enable_incremental_refinement(false);
    if (!path_planner.GenerateInitialGPPath(
            reference_line, frenet_state, plan_length, hint,
            &path_without_curvature_constraint)) {
      LOG(ERROR) << "gp planner without curvature constraints -- failed";
      continue;
      // return -1;
    }
    ++total_num;

    // init logger
    info_logger =
        std::ofstream(info_path + std::to_string(i) + ".csv", std::ios::trunc);
    env_logger =
        std::ofstream(env_path + std::to_string(i) + ".csv", std::ios::trunc);
    result_logger = std::ofstream(results_path + std::to_string(i) + ".csv",
                                  std::ios::trunc);
    common::DotLog(result_logger, "planner", "x", "y", "theta", "k");
    common::DotLog(env_logger, "left_top_x", "left_top_y", "right_bottom_x",
                   "right_bottom_y");
    common::DotLog(info_logger, "planner", "success", "time");
    LogEnv(obstacles);

    HybridAStartResult initial_guess_obca;
    std::vector<common::State> samples;
    path_without_curvature_constraint.GetSamplePathPoints(0.2, &samples);

    std::vector<double> x, y, k, theta;
    Eigen::MatrixXd xWs(3, samples.size());
    for (int i = 0; i < samples.size(); ++i) {
      xWs(0, i) = samples[i].position.x();
      xWs(1, i) = samples[i].position.y();
      xWs(2, i) = samples[i].heading;
      x.emplace_back(samples[i].position.x());
      y.emplace_back(samples[i].position.y());
      theta.emplace_back(samples[i].heading);
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
    LogPathData("GPNoCur", x, y, theta, k);

    if (plot) {
      plt::subplot(2, 1, 1);
      plt::named_plot("G3P", x, y);
      plt::legend();
      plt::subplot(2, 1, 2);
      plt::named_plot("G3P", x, k);
      plt::legend();
    }
    // method DL-IAPS
    std::vector<std::vector<Vec2d>> obstacles_vertices_vec;
    for (const auto& obstacle : obstacles) {
      std::vector<Vec2d> obstacle_vertices;
      for (const auto& point : obstacle) {
        obstacle_vertices.emplace_back(Vec2d(point.x(), point.y()));
      }
      obstacles_vertices_vec.emplace_back(obstacle_vertices);
    }

    common::Timer timer;

    IterativeAnchoringSmoother dl_iaps_smoother;
    DiscretizedPath dl_iaps_path;

    timer.Start();
    bool dl_iaps_status =
        dl_iaps_smoother.Smooth(xWs, obstacles_vertices_vec, &dl_iaps_path);
    double dl_iaps_time = timer.End();

    if (dl_iaps_status) {
      // ++dl_iaps_success;
      // LOG(ERROR) << "DL-IAPS smoothing solved";
      // return -1;

      std::vector<double> dkappa, headings;
      std::vector<std::pair<double, double>> discrete_points;
      x.clear(), y.clear(), theta.clear(), k.clear();
      for (int i = 0; i < dl_iaps_path.size(); ++i) {
        discrete_points.emplace_back(
            std::make_pair(dl_iaps_path[i].x(), dl_iaps_path[i].y()));
        x.emplace_back(dl_iaps_path[i].x());
        y.emplace_back(dl_iaps_path[i].y());
        theta.emplace_back(dl_iaps_path[i].theta());
      }
      std::vector<double> tmp_s;
      DiscretePointsMath::ComputePathProfile(discrete_points, &headings, &tmp_s,
                                             &k, &dkappa);
      int success = 0;
      if (IsCurvatureValiad(kappa_limit, k)) {
        ++dl_iaps_success;
        success = 1;
        // std::cout << "iaps ok" << std::endl;
      }
      LogPathData("DLIAPS", x, y, theta, k);
      LogInfo("DLIAPS", success, dl_iaps_time);

      if (plot) {
        plt::subplot(2, 1, 1);
        plt::named_plot("DL-IAPS", x, y);
        plt::legend();
        plt::axis("equal");
        plt::subplot(2, 1, 2);
        plt::named_plot("DL-IAPS", x, k);
        plt::legend();
      }
    }

    // method GP-Cur   a
    GPPath path_with_curvature_constraint;
    path_planner.set_enable_curvature_constraint(true);
    path_planner.set_enable_incremental_refinement(true);

    timer.Start();
    bool gp_cur_status = path_planner.GenerateInitialGPPath(
        reference_line, frenet_state, plan_length, hint,
        &path_with_curvature_constraint);
    double gp_cur_time = timer.End();

    if (gp_cur_status) {
      path_with_curvature_constraint.GetSamplePathPoints(0.2, &samples);
      x.clear(), y.clear(), theta.clear(), k.clear();
      for (int i = 0; i < samples.size(); ++i) {
        x.emplace_back(samples[i].position.x());
        y.emplace_back(samples[i].position.y());
        theta.emplace_back(samples[i].heading);
        k.emplace_back(samples[i].kappa);
      }

      int success = 0;
      if (IsCurvatureValiad(kappa_limit, k)) {
        ++g3p_cur_success;
        success = 1;
      }
      LogPathData("GPCur", x, y, theta, k);
      LogInfo("GPCur", success, gp_cur_time);

      if (plot) {
        plt::subplot(2, 1, 1);
        plt::named_plot("G3P-Cur", x, y);
        plt::legend();
        plt::subplot(2, 1, 2);
        plt::named_plot("G3P-Cur", x, k);
        plt::legend();
      }
    }

    // method TDR-OBCA
    // HybridAStar hybrid_a_star;
    ObstacleContainer obs_container;
    ResultContainer tdr_obca_result;
    double XYbounds[4] = {0, 100, -15, 15};
    obs_container.AddObstacle(k_obstacle_num, edge_num_each_obs,
                              &obs_vertexs[0]);

    timer.Start();
    bool tdr_obca_status = TDROBCAPlanner::DistancePlan(
        &initial_guess_obca, &obs_container, &tdr_obca_result,
        initial_state.position.x(), initial_state.position.y(),
        initial_state.heading,  //
        90.0, 0, 0,             //
        &XYbounds[0]);
    double tdr_obca_time = timer.End();

    if (tdr_obca_status) {
      std::vector<double> opt_x, opt_y, opt_phi, opt_v, opt_a, opt_steer,
          opt_kappa;
      TDROBCAPlanner::DistanceGetResult(&tdr_obca_result, &obs_container,
                                        &opt_x, &opt_y, &opt_phi, &opt_v,
                                        &opt_a, &opt_steer, &opt_kappa);

      int success = 0;
      if (IsCurvatureValiad(kappa_limit, opt_kappa)) {
        ++tdr_obca_success;
        success = 1;
      }
      // std::cout << opt_x.size() << "," << opt_kappa.size() << std::endl;
      LogPathData("TDROBCA", opt_x, opt_y, opt_phi, opt_kappa);
      LogInfo("TDROBCA", success, tdr_obca_time);
      // logger.close();

      if (plot) {
        plt::subplot(2, 1, 1);
        plt::named_plot("TDR-OBCA", opt_x, opt_y);
        plt::legend();
        plt::subplot(2, 1, 2);
        opt_x.resize(opt_kappa.size());
        plt::named_plot("TDR-OBCA", opt_x, opt_kappa);
        plt::legend();
      }
    }

    // plt::show
    if (plot) {
      plt::subplot(2, 1, 1);
      plt::xlim(0, 90);
      plt::subplot(2, 1, 2);
      plt::xlim(0, 90);
      plt::show();
    }

    result_logger.close();
    env_logger.close();
    info_logger.close();
  }

  printf("total experiment num: %d\n", total_num);
  printf("G3P-Cur success rate: %f%\n",
         (double)g3p_cur_success / (double)total_num * 100);
  printf("DL-IAPS success rate: %f%\n",
         (double)dl_iaps_success / (double)total_num * 100);
  printf("TDR-OBCA success rate: %f%\n",
         (double)tdr_obca_success / (double)total_num * 100);

  return 0;
}
