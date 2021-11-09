/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/*
 * @file
 */

#include "gp_planner/benchmark/TDR_OBCA/interface.h"

#include <future>

#include "common/utils/timer.h"
#include "gp_planner/benchmark/TDR_OBCA/dual_variable_warm_start_problem.h"
#include "gp_planner/benchmark/TDR_OBCA/file.h"

namespace {
double InterpolateUsingLinearApproximation(const double p0, const double p1,
                                           const double w) {
  return p0 * (1.0 - w) + p1 * w;
}

std::vector<double> VectorLinearInterpolation(const std::vector<double>& x,
                                              int extend_size) {
  // interplation example:
  // x: [x0, x1, x2], extend_size: 3
  // output: [y0(x0), y1, y2, y3(x1), y4, y5, y6(x2)]
  size_t origin_last = x.size() - 1;
  std::vector<double> res(origin_last * extend_size + 1, 0.0);

  for (size_t i = 0; i < origin_last * extend_size; ++i) {
    size_t idx0 = i / extend_size;
    size_t idx1 = idx0 + 1;
    double w =
        static_cast<double>(i % extend_size) / static_cast<double>(extend_size);
    res[i] = InterpolateUsingLinearApproximation(x[idx0], x[idx1], w);
  }

  res.back() = x.back();
  return res;
}
}  // namespace

namespace apollo {
namespace planning {

//////////////////////////////////////////////////////////////////////////////

bool ObstacleContainer::VPresentationObstacle(
    const size_t obstacle_num, const Eigen::MatrixXi& edge_num_each_obs,
    const double* ROI_distance_approach_parking_boundary) {
  obstacles_num_ = obstacle_num;
  obstacles_edges_num_ = edge_num_each_obs;

  size_t index = 0;
  for (size_t i = 0; i < obstacles_num_; i++) {
    std::vector<common::math::Vec2d> vertices_cw;  // one obstacle vertexs
    for (int j = 0; j < obstacles_edges_num_(i, 0) + 1; j++) {
      // extract x, y for one vertex
      common::math::Vec2d vertice = common::math::Vec2d(
          ROI_distance_approach_parking_boundary[index],
          ROI_distance_approach_parking_boundary[index + 1]);
      index += 2;
      vertices_cw.emplace_back(vertice);
    }
    obstacles_vertices_vec_.emplace_back(vertices_cw);
  }
  return true;
}

bool ObstacleContainer::HPresentationObstacle() {
  obstacles_A_ = Eigen::MatrixXd::Zero(obstacles_edges_num_.sum(), 2);
  obstacles_b_ = Eigen::MatrixXd::Zero(obstacles_edges_num_.sum(), 1);
  // vertices using H-representation
  if (!ObsHRep(obstacles_num_, obstacles_edges_num_, obstacles_vertices_vec_,
               &obstacles_A_, &obstacles_b_)) {
    std::cout << "Fail to present obstacle in hyperplane \n ";
    return false;
  }
  return true;
}

bool ObstacleContainer::ObsHRep(
    const size_t obstacles_num, const Eigen::MatrixXi& obstacles_edges_num,
    const std::vector<std::vector<common::math::Vec2d>>& obstacles_vertices_vec,
    Eigen::MatrixXd* A_all, Eigen::MatrixXd* b_all) {
  if (obstacles_num != obstacles_vertices_vec.size()) {
    std::cout << "obstacles_num != obstacles_vertices_vec.size() \n ";
    return false;
  }

  A_all->resize(obstacles_edges_num.sum(), 2);
  b_all->resize(obstacles_edges_num.sum(), 1);

  int counter = 0;
  double kEpsilon = 1.0e-5;
  // start building H representation
  for (size_t i = 0; i < obstacles_num; ++i) {
    size_t current_vertice_num = obstacles_edges_num(i, 0);
    Eigen::MatrixXd A_i(current_vertice_num, 2);
    Eigen::MatrixXd b_i(current_vertice_num, 1);

    // take two subsequent vertices, and computer hyperplane
    for (size_t j = 0; j < current_vertice_num; ++j) {
      common::math::Vec2d v1 = obstacles_vertices_vec[i][j];
      common::math::Vec2d v2 = obstacles_vertices_vec[i][j + 1];

      Eigen::MatrixXd A_tmp(2, 1), b_tmp(1, 1), ab(2, 1);
      // find hyperplane passing through v1 and v2
      if (std::abs(v1.x() - v2.x()) < kEpsilon) {
        if (v2.y() < v1.y()) {
          A_tmp << 1, 0;
          b_tmp << v1.x();
        } else {
          A_tmp << -1, 0;
          b_tmp << -v1.x();
        }
      } else if (std::abs(v1.y() - v2.y()) < kEpsilon) {
        if (v1.x() < v2.x()) {
          A_tmp << 0, 1;
          b_tmp << v1.y();
        } else {
          A_tmp << 0, -1;
          b_tmp << -v1.y();
        }
      } else {
        Eigen::MatrixXd tmp1(2, 2);
        tmp1 << v1.x(), 1, v2.x(), 1;
        Eigen::MatrixXd tmp2(2, 1);
        tmp2 << v1.y(), v2.y();
        ab = tmp1.inverse() * tmp2;
        double a = ab(0, 0);
        double b = ab(1, 0);

        if (v1.x() < v2.x()) {
          A_tmp << -a, 1;
          b_tmp << b;
        } else {
          A_tmp << a, -1;
          b_tmp << -b;
        }
      }

      // store vertices
      A_i.block(j, 0, 1, 2) = A_tmp.transpose();
      b_i.block(j, 0, 1, 1) = b_tmp;
    }

    A_all->block(counter, 0, A_i.rows(), 2) = A_i;
    b_all->block(counter, 0, b_i.rows(), 1) = b_i;
    counter += static_cast<int>(current_vertice_num);
  }
  return true;
}

void ObstacleContainer::AddObstacle(
    const size_t obstacle_num, const Eigen::MatrixXi& edge_num_each_obs,
    const double* ROI_distance_approach_parking_boundary) {
  TIC;
  if (!(VPresentationObstacle(obstacle_num, edge_num_each_obs,
                              ROI_distance_approach_parking_boundary) &&
        HPresentationObstacle())) {
    std::cout << "obstacle presentation fails;  \n ";
  } else {
    std::cout << "[ObstacleContainer]: add obstacle successfully. \n";
  }
  TOC("ObstacleContainer::AddObs");
}

//////////////////////////////////////////////////////////////////////////////

bool DistanceSmoothing(
    const apollo::planning::PlannerOpenSpaceConfig& planner_open_space_config,
    const ObstacleContainer& obstacles, double sx, double sy, double sphi,
    double ex, double ey, double ephi, const std::vector<double>& XYbounds,
    HybridAStartResult* hybrid_a_star_result, Eigen::MatrixXd* state_result_ds_,
    Eigen::MatrixXd* control_result_ds_, Eigen::MatrixXd* time_result_ds_,
    Eigen::MatrixXd* dual_l_result_ds_, Eigen::MatrixXd* dual_n_result_ds_,
    double& dual_time, double& ipopt_time) {
  // load Warm Start result(horizon is the "N", not the size of step points)
  size_t horizon_ = hybrid_a_star_result->x.size() - 1;
  // nominal sampling time
  float ts_ = planner_open_space_config.delta_t();

  Eigen::VectorXd x;
  Eigen::VectorXd y;
  Eigen::VectorXd phi;
  Eigen::VectorXd v;
  Eigen::VectorXd steer;
  Eigen::VectorXd a;

  // TODO(Runxin): extend logics in future
  if (horizon_ <= 10 && horizon_ > 2 &&
      planner_open_space_config.enable_linear_interpolation()) {
    // TODO(Runxin): extend this number
    int extend_size = 5;
    // modify state and control vectors sizes
    horizon_ = extend_size * horizon_;
    // modify delta t
    ts_ = ts_ / static_cast<float>(extend_size);

    std::vector<double> x_extend =
        VectorLinearInterpolation(hybrid_a_star_result->x, extend_size);
    x = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(x_extend.data(),
                                                      horizon_ + 1);

    std::vector<double> y_extend =
        VectorLinearInterpolation(hybrid_a_star_result->y, extend_size);
    y = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(y_extend.data(),
                                                      horizon_ + 1);

    std::vector<double> phi_extend =
        VectorLinearInterpolation(hybrid_a_star_result->phi, extend_size);
    phi = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(phi_extend.data(),
                                                        horizon_ + 1);

    std::vector<double> v_extend =
        VectorLinearInterpolation(hybrid_a_star_result->v, extend_size);
    v = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(v_extend.data(),
                                                      horizon_ + 1);

    steer = Eigen::VectorXd(horizon_);
    a = Eigen::VectorXd(horizon_);
    for (size_t i = 0; i < static_cast<size_t>(horizon_); ++i) {
      steer[i] = hybrid_a_star_result->steer[i / extend_size];
      a[i] = hybrid_a_star_result->a[i / extend_size];
    }

    // ADEBUG << "hybrid A x: ";
    // for (size_t i = 0; i < hybrid_a_star_result->x.size(); ++i) {
    //   ADEBUG << "i: " << i << ", val: " << hybrid_a_star_result->x[i];
    // }
    // ADEBUG << "interpolated x: \n" << x;

    // ADEBUG << "hybrid A steer: ";
    // for (size_t i = 0; i < hybrid_a_star_result->steer.size(); ++i) {
    //   ADEBUG << "i: " << i << ", val: " << hybrid_a_star_result->steer[i];
    // }
    // ADEBUG << "interpolated steer: \n" << steer;
  } else {
    x = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        hybrid_a_star_result->x.data(), horizon_ + 1);
    y = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        hybrid_a_star_result->y.data(), horizon_ + 1);
    phi = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        hybrid_a_star_result->phi.data(), horizon_ + 1);
    v = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        hybrid_a_star_result->v.data(), horizon_ + 1);
    steer = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        hybrid_a_star_result->steer.data(), horizon_);
    a = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        hybrid_a_star_result->a.data(), horizon_);
  }

  Eigen::MatrixXd xWS = Eigen::MatrixXd::Zero(4, horizon_ + 1);
  Eigen::MatrixXd uWS = Eigen::MatrixXd::Zero(2, horizon_);
  xWS.row(0) = x;
  xWS.row(1) = y;
  xWS.row(2) = phi;
  xWS.row(3) = v;
  uWS.row(0) = steer;
  uWS.row(1) = a;

  Eigen::MatrixXd x0(4, 1);
  x0 << sx, sy, sphi, 0.0;

  Eigen::MatrixXd xF(4, 1);
  xF << ex, ey, ephi, 0.0;

  Eigen::MatrixXd last_time_u(2, 1);
  last_time_u << 0.0, 0.0;

  // load vehicle configuration
  Eigen::MatrixXd ego_(4, 1);
  double front_to_center = BenchmarkVehicleConfig::front_to_center;
  double back_to_center = BenchmarkVehicleConfig::back_to_center;
  double left_to_center = BenchmarkVehicleConfig::left_to_center;
  double right_to_center = BenchmarkVehicleConfig::right_to_center;
  ego_ << front_to_center, right_to_center, back_to_center, left_to_center;

  // result for distance approach problem
  Eigen::MatrixXd l_warm_up;
  Eigen::MatrixXd n_warm_up;
  Eigen::MatrixXd s_warm_up =
      Eigen::MatrixXd::Zero(obstacles.GetObstaclesNum(), horizon_ + 1);

  // const auto t1 = std::chrono::system_clock::now();

  DualVariableWarmStartProblem* dual_variable_warm_start_ptr =
      new DualVariableWarmStartProblem(planner_open_space_config);
  if (BenchmarkMehtodConfig::use_dual_variable_warm_start) {
    bool dual_variable_warm_start_status = dual_variable_warm_start_ptr->Solve(
        horizon_, ts_, ego_, obstacles.GetObstaclesNum(),
        obstacles.GetObstaclesEdgesNum(), obstacles.GetAMatrix(),
        obstacles.GetbMatrix(), xWS, &l_warm_up, &n_warm_up, &s_warm_up);

    if (dual_variable_warm_start_status) {
      AINFO << "Dual variable problem solved successfully!";
    } else {
      AERROR << "Dual variable problem solving failed";
      return false;
    }
  } else {
    l_warm_up = Eigen::MatrixXd::Zero(obstacles.GetObstaclesEdgesNum().sum(),
                                      horizon_ + 1);
    n_warm_up =
        Eigen::MatrixXd::Zero(4 * obstacles.GetObstaclesNum(), horizon_ + 1);
  }

  // const auto t2 = std::chrono::system_clock::now();
  // dual_time = std::chrono::duration<double>(t2 - t1).count() * 1000;

  DistanceApproachProblem* distance_approach_ptr =
      new DistanceApproachProblem(planner_open_space_config);

  bool status = distance_approach_ptr->Solve(
      x0, xF, last_time_u, horizon_, ts_, ego_, xWS, uWS, l_warm_up, n_warm_up,
      s_warm_up, XYbounds, obstacles.GetObstaclesNum(),
      obstacles.GetObstaclesEdgesNum(), obstacles.GetAMatrix(),
      obstacles.GetbMatrix(), state_result_ds_, control_result_ds_,
      time_result_ds_, dual_l_result_ds_, dual_n_result_ds_);
  delete distance_approach_ptr;  // new added

  // const auto t3 = std::chrono::system_clock::now();
  // ipopt_time = std::chrono::duration<double>(t3 - t2).count() * 1000;

  if (!status) {
    std::cout << "[ERROR] Distance fail \n";
    return false;
  }
  return true;
}

extern "C" {
bool TDROBCAPlanner::DistancePlan(HybridAStartResult* hybrid_astar_result,
                                  ObstacleContainer* obstacles_ptr,
                                  ResultContainer* result_ptr, double sx,
                                  double sy, double sphi, double ex, double ey,
                                  double ephi, double* XYbounds) {
  TIC;

  apollo::planning::PlannerOpenSpaceConfig planner_open_space_config_;
  if (!apollo::cyber::common::GetProtoFromFile(
          BenchmarkMehtodConfig::tdr_obca_config_path,
          &planner_open_space_config_)) {
    std::cout << "[DistancePlan]: fail to load, path= \n"
              << BenchmarkMehtodConfig::tdr_obca_config_path << std::endl;
    return false;
  }

  // AINFO << "FLAGS_planner_open_space_config_filename: "
  //       << FLAGS_planner_open_space_config_filename;
  std::cout << "[DistancePlan]: start \n";

  double hybrid_total = 0.0;
  double dual_total = 0.0;
  double ipopt_total = 0.0;

  // std::string flag_file_path = "/apollo/modules/planning/conf/planning.conf";
  // google::SetCommandLineOption("flagfile", flag_file_path.c_str());

  std::vector<double> XYbounds_(XYbounds, XYbounds + 4);

  // input initial guess directly
  //
  // HybridAStartResult hybrid_astar_result;
  // if (!hybridA_ptr->Plan(sx, sy, sphi, ex, ey, ephi, XYbounds_,
  //                        obstacles_ptr->GetObstacleVec(),
  //                        &hybrid_astar_result)) {
  //   std::cout << "Hybrid A Star fails" << std::endl;
  //   return false;
  // }

  // if (Enable_parallel_trajectory_smoothing) {
  //   std::vector<HybridAStartResult> partition_trajectories;
  //   if (!hybridA_ptr->TrajectoryPartition(hybrid_astar_result,
  //                                         &partition_trajectories)) {
  //     return false;
  //   }
  //   size_t size = partition_trajectories.size();
  //   std::vector<Eigen::MatrixXd> state_result_ds_vec;
  //   std::vector<Eigen::MatrixXd> control_result_ds_vec;
  //   std::vector<Eigen::MatrixXd> time_result_ds_vec;
  //   std::vector<Eigen::MatrixXd> dual_l_result_ds_vec;
  //   std::vector<Eigen::MatrixXd> dual_n_result_ds_vec;
  //   state_result_ds_vec.resize(size);
  //   control_result_ds_vec.resize(size);
  //   time_result_ds_vec.resize(size);
  //   dual_l_result_ds_vec.resize(size);
  //   dual_n_result_ds_vec.resize(size);
  //   std::vector<std::future<bool>> results;

  //   // In for loop
  //   double dual_tmp = 0.0;
  //   double ipopt_tmp = 0.0;
  //   for (size_t i = 0; i < size; ++i) {
  //     double piece_wise_sx = partition_trajectories[i].x.front();
  //     double piece_wise_sy = partition_trajectories[i].y.front();
  //     double piece_wise_sphi = partition_trajectories[i].phi.front();
  //     double piece_wise_ex = partition_trajectories[i].x.back();
  //     double piece_wise_ey = partition_trajectories[i].y.back();
  //     double piece_wise_ephi = partition_trajectories[i].phi.back();

  //     if (planner_open_space_config_.enable_check_parallel_trajectory()) {
  //       std::cout << "trajectory idx: " << i << std::endl;
  //       std::cout << "trajectory pt number: "
  //                 << partition_trajectories[i].x.size() << std::endl;
  //     }

  //     if (!DistanceSmoothing(planner_open_space_config_, *obstacles_ptr,
  //                            piece_wise_sx, piece_wise_sy, piece_wise_sphi,
  //                            piece_wise_ex, piece_wise_ey, piece_wise_ephi,
  //                            XYbounds_, &partition_trajectories[i],
  //                            &state_result_ds_vec[i],
  //                            &control_result_ds_vec[i],
  //                            &time_result_ds_vec[i],
  //                            &dual_l_result_ds_vec[i],
  //                            &dual_n_result_ds_vec[i], dual_tmp, ipopt_tmp))
  //                            {
  //       return false;
  //     }
  //     dual_total += dual_tmp;
  //     ipopt_total += ipopt_tmp;
  //   }

  //   // Retrieve result in one single trajectory
  //   size_t trajectory_point_size = 0;
  //   for (size_t i = 0; i < size; ++i) {
  //     if (state_result_ds_vec[i].cols() < 2) {
  //       std::cout << "[ERROR] state horizon smaller than 2";
  //       return false;
  //     }
  //     std::cout << "trajectory idx: "
  //               << "idx range: " << trajectory_point_size << ", "
  //               << trajectory_point_size +
  //                      static_cast<size_t>(state_result_ds_vec[i].cols()) - 1
  //               << std::endl;
  //     trajectory_point_size +=
  //         static_cast<size_t>(state_result_ds_vec[i].cols()) - 1;
  //   }
  //   ++trajectory_point_size;

  //   const uint64_t state_dimension = state_result_ds_vec.front().rows();
  //   Eigen::MatrixXd state_result_ds;
  //   state_result_ds.resize(state_dimension, trajectory_point_size);
  //   uint64_t k = 0;
  //   for (size_t i = 0; i < size; ++i) {
  //     // leave out the last repeated point so set column minus one
  //     uint64_t state_col_num = state_result_ds_vec[i].cols() - 1;
  //     for (uint64_t j = 0; j < state_col_num; ++j) {
  //       state_result_ds.col(k) = state_result_ds_vec[i].col(j);
  //       ++k;
  //     }
  //   }
  //   state_result_ds.col(k) =
  //       state_result_ds_vec.back().col(state_result_ds_vec.back().cols() -
  //       1);

  //   const uint64_t control_dimension = control_result_ds_vec.front().rows();
  //   Eigen::MatrixXd control_result_ds;
  //   control_result_ds.resize(control_dimension, trajectory_point_size - 1);
  //   k = 0;

  //   for (size_t i = 0; i < size; ++i) {
  //     uint64_t control_col_num = control_result_ds_vec[i].cols() - 1;
  //     for (uint64_t j = 0; j < control_col_num; ++j) {
  //       control_result_ds.col(k) = control_result_ds_vec[i].col(j);
  //       ++k;
  //     }
  //   }

  //   const uint64_t time_dimension = time_result_ds_vec.front().rows();
  //   Eigen::MatrixXd time_result_ds;
  //   time_result_ds.resize(time_dimension, trajectory_point_size - 1);
  //   k = 0;
  //   for (size_t i = 0; i < size; ++i) {
  //     uint64_t time_col_num = time_result_ds_vec[i].cols() - 1;
  //     for (uint64_t j = 0; j < time_col_num; ++j) {
  //       time_result_ds.col(k) = time_result_ds_vec[i].col(j);
  //       ++k;
  //     }
  //   }

  //   *(result_ptr->PrepareHybridAResult()) = hybrid_astar_result;
  //   *(result_ptr->PrepareStateResult()) = state_result_ds;
  //   *(result_ptr->PrepareControlResult()) = control_result_ds;
  //   *(result_ptr->PrepareTimeResult()) = time_result_ds;
  //   *(result_ptr->GetHybridTime()) = 0.;  // hybrid_total
  //   *(result_ptr->GetDualTime()) = dual_total;
  //   *(result_ptr->GetIpoptTime()) = ipopt_total;
  // } else
  {
    Eigen::MatrixXd state_result_ds;
    Eigen::MatrixXd control_result_ds;
    Eigen::MatrixXd time_result_ds;
    Eigen::MatrixXd dual_l_result_ds;
    Eigen::MatrixXd dual_n_result_ds;
    if (!DistanceSmoothing(planner_open_space_config_, *obstacles_ptr, sx, sy,
                           sphi, ex, ey, ephi, XYbounds_, hybrid_astar_result,
                           &state_result_ds, &control_result_ds,
                           &time_result_ds, &dual_l_result_ds,
                           &dual_n_result_ds, dual_total, ipopt_total)) {
      return false;
    }
    *(result_ptr->PrepareHybridAResult()) = *hybrid_astar_result;
    *(result_ptr->PrepareStateResult()) = state_result_ds;
    *(result_ptr->PrepareControlResult()) = control_result_ds;
    *(result_ptr->PrepareTimeResult()) = time_result_ds;
    *(result_ptr->PrepareLResult()) = dual_l_result_ds;
    *(result_ptr->PrepareNResult()) = dual_n_result_ds;
    *(result_ptr->GetHybridTime()) = 0.;  // hybrid_total
    *(result_ptr->GetDualTime()) = dual_total;
    *(result_ptr->GetIpoptTime()) = ipopt_total;
  }

  TOC("TDR_OBCA PLAN SUCCESS");
  return true;
}

};  // extern "C" {

void TDROBCAPlanner::DistanceGetResult(
    ResultContainer* result_ptr, ObstacleContainer* obstacles_ptr,
    std::vector<double>* opt_x, std::vector<double>* opt_y,
    std::vector<double>* opt_phi, std::vector<double>* opt_v,
    std::vector<double>* opt_a, std::vector<double>* opt_steer,
    std::vector<double>* opt_kappa) {
  result_ptr->LoadHybridAResult();
  size_t size = result_ptr->GetX()->size();
  size_t size_by_distance = result_ptr->PrepareStateResult()->cols();
  AERROR_IF(size != size_by_distance)
      << "sizes by hybrid A and distance approach not consistent";

  opt_x->clear();
  opt_y->clear();
  opt_phi->clear();
  opt_v->clear();
  opt_a->clear();
  opt_steer->clear();
  opt_kappa->clear();

  size_t obstacles_edges_sum = obstacles_ptr->GetObstaclesEdgesNum().sum();
  size_t obstacles_num_to_car = 4 * obstacles_ptr->GetObstaclesNum();
  for (size_t i = 0; i < size_by_distance; ++i) {
    opt_x->emplace_back((*(result_ptr->PrepareStateResult()))(0, i));
    opt_y->emplace_back((*(result_ptr->PrepareStateResult()))(1, i));
    opt_phi->emplace_back((*(result_ptr->PrepareStateResult()))(2, i));
    opt_v->emplace_back((*(result_ptr->PrepareStateResult()))(3, i));
  }

  // std::cout << "what\n";
  for (size_t i = 0; i + 1 < size_by_distance; ++i) {
    opt_a->emplace_back((*(result_ptr->PrepareControlResult()))(1, i));
    const double get_steer = (*(result_ptr->PrepareControlResult()))(0, i);
    opt_steer->emplace_back(get_steer);
    // std::cout << i << "]get_steer= " << get_steer << std::endl;
    const double get_kappa =
        std::tan(get_steer) / BenchmarkVehicleConfig::wheelbase;
    opt_kappa->emplace_back(get_kappa);
    // std::cout << i << "]get_kappa= " << get_kappa << std::endl;
  }
}

// void ResultContainer::GetAccumlatedDistAndCurvature(
//     std::vector<double>* array_s, std::vector<double>* array_k) const {
//   array_s->clear();
//   array_k->clear();
//   const size_t common_siz =
//       std::min(std::min(x_.size(), y_.size()), steer_.size());

//   double s_sum = 0.;
//   double last_x, last_y;
//   for (int i = 0; i < common_siz; i++) {
//     const double this_x = x_.at(i);
//     // const double this_y = y_.at(i);
//     // if (i > 0) {
//     //   const double dx = this_x - last_x;
//     //   const double dy = this_y - last_y;
//     //   s_sum += std::sqrt(dx * dx + dy * dy);
//     // }
//     // last_x = this_x;
//     // last_y = this_y;

//     const double get_k =
//         std::tan(steer_.at(i)) / BenchmarkVehicleConfig::wheelbase;
//     array_s->emplace_back(this_x);
//     array_k->emplace_back(get_k);
//   }
// }

}  // namespace planning
}  // namespace apollo
