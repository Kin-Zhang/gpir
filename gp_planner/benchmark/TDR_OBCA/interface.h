/*
 * @file
 */

#pragma once

#include <Eigen/Dense>
#include <coin-or/IpIpoptApplication.hpp>
#include <coin-or/IpSolveStatistics.hpp>
#include <vector>

#include "gp_planner/benchmark/DL_IAPS/coarse_trajectory_generator/hybrid_a_star.h"
#include "gp_planner/benchmark/DL_IAPS/vec2d.h"
#include "gp_planner/benchmark/TDR_OBCA/distance_approach_problem.h"

// #include "planning_core/planning_common/vehicle_info.h"
//  using proto instead

namespace apollo {
namespace planning {

// ObstacleContainer
class ObstacleContainer {
 public:
  ObstacleContainer() = default;

  // example: 4 obstacles, edge num are
  //   3, 2, 3, 2 = 10, with
  //      ROI_distance_approach_parking_boundary[20]
  //        10 x 2 = 20, {[x, y]x10, } (clock wise order)
  // obstacles_num_ = 4;  // 4 obstacles
  // obstacles_edges_num_.resize(4, 1);
  // obstacles_edges_num_ << 3, 2, 3, 2;
  //    第一个障碍物有2条边,第二个1条... (既几条线段连续组成)
  void AddObstacle(const size_t obstacle_num,
                   const Eigen::MatrixXi& edge_num_each_obs,
                   const double* ROI_distance_approach_parking_boundary);

  const std::vector<std::vector<common::math::Vec2d>>& GetObstacleVec() const {
    return obstacles_vertices_vec_;
  }
  const Eigen::MatrixXd& GetAMatrix() const { return obstacles_A_; }
  const Eigen::MatrixXd& GetbMatrix() const { return obstacles_b_; }
  size_t GetObstaclesNum() const { return obstacles_num_; }
  const Eigen::MatrixXi& GetObstaclesEdgesNum() const {
    return obstacles_edges_num_;
  }

 private:
  bool VPresentationObstacle(
      const size_t obstacle_num, const Eigen::MatrixXi& edge_num_each_obs,
      const double* ROI_distance_approach_parking_boundary);
  bool HPresentationObstacle();
  bool ObsHRep(const size_t obstacles_num,
               const Eigen::MatrixXi& obstacles_edges_num,
               const std::vector<std::vector<common::math::Vec2d>>&
                   obstacles_vertices_vec,
               Eigen::MatrixXd* A_all, Eigen::MatrixXd* b_all);

 private:
  size_t obstacles_num_ = 0;
  Eigen::MatrixXi obstacles_edges_num_;
  std::vector<std::vector<common::math::Vec2d>> obstacles_vertices_vec_;
  Eigen::MatrixXd obstacles_A_;
  Eigen::MatrixXd obstacles_b_;
};

// ResultContainer
class ResultContainer {
 public:
  ResultContainer() = default;
  void LoadHybridAResult() {
    x_ = std::move(result_.x);
    y_ = std::move(result_.y);
    phi_ = std::move(result_.phi);
    v_ = std::move(result_.v);
    a_ = std::move(result_.a);
    steer_ = std::move(result_.steer);
  }

  std::vector<double>* GetX() { return &x_; }
  std::vector<double>* GetY() { return &y_; }
  std::vector<double>* GetPhi() { return &phi_; }
  std::vector<double>* GetV() { return &v_; }
  std::vector<double>* GetA() { return &a_; }
  std::vector<double>* GetSteer() { return &steer_; }
  HybridAStartResult* PrepareHybridAResult() { return &result_; }
  Eigen::MatrixXd* PrepareStateResult() { return &state_result_ds_; }
  Eigen::MatrixXd* PrepareControlResult() { return &control_result_ds_; }
  Eigen::MatrixXd* PrepareTimeResult() { return &time_result_ds_; }
  Eigen::MatrixXd* PrepareLResult() { return &dual_l_result_ds_; }
  Eigen::MatrixXd* PrepareNResult() { return &dual_n_result_ds_; }
  double* GetHybridTime() { return &hybrid_time_; }
  double* GetDualTime() { return &dual_time_; }
  double* GetIpoptTime() { return &ipopt_time_; }

 private:
  HybridAStartResult result_;
  std::vector<double> x_;
  std::vector<double> y_;
  std::vector<double> phi_;
  std::vector<double> v_;
  std::vector<double> a_;
  std::vector<double> steer_;
  Eigen::MatrixXd state_result_ds_;
  Eigen::MatrixXd control_result_ds_;
  Eigen::MatrixXd time_result_ds_;
  Eigen::MatrixXd dual_l_result_ds_;
  Eigen::MatrixXd dual_n_result_ds_;
  double hybrid_time_;
  double dual_time_;
  double ipopt_time_;
};

class TDROBCAPlanner {
 public:
  static bool DistancePlan(
      HybridAStartResult* hybrid_astar_result,  // input with legal values
      ObstacleContainer* obstacles_ptr,         // input
      ResultContainer* result_ptr,              // output
      double sx, double sy, double sphi,        // inputs start point
      double ex, double ey, double ephi,        // inputs end point
      double* XYbounds  // input, [x_l, x_u, y_l, y_u], siz = 4
  );

  static void DistanceGetResult(
      ResultContainer* result_ptr, ObstacleContainer* obstacles_ptr,
      std::vector<double>* opt_x, std::vector<double>* opt_y,
      std::vector<double>* opt_phi, std::vector<double>* opt_v,
      std::vector<double>* opt_a, std::vector<double>* opt_steer,
      std::vector<double>* opt_kappa);
};

}  // namespace planning
}  // namespace apollo
