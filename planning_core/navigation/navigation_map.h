/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <unordered_set>

#include "common/base/state.h"
#include "common/base/trajectory.h"
#include "hdmap/routing/full_route.h"
#include "planning_core/navigation/reference_line.h"
#include "planning_core/navigation/route_sequence.h"
#include "planning_core/planning_common/data_frame.h"

namespace planning {

class NavigationMap {
 public:
  NavigationMap() = default;
  NavigationMap(const hdmap::FullRoute fullroute)
      : full_route_(new hdmap::FullRoute(fullroute)) {}

  void Init();

  void Update(std::shared_ptr<DataFrame> data_frame);

  bool HasActiveTask();

  bool RandomlyUpdateRoute();

  bool SuggestLaneChange(const hdmap::LaneSegmentBehavior type);

  bool CreateTask(const geometry_msgs::PoseStamped& goal_pose);

  bool CreateTask(const Eigen::Vector2d& goal_pos, const double goal_heading);

  bool UpdateReferenceLine();

  //  private:
  bool SelectRouteSequence(const common::State& state);

  void GetReferenceWaypoints(const common::State& state,
                             const RouteSequence& route_sequence,
                             const double forward, const double backward,
                             std::vector<hdmap::WayPoint>* waypoints,
                             std::vector<int>* lane_id_list);

  void GenerateWaypoints(const common::State& state, const double forward,
                         const double backward, int target_lane,
                         int forward_lane_hint, int backward_lane_hint,
                         std::vector<hdmap::WayPoint>* waypoints,
                         std::vector<int>* lane_id_list);

  Eigen::Vector2d GetPoint(const double s);

  const common::State& ego_state() const { return data_frame_->state; }
  const ReferenceLine& reference_line() const { return reference_line_; }
  const std::vector<ReferenceLine>& reference_lines() const {
    return reference_lines_;
  }
  inline const double reference_speed() const { return refernce_speed_; }
  const std::vector<Obstacle>& obstacles() const {
    return data_frame_->obstacles;
  }

  const common::Trajectory& trajectory() const { return trajectory_; }
  common::Trajectory* mutable_trajectory() { return &trajectory_; }

 private:
  std::vector<RouteSequence*> GetRouteCandidate();
  bool UpdateRouteSequence(RouteSequence* route_sequence);
  void AddLaneToRouteSequence(
      const int lane_id, RouteSequence* route_sequence,
      hdmap::LaneSegmentBehavior type = hdmap::LaneSegmentBehavior::kKeep);

  void PublishReferenceLine();
  void PublishRouteSequence();

 private:
  ros::Publisher reference_line_pub_;
  ros::Publisher route_sequence_pub_;

  double refernce_speed_ = 0.0;
  ReferenceLine reference_line_;
  std::vector<ReferenceLine> reference_lines_;
  common::Trajectory trajectory_;

  bool in_lane_changing_ = false;

  std::shared_ptr<DataFrame> data_frame_ = nullptr;
  std::unique_ptr<hdmap::FullRoute> full_route_ = nullptr;
  std::unique_ptr<RouteSequence> route_sequence_ = nullptr;
  std::unique_ptr<RouteSequence> route_sequence_lane_change_ = nullptr;
};
}  // namespace planning
