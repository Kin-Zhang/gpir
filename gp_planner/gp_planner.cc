/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include "gp_planner/gp_planner.h"

#include <glog/logging.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <visualization_msgs/MarkerArray.h>

#include "common/smoothing/osqp_spline2d_solver.h"
#include "common/utils/color_map.h"
#include "gp_planner/gp/gp_path_optimizer.h"
#include "gp_planner/st_plan/st_graph.h"
#include "planning_core/planning_common/planning_visual.h"

namespace planning {

void GPPlanner::Init() {
  ros::NodeHandle node;
  trajectory_pub_ =
      node.advertise<visualization_msgs::MarkerArray>("/gp_path", 1);
  critical_obstacle_pub_ =
      node.advertise<visualization_msgs::MarkerArray>("/critical_obstacles", 1);
  virtual_obstacle_pub_ =
      node.advertise<visualization_msgs::MarkerArray>("/virtual_obstacle", 1);
  target_lane_pub_ = node.advertise<visualization_msgs::MarkerArray>(
      "/behavior_target_lane", 1);
  virtual_obstacle_sub_ =
      node.subscribe("/add_virtual", 1, &GPPlanner::VirtualObstacleSub, this);

  vehicle_param_ = VehicleInfo::Instance().vehicle_param();
}

// void GPPlanner::PlanOnce(NavigationMap* navigation_map_) {
//   // * build sdf
//   const auto& reference_line = navigation_map_->reference_line();
//   const double length = reference_line.length();

//   ProcessObstacles(navigation_map_->obstacles(), reference_line);

//   if (add_virtual_obstacle_) {
//     if (virtual_obstacles_.empty()) {
//       Eigen::Vector2d pos;
//       LOG(INFO) << navigation_map_->ego_state().DebugString();
//       reference_line.FrenetToCartesion(65, 1, &pos);
//       LOG(INFO) << pos;
//       virtual_obstacles_.emplace_back(pos);
//     }
//     add_virtual_obstacle_ = false;
//   }

//   if (!virtual_obstacles_.empty()) {
//     LOG(WARNING) << "has virtual";
//     auto proj = reference_line.GetProjection(virtual_obstacles_.front());
//     LOG(INFO) << proj.DebugString();
//     if (proj.s < 0 || proj.s > length || fabs(proj.d) > 4) {
//       virtual_obstacles_.clear();
//     }
//   }

//   OccupancyMap occupancy_map;
//   occupancy_map.set_origin({0, -5});
//   occupancy_map.set_cell_number(
//       std::array<int, 2>{(int)std::ceil(length / 0.1), 100});
//   occupancy_map.set_resolution({0.1, 0.1});
//   common::FrenetPoint proj;
//   if (!virtual_obstacles_.empty()) {
//     proj = reference_line.GetProjection(virtual_obstacles_.front());
//     occupancy_map.FillCircle(Eigen::Vector2d(proj.s, proj.d), 0.2);
//   } else {
//     proj.s = -1;
//   }

//   auto sdf =
//   std::make_shared<SignedDistanceField2D>(std::move(occupancy_map));
//   sdf->UpdateSDF();

//   GPPathOptimizer gp_path_optimizer;
//   gp_path_optimizer.set_sdf(sdf);

//   common::State start_state = navigation_map_->ego_state();
//   const auto& last_trajectory = navigation_map_->trajectory();

//   auto output_traj = navigation_map_->mutable_trajectory();

//   if (!last_trajectory.empty()) {
//     start_state =
//         last_trajectory.GetNearestState(navigation_map_->ego_state().position);
//   }

//   navigation_map_->mutable_trajectory()->clear();

//   // delete this
//   // start_state.debug(0) = 4;
//   // start_state.position = static_obstacles_.front().state().position;

//   const double ref_v = navigation_map_->reference_speed();
//   GPPath gp_path;
//   if (!gp_path_optimizer.GenerateGPPath(reference_line, start_state, 90,
//   proj.s,
//                                         navigation_map_->mutable_trajectory(),
//                                         &gp_path)) {
//     LOG(ERROR) << "path failed";
//     return;
//   }

//   common::FrenetState frenet_state;
//   reference_line.ToFrenetState(start_state, &frenet_state);
//   frenet_state.s[1] = start_state.frenet_s[1];
//   frenet_state.s[2] = start_state.frenet_s[2];
//   StGraph st_graph(frenet_state.s);
//   LOG(INFO) << frenet_state.DebugString();
//   st_graph.SetReferenceSpeed(ref_v);
//   st_graph.BuildStGraph(dynamic_obstacles_, gp_path);
//   if (!st_graph.LocalTopSearch(13, nullptr)) {
//     return;
//   }
//   if (!st_graph.OptimizeTest()) {
//     st_graph.VisualizeStGraph();
//     return;
//   }
//   st_graph.GenerateTrajectory(reference_line, gp_path,
//                               navigation_map_->mutable_trajectory());
//   // st_graph.VisualizeStGraph();
//   // auto trajectory = navigation_map_->mutable_trajectory();
//   // for (size_t i = 0; i < trajectory->size(); ++i) {
//   //   (*trajectory)[i].velocity = ref_v;
//   //   (*trajectory)[i].acceleration = 0.0;
//   // }
//   // LOG(INFO) << navigation_map_->trajectory();

//   // * visual
//   visualization_msgs::MarkerArray markers;
//   visualization_msgs::Marker line, node;

//   line.header.frame_id = "map";
//   line.header.stamp = ros::Time::now();
//   line.id = 0;
//   line.type = visualization_msgs::Marker::LINE_STRIP;
//   line.action = visualization_msgs::Marker::MODIFY;
//   line.color = common::ColorMap::at(common::Color::kOrangeRed).toRosMsg();
//   line.pose.orientation.w = 1;
//   line.scale.x = line.scale.y = line.scale.z = 0.15;

//   node.header = line.header;
//   node.id = 1;
//   node.type = visualization_msgs::Marker::SPHERE_LIST;
//   node.action = visualization_msgs::Marker::MODIFY;
//   node.color = common::ColorMap::at(common::Color::kBlack).toRosMsg();
//   node.pose.orientation.w = 1;
//   node.scale.x = node.scale.y = node.scale.z = 0.2;

//   geometry_msgs::Point point;
//   for (const auto& p : navigation_map_->trajectory()) {
//     point.x = p.position.x();
//     point.y = p.position.y();
//     point.z = 0.1;
//     // LOG(INFO) << p.x() << ", " << p.y();
//     line.points.emplace_back(point);
//     node.points.emplace_back(point);
//   }
//   markers.markers.emplace_back(line);
//   markers.markers.emplace_back(node);

//   // delete this
//   Eigen::Vector3d d;
//   common::State tmp;
//   int count = 2;
//   for (double i = gp_path.start_s(); i < 100; i += 5) {
//     gp_path.GetInterpolateNode(i, &d);
//     reference_line.FrenetToState(i, d, &tmp);
//     // visualization_msgs::Marker marker;
//     // marker.id = count++;
//     visualization_msgs::Marker marker_ego;
//     marker_ego.id = count++;
//     // if (count >= 5) {
//     //   PlanningVisual::GetPlannerBoxMarker(tmp.position, 2.16, 4.8,
//     //   tmp.heading,
//     //                                       common::Color::kOrange,
//     &marker);
//     // } else {
//     //   PlanningVisual::GetPlannerBoxMarker(tmp.position, 2.16, 4.8,
//     //   tmp.heading,
//     //                                       common::Color::kRoyalBlue,
//     //                                       &marker);
//     // }
//     auto ref = reference_line.GetFrenetReferncePoint(i);
//     PlanningVisual::GetPlannerBoxMarker(tmp.position, 2.16, 4.8, tmp.heading,
//                                         common::Color::kGrey, &marker_ego);
//     // markers.markers.emplace_back(marker);
//     markers.markers.emplace_back(marker_ego);
//   }

//   trajectory_pub_.publish(markers);

//   VisualizeVirtualObstacle();
// }

void GPPlanner::PlanOnce(NavigationMap* navigation_map_) {
  // * build sdf
  const auto& reference_lines = navigation_map_->reference_lines();
  navigation_map_->mutable_trajectory()->clear();

  visualization_msgs::MarkerArray markers;
  int id_count = 0;

  std::vector<Obstacle> critical_obstacles;
  std::vector<std::pair<hdmap::LaneSegmentBehavior, common::Trajectory>>
      trajectory_candidate;

  for (const auto& reference_line : reference_lines) {
    ProcessObstacles(navigation_map_->obstacles(), reference_line,
                     &critical_obstacles);

    const double length = reference_line.length();

    OccupancyMap occupancy_map;
    occupancy_map.set_origin({0, -5});
    occupancy_map.set_cell_number(
        std::array<int, 2>{(int)std::ceil(length / 0.1), 100});
    occupancy_map.set_resolution({0.1, 0.1});
    common::FrenetPoint proj;
    if (!virtual_obstacles_.empty()) {
      proj = reference_line.GetProjection(virtual_obstacles_.front());
      occupancy_map.FillCircle(Eigen::Vector2d(proj.s, proj.d), 0.2);
    } else {
      proj.s = -1;
    }

    auto sdf =
        std::make_shared<SignedDistanceField2D>(std::move(occupancy_map));
    sdf->UpdateSDF();

    GPPathOptimizer gp_path_optimizer;
    gp_path_optimizer.set_sdf(sdf);

    common::State state = navigation_map_->ego_state();
    std::cout << state.DebugString();
    common::FrenetState frenet_state;
    if (last_behavior_ != reference_line.behavior()) {
      reference_line.ToFrenetState(state, &frenet_state);
      LOG(WARNING) << "recalculate planning start point";
    } else {
      if (!navigation_map_->trajectory().empty()) {
        state = navigation_map_->trajectory().GetNearestState(state.position);
        frenet_state.d = state.frenet_d;
        frenet_state.s = state.frenet_s;
        LOG(INFO) << "use last trajectory";
      } else {
        reference_line.ToFrenetState(state, &frenet_state);
        LOG(WARNING) << "not use last trajectory";
      }
    }

    const double ref_v = navigation_map_->reference_speed();
    GPPath gp_path;
    if (!gp_path_optimizer.GenerateGPPath(
            reference_line, frenet_state, 90, proj.s,
            navigation_map_->mutable_trajectory(), &gp_path)) {
      LOG(ERROR) << "GP path planning failed";
      return;
    }

    StGraph st_graph(frenet_state.s);
    LOG(INFO) << frenet_state.DebugString();
    st_graph.SetReferenceSpeed(ref_v);
    st_graph.BuildStGraph(critical_obstacles, gp_path);
    if (!st_graph.LocalTopSearch(13, nullptr)) {
      LOG(ERROR) << "ST graph search failed";
      return;
    }
    if (!st_graph.OptimizeTest()) {
      LOG(ERROR) << "Fail to optimize speed profile";
      st_graph.VisualizeStGraph();
      return;
    }
    trajectory_candidate.emplace_back();
    st_graph.GenerateTrajectory(reference_line, gp_path,
                                &trajectory_candidate.back().second);
    trajectory_candidate.back().first = reference_line.behavior();

    // * visual
    visualization_msgs::Marker line, node;

    line.header.frame_id = "map";
    line.header.stamp = ros::Time::now();
    line.id = id_count++;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.action = visualization_msgs::Marker::MODIFY;
    line.color = common::ColorMap::at(common::Color::kOrangeRed).toRosMsg();
    line.pose.orientation.w = 1;
    line.scale.x = line.scale.y = line.scale.z = 0.15;
    line.lifetime = ros::Duration(0.15);

    node.header = line.header;
    node.id = id_count++;
    node.type = visualization_msgs::Marker::SPHERE_LIST;
    node.action = visualization_msgs::Marker::MODIFY;
    node.color = common::ColorMap::at(common::Color::kBlack).toRosMsg();
    node.pose.orientation.w = 1;
    node.scale.x = node.scale.y = node.scale.z = 0.2;
    node.lifetime = ros::Duration(0.15);

    geometry_msgs::Point point;
    for (const auto& p : trajectory_candidate.back().second) {
      point.x = p.position.x();
      point.y = p.position.y();
      point.z = 0.1;
      // LOG(INFO) << p.x() << ", " << p.y();
      line.points.emplace_back(point);
      node.points.emplace_back(point);
    }
    markers.markers.emplace_back(line);
    markers.markers.emplace_back(node);
  }
  *navigation_map_->mutable_trajectory() = trajectory_candidate.front().second;
  navigation_map_->SetPlannerLCFeedback(trajectory_candidate.front().first !=
                                        hdmap::LaneSegmentBehavior::kKeep);
  last_behavior_ = trajectory_candidate.front().first;

  trajectory_pub_.publish(markers);
  VisualizeTargetLane(
      reference_lines[reference_lines.size() - trajectory_candidate.size()]);
  VisualizeVirtualObstacle();
  VisualizeCriticalObstacle(critical_obstacles);
}

bool GPPlanner::ProcessObstacles(const std::vector<Obstacle>& raw_obstacles,
                                 const ReferenceLine& reference_line,
                                 std::vector<Obstacle>* cirtical_obstacles) {
  cirtical_obstacles->clear();
  ego_box_ = common::Box2D(state_.position, vehicle_param_.length,
                           vehicle_param_.width, state_.heading,
                           vehicle_param_.height);
  const auto& ref_lane_list = reference_line.lane_id_list();

  for (const auto& obstacle : raw_obstacles) {
    if (ego_box_.HasOverlapWith(obstacle.BoundingBox())) {
      LOG(ERROR) << "collision detected!";
      return false;
    }
    // TODO: use better filter
    std::vector<int> intersect_lane;
    const auto& obstacle_lane_list = obstacle.lane_id_list();
    std::set_intersection(obstacle_lane_list.begin(), obstacle_lane_list.end(),
                          ref_lane_list.begin(), ref_lane_list.end(),
                          std::back_inserter(intersect_lane));

    auto proj = reference_line.GetRoughProjection(obstacle.state().position);

    bool outside_roi = proj.s < -30 || proj.s > reference_line.length() ||
                       std::fabs(proj.d) > lateral_critical_thereshold_;
    bool intersect_safe = intersect_lane.empty();

    if (outside_roi && intersect_safe) continue;
    LOG(WARNING) << "critical obstacle " << obstacle.id();
    if (!obstacle.is_static()) {
      cirtical_obstacles->emplace_back(obstacle);
    }
  }
  return true;
}

void GPPlanner::VisualizeCriticalObstacle(
    const std::vector<Obstacle>& critical_obstacles) {
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker marker;
  marker.lifetime = ros::Duration(0.15);

  geometry_msgs::Point p;
  int id_count = 0;
  for (const auto& obs : critical_obstacles) {
    PlanningVisual::GetPlannerBoxMarker(obs.state().position, obs.width() + 0.3,
                                        obs.length() + 0.3, obs.state().heading,
                                        common::Color::kRed, &marker);
    marker.id = id_count++;
    markers.markers.emplace_back(marker);
  }

  critical_obstacle_pub_.publish(markers);
}

void GPPlanner::VisualizeTargetLane(const ReferenceLine& reference_line) {
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::MODIFY;
  marker.pose.orientation.w = 1.0;

  constexpr double step_length = 5.0;
  constexpr double angle = M_PI_4;
  double length = 0.75 / std::sin(angle);

  auto rotate_vector = [&length](const Eigen::Vector3d& base,
                                 const double angle) {
    geometry_msgs::Point point;
    auto rotated_vector =
        base.topRows(2) + length * Eigen::Vector2d(std::cos(base(2) + angle),
                                                   std::sin(base(2) + angle));
    point.x = rotated_vector.x();
    point.y = rotated_vector.y();
    return point;
  };

  marker.id = 0;
  for (double s = 0; s < reference_line.length(); s += step_length) {
    auto bh = reference_line.behavior();
    if (bh == hdmap::LaneSegmentBehavior::kKeep) {
      PlanningVisual::SetScaleAndColor({0.4, 0, 0}, common::Color::kGreen,
                                       &marker);
    } else {
      PlanningVisual::SetScaleAndColor({0.4, 0, 0}, common::Color::kMagenta,
                                       &marker);
    }
    auto se2 = reference_line.GetSE2(s);
    geometry_msgs::Point origin;
    origin.x = se2.x();
    origin.y = se2.y();
    auto left = rotate_vector(se2, M_PI - angle);
    marker.points.push_back(origin);
    marker.points.push_back(left);
    auto right = rotate_vector(se2, -M_PI + angle);
    marker.points.push_back(origin);
    marker.points.push_back(right);
  }
  markers.markers.push_back(marker);

  target_lane_pub_.publish(markers);
}

void GPPlanner::VisualizeVirtualObstacle() {
  visualization_msgs::MarkerArray markers;
  for (size_t i = 0; i < virtual_obstacles_.size(); ++i) {
    visualization_msgs::Marker marker;
    PlanningVisual::GetTrafficConeMarker(virtual_obstacles_[i], i, &marker);
    markers.markers.emplace_back(marker);
  }
  virtual_obstacle_pub_.publish(markers);
}

}  // namespace planning
