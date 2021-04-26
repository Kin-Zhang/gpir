/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>

#include <chrono>
#include <thread>

#include "common/utils/timer.h"
#include "hdmap/hdmap.h"
#include "hdmap/matplotlibcpp.h"
#include "hdmap/utils/admap_visual.h"
#include "planning_core/navigation/navigation_map.h"
#include "planning_core/navigation/reference_line.h"

using namespace hdmap;

bool has_init_pose = false;
Eigen::Vector2d init, goal;
double start_heading, end_heading;

class Foo {
 public:
  Foo() {
    ros::NodeHandle node;
    init_sub_ =
        node.subscribe("/initialpose", 10, &Foo::InitPoseCallBack, this);
    goal_sub_ = node.subscribe("/move_base_simple/goal", 10,
                               &Foo::GoalPoseCallBack, this);
    ref_pub_ = node.advertise<visualization_msgs::MarkerArray>("/ref", 10);
  }

  void InitPoseCallBack(
      const geometry_msgs::PoseWithCovarianceStamped& init_pose) {
    spdlog::info("init pose received");
    has_init_pose = true;
    init.x() = init_pose.pose.pose.position.x;
    init.y() = init_pose.pose.pose.position.y;
    start_heading = tf::getYaw(init_pose.pose.pose.orientation);
  }

  void GoalPoseCallBack(const geometry_msgs::PoseStamped& goal_pose) {
    if (!has_init_pose) {
      spdlog::warn("goal pose received, but do not have a init pose");
      return;
    }
    spdlog::info("goal pose received");
    goal.x() = goal_pose.pose.position.x;
    goal.y() = goal_pose.pose.position.y;
    end_heading = tf::getYaw(goal_pose.pose.orientation);
    FullRoute full_route;
    HdMap::GetMap().CreateFullRoute(init, goal, start_heading, end_heading,
                                    &full_route);
    planning::NavigationMap rs(full_route);
    common::State state;
    state.position = init;
    rs.SelectRouteSequence(state);
    std::vector<WayPoint> waypoints, refs;
    rs.GetReferenceWaypoints(state, 100, 20, &waypoints);
    std::vector<double> x,y;
    for (const auto& p : waypoints) {
      x.emplace_back(p.point.x());
      y.emplace_back(p.point.y());
    }

    namespace plt = matplotlibcpp;
    plt::plot(x,y);
    plt::axis("equal");
    plt::show();

    planning::ReferenceLine ref_line;
    ref_line.GenerateReferenceLine(waypoints);
    std::cout << waypoints << std::endl;

    for (const auto& p : waypoints) {
      WayPoint pp;
      pp.point = ref_line.GetReferencePoint(p.s);
      pp.s = p.s;
      refs.emplace_back(pp);
    }
    std::cout << "\n" << refs << std::endl;

    visualization_msgs::MarkerArray markers;
    AdMapVisual::WaypointsToMarkerArray(refs, &markers);
    ref_pub_.publish(markers);
  }

 private:
  ros::Subscriber init_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher ref_pub_;
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "hdmap");
  ros::NodeHandle node;

  Foo foo;

  // wait for rviz initialization
  spdlog::info("wait for rviz initialization");
  // std::this_thread::sleep_for(std::chrono::seconds(1));
  // HdMap::GetMap().LoadMap(argv[1], argv[2]);
  // TODO: why rviz does not show anything unless load twice?
  std::this_thread::sleep_for(std::chrono::seconds(1));
  HdMap::GetMap().LoadMap(argv[1], argv[2]);
  ros::Rate rate(10);
  while (ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}
