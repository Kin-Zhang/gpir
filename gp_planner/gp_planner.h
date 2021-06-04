/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include <std_msgs/Bool.h>

#include "gp_planner/sdf/signed_distance_field_2d.h"
#include "planning_core/planner/planner.h"
#include "planning_core/planning_common/vehicle_info.h"

namespace planning {

class GPPlanner : public Planner {
 public:
  GPPlanner() = default;
  virtual ~GPPlanner() = default;

  void Init() override;
  void PlanOnce(NavigationMap* navigation_map_) override;

 private:
  bool ProcessObstacles(const std::vector<Obstacle>& raw_obstacles,
                        const ReferenceLine& reference_line,
                        std::vector<Obstacle>* cirtical_obstacles);

  void UpdateVirtualObstacles();

  void VisualizeCriticalObstacle();
  void VisualizeVirtualObstacle();

  void VirtualObstacleSub(const std_msgs::Bool& msg) {
    add_virtual_obstacle_ = true;
  }

 private:
  ros::Publisher trajectory_pub_;
  ros::Publisher critical_obstacle_pub_;
  ros::Publisher virtual_obstacle_pub_;
  ros::Subscriber virtual_obstacle_sub_;

  common::State state_;
  common::Box2D ego_box_;
  VehicleParam vehicle_param_;

  bool add_virtual_obstacle_ = false;
  const double lateral_critical_thereshold_ = 6;

  std::vector<Obstacle> static_obstacles_;
  std::vector<Obstacle> dynamic_obstacles_;
  vector_Eigen2d virtual_obstacles_;

  hdmap::LaneSegmentBehavior last_behavior_ = hdmap::LaneSegmentBehavior::kKeep;
};

}  // namespace planning
