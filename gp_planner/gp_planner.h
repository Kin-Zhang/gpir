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

  void LogDebugInfo() override { save_snapshot_ = true; }

 protected:
  bool PlanWithGPIR(const common::State& ego_state,
                    const ReferenceLine& reference_line,
                    const std::vector<Obstacle>& dynamic_agents,
                    const std::vector<Eigen::Vector2d>& virtual_obstacles,
                    const common::Trajectory& last_trajectory,
                    common::Trajectory* trajectory);

 private:
  bool ProcessObstacles(const std::vector<Obstacle>& raw_obstacles,
                        const ReferenceLine& reference_line,
                        std::vector<Obstacle>* cirtical_obstacles);

  void VisualizeTrajectory(
      const std::vector<std::pair<hdmap::LaneSegmentBehavior,
                                  common::Trajectory>>& trajectory_candidates);

  void VisualizeCriticalObstacle(
      const std::vector<Obstacle>& critical_obstacles);

  void VisualizeTargetLane(const ReferenceLine& reference_line);

 private:
  ros::Publisher trajectory_pub_;
  ros::Publisher critical_obstacle_pub_;
  ros::Publisher target_lane_pub_;

  common::State state_;
  common::Box2D ego_box_;
  VehicleParam vehicle_param_;

  int max_iter = 5;
  double reference_speed_ = 0.0;
  const double lateral_critical_thereshold_ = 6;

  std::vector<Obstacle> static_obstacles_;
  std::vector<Obstacle> dynamic_obstacles_;

  bool save_snapshot_ = true;

  hdmap::LaneSegmentBehavior last_behavior_ = hdmap::LaneSegmentBehavior::kKeep;
};

}  // namespace planning
