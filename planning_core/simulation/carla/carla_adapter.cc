/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include "planning_core/simulation/carla/carla_adapter.h"

#include <ackermann_msgs/AckermannDrive.h>
#include <glog/logging.h>

#include <thread>

#include "common/utils/math.h"

namespace planning {
namespace simulation {

CarlaAdapter::CarlaAdapter() {}

void CarlaAdapter::Init() {
  carla_ego_info_.Init();
  carla_mock_perception_.Init();

  ros::NodeHandle node;
  control_cmd_pub_ = node.advertise<ackermann_msgs::AckermannDrive>(
      "carla/ego_vehicle/ackermann_cmd", 1);
  mpc_controller_.Init();
  // control_thread_ = std::thread(&CarlaAdapter::ControlLoop, this);
};

bool CarlaAdapter::InitVehicleParam(VehicleParam* vehicle_param) {
  for (int i = 0; i < 10 && ros::ok(); ++i) {
    ros::spinOnce();
    if (carla_ego_info_.GetVehicleParam(vehicle_param)) {
      wheel_base_ = vehicle_param->wheel_base;
      return true;
    }
    LOG(WARNING) << "fail to get vehicle param from Carla, Retry: " << i;
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }
  return false;
}

bool CarlaAdapter::UpdateEgoState(common::State* state) {
  return carla_ego_info_.UpdateEgoState(state);
}

bool CarlaAdapter::UpdatePerceptionResults(std::vector<Obstacle>* obstacles) {
  return carla_mock_perception_.UpdateMockPerceptionResult(obstacles);
}

void CarlaAdapter::SetTrajectory(const common::Trajectory& trajectory) {
  // std::lock_guard<std::mutex> lock(control_mutex_);
  // trajectory_ = trajectory;
  // if (trajectory.empty()) return;

  common::State ego_state;
  ackermann_msgs::AckermannDrive ackermann_drive;
  CHECK_GT(wheel_base_, 2);
  mpc_controller_.set_wheel_base(wheel_base_);
  carla_ego_info_.UpdateEgoState(&ego_state);
  mpc_controller_.CalculateAckermannDrive(ego_state, trajectory,
                                          &ackermann_drive);
  // int min_index = trajectory.GetNearsetIndex(ego_state.position);
  // int ref_index = std::min<int>(min_index + 50, trajectory.size() - 1);
  // auto ref_state = trajectory[ref_index];
  // auto vec = ref_state.position - ego_state.position;
  // double angle = std::atan2(vec.y(), vec.x());
  // double angle_diff = common::NormalizeAngle(angle - ego_state.heading);
  // double dis = vec.norm();
  // ackermann_drive.speed = 5.0;
  // ackermann_drive.steering_angle =
  //     std::atan2(2.0 * wheel_base_ * std::sin(angle_diff), dis);
  // LOG(INFO) << "angle: " << ackermann_drive.steering_angle;
  control_cmd_pub_.publish(ackermann_drive);
}

void CarlaAdapter::ControlLoop() {
  ros::Rate rate(20);
  common::State ego_state;
  ackermann_msgs::AckermannDrive ackermann_drive;
  mpc_controller_.set_wheel_base(wheel_base_);

  while (ros::ok()) {
    carla_ego_info_.UpdateEgoState(&ego_state);
    mpc_controller_.CalculateAckermannDrive(ego_state, trajectory_,
                                            &ackermann_drive);
    control_cmd_pub_.publish(ackermann_drive);
    rate.sleep();
  }
}
}  // namespace simulation
}  // namespace planning
