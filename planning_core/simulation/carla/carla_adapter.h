/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#pragma once

#include <mutex>
#include <thread>

#include "planning_core/simulation/carla/ego_info/carla_ego_info.h"
#include "planning_core/simulation/carla/mock_perception/carla_mock_perception.h"
#include "planning_core/simulation/controller/mpc_controller.h"
#include "planning_core/simulation/simulator_adapter.h"

namespace planning {
namespace simulation {

class CarlaAdapter final : public SimulatorAdapter {
 public:
  CarlaAdapter();
  // virtual ~CarlaAdapter() { control_thread_.join(); }

  void Init() override;
  std::string Name() const { return "Carla"; }
  bool InitVehicleParam(VehicleParam* vehicle_param) override;
  bool UpdateEgoState(common::State* state) override;
  bool UpdatePerceptionResults(std::vector<Obstacle>* obstacles) override;
  void SetTrajectory(const common::Trajectory& trajectory) override;

 protected:
  void ControlLoop();

 private:
  CarlaEgoInfo carla_ego_info_;
  CarlaMockPerception carla_mock_perception_;

  // control
  ros::Publisher control_cmd_pub_;
  std::mutex control_mutex_;
  // std::thread control_thread_;

  double wheel_base_ = 0.0;

  common::Trajectory trajectory_;
  MpcController mpc_controller_;
};
}  // namespace simulation
}  // namespace planning
