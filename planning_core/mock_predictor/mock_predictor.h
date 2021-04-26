/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#pragma once

#include <ros/ros.h>

#include <unordered_map>

#include "planning_core/planning_common/obstacle.h"

namespace planning {

class MockPredictor {
 public:
  MockPredictor(const double prediction_horizon, const double dt)
      : prediction_horizon_(prediction_horizon), dt_(dt) {}
  virtual ~MockPredictor() = default;

  virtual void Init();

  virtual void UpdateVehicleLaneMap(const std::vector<Obstacle>& obstacles);

  virtual bool GeneratePrediction(std::vector<Obstacle>* obstacles) = 0;

 protected:
  void VisualizePrediction(const std::vector<Obstacle>& obstacles);

 protected:
  ros::Publisher predict_pub_;

  double dt_ = 0.0;
  double prediction_horizon_ = 0.0;

  // (VehicleId, LandId)
  std::unordered_map<int, int> vehicle_lane_map_;
};

}  // namespace planning
