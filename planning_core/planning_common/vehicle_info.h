/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#pragma once

namespace planning {

struct VehicleParam {
  double length = 0.0;
  double width = 0.0;
  double height = 0.0;
  double wheel_base = 0.0;
  double front_to_front_axle = 0.0;
  double back_to_rear_axle = 0.0;
  double rear_axle_to_center = 0.0;

  double max_acc = 0.0;          // m^2/s
  double max_speed = 0.0;        // m/s
  double max_steer_angle = 0.0;  // rad
  double max_kappa = 0.0;        // rad
};

class VehicleInfo {
 public:
  static VehicleInfo& Instance() {
    static VehicleInfo vehicle_info;
    return vehicle_info;
  }

  void set_id(const int ego_id) { id_ = ego_id; }
  int id() const { return id_; }

  inline const VehicleParam& vehicle_param() const { return vehicle_param_; }
  inline VehicleParam* mutable_vehicle_param() { return &vehicle_param_; }

 private:
  int id_;
  VehicleParam vehicle_param_;

 private:
  VehicleInfo() = default;
  VehicleInfo(const VehicleInfo&) = delete;
  VehicleInfo& operator=(const VehicleInfo&) = delete;
};

}  // namespace planning
