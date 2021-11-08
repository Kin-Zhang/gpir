
#pragma once

#include <string>

class BenchmarkMehtodConfig {
 public:
  static const std::string tdr_obca_config_path;
  static const bool use_dual_variable_warm_start;
};

class BenchmarkVehicleConfig {
 public:
  static const double max_steer_angle;
  static const double steer_ratio;
  static const double max_steer_angle_rate;
  static const double wheelbase;

  static const double length;
  static const double width;
  static const double height;
  static const double front_to_front_axle;
  static const double back_to_rear_axle;
  static const double rear_axle_to_center;

  static const double front_to_center;
  static const double back_to_center;
  static const double left_to_center;
  static const double right_to_center;
};
