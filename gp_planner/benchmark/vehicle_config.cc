

#include "gp_planner/benchmark/vehicle_config.h"

const bool BenchmarkMehtodConfig::use_dual_variable_warm_start = true;
const std::string BenchmarkMehtodConfig::tdr_obca_config_path =
    "/home/cj/research/gpir_ws/src/gpir/gp_planner/benchmark/TDR_OBCA/conf/"
    "planner_open_space_config.pb.txt";

const double BenchmarkVehicleConfig::max_steer_angle = 31.;
const double BenchmarkVehicleConfig::steer_ratio = 57.3;
const double BenchmarkVehicleConfig::max_steer_angle_rate = 10.;
const double BenchmarkVehicleConfig::wheelbase = 3.00464;

// back_to_rear_axle []  wheelbase  [] front_to_front_axle
const double BenchmarkVehicleConfig::length = 4.79178;
const double BenchmarkVehicleConfig::width = 2.16345;
const double BenchmarkVehicleConfig::height = 1.48832;
const double BenchmarkVehicleConfig::front_to_front_axle = 0.777665;
const double BenchmarkVehicleConfig::back_to_rear_axle = 1.00947;
const double BenchmarkVehicleConfig::rear_axle_to_center = 1.38642;

const double BenchmarkVehicleConfig::front_to_center = 3.78;
const double BenchmarkVehicleConfig::back_to_center = 1.00947;
const double BenchmarkVehicleConfig::left_to_center = 1.081725;
const double BenchmarkVehicleConfig::right_to_center = 1.081725;
