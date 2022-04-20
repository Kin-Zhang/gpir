#include <glog/logging.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>

#include <chrono>
#include <thread>

#include "csv/csv.h"
#include "curvature_experiment/controller/mpc_controller.h"
#include "curvature_experiment/simulator/car_simulator.h"
#include "curvature_experiment/timer.h"

using std::chrono::high_resolution_clock;
using std::chrono::microseconds;
// using common::Timer;

int main(int argc, char* argv[]) {
  google::InstallFailureSignalHandler();
  google::InitGoogleLogging(argv[0]);
  google::SetStderrLogging(google::INFO);
  // FLAGS_log_prefix = false
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "curvature_experiment");
  ros::NodeHandle n("~");

  // visualization
  ros::Publisher traj_pub = n.advertise<nav_msgs::Path>("path", 100, true);
  ros::Publisher car_pub =
      n.advertise<jsk_recognition_msgs::BoundingBox>("car", 100);
  ros::Publisher ref_pub =
      n.advertise<jsk_recognition_msgs::BoundingBox>("ref", 100);
  ros::Publisher obs_pub =
      n.advertise<jsk_recognition_msgs::BoundingBoxArray>("obs", 100, true);

  jsk_recognition_msgs::BoundingBoxArray markers;
  markers.header.frame_id = "world";

  jsk_recognition_msgs::BoundingBox marker;
  marker.header.frame_id = "world";
  marker.dimensions.x = 5;
  marker.dimensions.y = 11.5;
  marker.dimensions.z = 1.5;
  marker.pose.position.x = 32.5;
  marker.pose.position.y = (7.5 - 4) / 2.0 + 0.5;
  marker.pose.position.z = marker.dimensions.z / 2.0;
  marker.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI);
  markers.boxes.emplace_back(marker);

  marker.dimensions.x = 5.5;
  marker.dimensions.y = 10.5;
  marker.dimensions.z = 1.5;
  marker.pose.position.x = (44.5 + 50) / 2.0;
  marker.pose.position.y = (3 - 7.5) / 2.0 - 0.5;
  marker.pose.position.z = marker.dimensions.z / 2.0;
  marker.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI);
  markers.boxes.emplace_back(marker);

  marker.dimensions.x = 5;
  marker.dimensions.y = 9.5;
  marker.dimensions.z = 1.5;
  marker.pose.position.x = (60 + 65) / 2.0;
  marker.pose.position.y = (7.5 - 2) / 2.0 + 0.5;
  marker.pose.position.z = marker.dimensions.z / 2.0;
  marker.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI);
  markers.boxes.emplace_back(marker);

  obs_pub.publish(markers);

  nav_msgs::Path visual_path;
  geometry_msgs::PoseStamped path_point;
  path_point.pose.orientation.w = 1.0;
  visual_path.header.frame_id = "world";

  jsk_recognition_msgs::BoundingBox bbox;
  bbox.dimensions.x = 4.79;
  bbox.dimensions.y = 2.16;
  bbox.dimensions.z = 1.488;
  bbox.pose.position.z = bbox.dimensions.z / 2.0;
  bbox.header.frame_id = "world";

  jsk_recognition_msgs::BoundingBox ref_box = bbox;

  // load trajectory
  std::string file(
      "/home/kin/gpir_ws/src/gpir/curvature_experiment/"
      "curvature_benchmark.csv");
  std::vector<double> x, y, s, kappa;
  std::vector<std::string> planner;
  io::CSVReader<5> in(file);
  in.read_header(io::ignore_extra_column, "planner", "x", "y", "s", "k");
  std::string in_planner;
  double in_x, in_y, in_s, in_k;
  while (in.read_row(in_planner, in_x, in_y, in_s, in_k)) {
    planner.emplace_back(in_planner);
    x.emplace_back(in_x);
    y.emplace_back(in_y);
    kappa.emplace_back(in_k);
    // printf("%s,%f,%f\n", in_planner.c_str(), in_x, in_y);
  }

  // std::string target_planner = "GPCur";
  std::string target_planner;
  n.getParam("target_planner", target_planner);
  std::cout << target_planner << std::endl;

  common::Trajectory traj;
  common::State state;
  Eigen::Vector2d start_pos;
  double theta = 0;
  bool new_traj = true;

  std::cout << "??1" << std::endl;
  for (size_t i = 0; i < planner.size(); ++i) {
    if (planner[i] == target_planner) {
      if (new_traj) {
        start_pos = Eigen::Vector2d(x[i], y[i]);
      } else {
        auto cur_pos = Eigen::Vector2d(x[i], y[i]);
        auto diff = cur_pos - start_pos;
        theta = std::atan2(diff.y(), diff.x());
        start_pos = cur_pos;
      }
      state.position.x() = x[i];
      state.position.y() = y[i];
      state.velocity = 3.5;
      state.heading = theta;
      state.kappa = kappa[i];
      traj.emplace_back(state);

      path_point.pose.position.x = x[i];
      path_point.pose.position.y = y[i];
      visual_path.poses.emplace_back(path_point);

      if (new_traj) new_traj = false;
    }
  }
  traj.front().heading = traj[1].heading;

  traj_pub.publish(visual_path);

  std::cout << "??" << std::endl;
  const double wheel_base = 3.0;

  VehicleModel car(wheel_base);
  MpcController controller;
  controller.set_wheel_base(wheel_base);
  controller.Init();

  auto init_pos = traj.front();
  State car_state(init_pos.position.x(), init_pos.position.y(), 0.0);
  car.SetState(car_state);

  auto start = high_resolution_clock::now();
  auto now = start;
  int count = 0;

  Control cmd;
  while (n.ok()) {
    TIC;
    start = high_resolution_clock::now();

    car_state = car.state();
    state.position.x() = car_state.x;
    state.position.y() = car_state.y;
    state.heading = car_state.heading;

    if (count == 4) {
      controller.CalculateAckermannDrive(state, traj, &cmd);
      // traj_pub.publish(visual_path);

      count = 0;
    } else {
      count++;
    }

    car.SetControl(cmd);
    car.Step(0.01);
    // TOC("wtf");

    bbox.pose.position.x = car_state.x;
    bbox.pose.position.y = car_state.y;
    bbox.pose.orientation = tf::createQuaternionMsgFromYaw(car_state.heading);
    bbox.header.stamp = ros::Time::now();
    car_pub.publish(bbox);

    auto ref_state = traj.GetInterpolatedNearestState(state.position);
    // std::cout << "111" << std::endl;
    ref_box.pose.position.x = ref_state.position.x();
    ref_box.pose.position.y = ref_state.position.y();
    ref_box.pose.orientation =
        tf::createQuaternionMsgFromYaw(ref_state.heading);
    ref_box.header.stamp = ros::Time::now();
    ref_pub.publish(ref_box);
    // std::cout << "111" << std::endl;

    auto elapsed = std::chrono::duration_cast<microseconds>(
                       high_resolution_clock::now() - start)
                       .count();
    // std::cout << elapsed << std::endl;
    std::this_thread::sleep_for(std::chrono::microseconds(10000 - elapsed));
  }

  return 0;
}
