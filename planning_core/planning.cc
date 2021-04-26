/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include "planning_core/planning_core.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "planning");
  ros::NodeHandle node;

  google::InstallFailureSignalHandler();
  google::InitGoogleLogging(argv[0]);
  google::SetStderrLogging(google::INFO);
  // FLAGS_log_prefix = false
  FLAGS_colorlogtostderr = true;

  planning::PlanningCore planning_core;
  planning_core.Init();

  ros::Timer main_loop = node.createTimer(
      ros::Duration(0.05), &planning::PlanningCore::Run, &planning_core);

  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();

  return 0;
}
