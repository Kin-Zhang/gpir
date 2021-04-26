#include <glog/logging.h>

#include <Eigen/Dense>
#include <iostream>

#include "common/base/state.h"
#include "common/base/trajectory.h"
#include "planning_core/simulation/controller/mmpc_controller.h"

int main(int argc, char const *argv[]) {
  planning::simulation::MpcController control;
  control.Init();
  control.set_wheel_base(2.0);
  int a = 0;

  { Eigen::MatrixXd aa = Eigen::MatrixXd::Zero(a, a); }

  common::State state;
  common::Trajectory traj;
  for (int i = 0; i < 100; ++i) {
    state.position.x() = 0.1 * i;
    traj.emplace_back(state);
  }

  ackermann_msgs::AckermannDrive dr;
  int count = 0;
  while (count < 10) {
    bool res = control.CalculateAckermannDrive(traj[0], traj, &dr);
    count++;
  }

  return 0;
}
