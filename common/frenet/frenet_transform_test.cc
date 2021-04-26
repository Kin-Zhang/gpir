/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include "common/frenet/frenet_transform.h"

#include <gtest/gtest.h>

namespace common {

TEST(FrenetTransfromTest, basic) {
  FrenetReferencePoint ref_point;
  ref_point.point = Eigen::Vector2d(10.1382, 146.583);
  ref_point.s = 19.5102;
  ref_point.theta = 1.5462;
  ref_point.kappa = -3.54267e-05;
  ref_point.dkappa = -1.82075e-05;

  State state;
  state.position = Eigen::Vector2d(6.64011, 146.669);
  state.heading = 1.54612;
  state.velocity = 0;
  state.acceleration = 0;

  FrenetState frenet_state;
  FrenetTransfrom::StateToFrenetState(state, ref_point, &frenet_state);

  std::cout << frenet_state.DebugString() << std::endl;
}

}  // namespace common
