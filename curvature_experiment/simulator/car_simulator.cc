#include "curvature_experiment/simulator/car_simulator.h"

#include "odeint-v2/boost/numeric/odeint.hpp"

namespace odeint = boost::numeric::odeint;

void VehicleModel::Step(double dt) {
  odeint::integrate(boost::ref(*this), internal_state_, 0.0, dt, dt);
  state_.x = internal_state_[0];
  state_.y = internal_state_[1];
  state_.heading = internal_state_[2];
  state_.steer = control_.steer;
  state_.velocity = control_.vel;
  // state_.acc = control_.acc;
  UpdateInternalState();
}

void VehicleModel::operator()(const InternalState &x, InternalState &dxdt,
                              const double /* t */) {
  State cur_state;
  cur_state.x = x[0];
  cur_state.y = x[1];
  cur_state.heading = x[2];

  dxdt[0] = std::cos(cur_state.heading) * control_.vel;
  dxdt[1] = std::sin(cur_state.heading) * control_.vel;
  dxdt[2] = std::tan(control_.steer) * control_.vel / wheel_base_;
}

void VehicleModel::UpdateInternalState() {
  internal_state_[0] = state_.x;
  internal_state_[1] = state_.y;
  internal_state_[2] = state_.heading;
}