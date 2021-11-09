#pragma once

#include <boost/array.hpp>
#include <boost/bind.hpp>

struct State {
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  double velocity = 0.0;
  // double acc = 0.0;
  double steer = 0.0;

  State() = default;
  State(const double x, const double y, const double heading)
      : x(x), y(y), heading(heading) {}
};

struct Control {
  double vel = 0.0;
  double steer = 0.0;

  Control() = default;
  Control(const double vel, const double steer) : vel(vel), steer(steer) {}
};

class VehicleModel {
 public:
  VehicleModel() = default;
  VehicleModel(const double wheel_base) : wheel_base_(wheel_base) {}

  const State &state() { return state_; }

  void SetState(const State &state) {
    state_ = state;
    UpdateInternalState();
  }
  void SetControl(const Control &control) { control_ = control; }

  void Step(const double dt);

  typedef boost::array<double, 3> InternalState;
  void operator()(const InternalState &x, InternalState &dxdt,
                  const double /* t */);

 protected:
  void UpdateInternalState();

 private:
  State state_;
  Control control_;
  InternalState internal_state_;

  double wheel_base_ = 0.0;
};