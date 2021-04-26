/* Copyright 2021 Unity-Drive Inc. A  ll rights reserved */

#pragma once

#include <Eigen/Dense>
#include <array>
#include <string>

namespace common {

struct FrenetPoint {
  double s = 0.0;
  double d = 0.0;

  FrenetPoint() = default;
  FrenetPoint(const double s, const double d) : s(s), d(d) {}

  std::string DebugString() const {
    return "[FrenetPoint] s: " + std::to_string(s) +
           ", d: " + std::to_string(d);
  }
};

struct FrenetReferencePoint {
  Eigen::Vector2d point;
  double s = 0.0;
  double theta = 0.0;
  double kappa = 0.0;
  double dkappa = 0.0;
  ;
  std::string DebugString() const {
    std::ostringstream os;
    os << "[FrenetReferencePoint] "
       << "point: (" << point.x() << ", " << point.y() << "), "
       << "s: " << s << ", "
       << "theta: " << theta << ", "
       << "kappa: " << kappa << ", "
       << "dkappa: " << dkappa;
    return os.str();
  }
};

struct FrenetState {
  enum class DerivativeType {
    kDs = 0,
    kDt = 1,
  };

  std::array<double, 3> s;
  std::array<double, 3> d;
  DerivativeType type = DerivativeType::kDs;

  FrenetPoint frenet_point() { return FrenetPoint(s[0], d[0]); }

  std::string DebugString() const {
    std::ostringstream os;
    os << "[FrenetState] "
       << "s: (" << s[0] << ", " << s[1] << ", " << s[2] << ") "
       << "d: (" << d[0] << ", " << d[1] << ", " << d[2] << ")";
    return os.str();
  }
};

}  // namespace common