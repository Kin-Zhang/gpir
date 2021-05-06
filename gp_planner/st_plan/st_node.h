/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#pragma once

#include <memory>

namespace planning {

struct StNodeWeights {
  double ref_v = 0.0;
  double obstacle = 0.0;
  double control = 0.0;
};

class StNode {
 public:
  StNode() = default;
  StNode(const double s, const double v) : s(s), v(v) {}

  static void SetReferenceSpeed(const double ref_v) { ref_v_ = ref_v; }
  static void SetWeights(const StNodeWeights& weights) { weights_ = weights; }

  std::unique_ptr<StNode> Forward(const double delta_t, const double a) const;
  void CalObstacleCost(const double d);
  inline double GetDistance(const double delta_t, const double a) const {
    return s + v * delta_t + 0.5 * a * delta_t * delta_t;
  }

  double t = 0.0;
  double s = 0.0;
  double v = 0.0;
  double a = 0.0;
  double cost = 0.0;
  const StNode* parent = nullptr;

 private:
  static double ref_v_;
  static StNodeWeights weights_;
};

}  // namespace planning
