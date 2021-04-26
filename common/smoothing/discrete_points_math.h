/* Copyright 2019 Unity-Drive Inc. All rights reserved */
/*     Author: ChengJie (chengjie@unity-drive.com)     */

#pragma once

#include <utility>
#include <vector>

namespace common {

class DiscretePointsMath {
 public:
  DiscretePointsMath() = delete;

  static bool ComputePathProfile(
      const std::vector<std::pair<double, double>>& xy_points,
      std::vector<double>* headings, std::vector<double>* accumulated_s,
      std::vector<double>* kappas, std::vector<double>* dkappas);
};

}  // namespace common
