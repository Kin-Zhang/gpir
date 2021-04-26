/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#pragma once

#include <utility>
#include <vector>

#include "gp_planner/initializer/piecewise_jerk_problem.h"

namespace udrive {
namespace planning {

/**
 * @brief: borrow from apollo open source code
 *
 * This class solve an optimization problem:
 * x
 * |
 * |                       P(s1, x1)  P(s2, x2)
 * |            P(s0, x0)                       ... P(s(k-1), x(k-1))
 * |P(start)
 * |
 * |________________________________________________________ s
 *
 * we suppose s(k+1) - s(k) == s(k) - s(k-1)
 *
 * Given the x, x', x'' at P(start),  The goal is to find x0, x1, ... x(k-1)
 * which makes the line P(start), P0, P(1) ... P(k-1) "smooth".
 */

class PiecewiseJerkPathProblem : public PiecewiseJerkProblem {
 public:
  PiecewiseJerkPathProblem(const size_t num_of_knots, const double delta_s,
                           const std::array<double, 3>& x_init);

  virtual ~PiecewiseJerkPathProblem() = default;

 protected:
  void CalculateKernel(std::vector<c_float>* P_data,
                       std::vector<c_int>* P_indices,
                       std::vector<c_int>* P_indptr) override;

  void CalculateOffset(std::vector<c_float>* q) override;
};

}  // namespace planning
}  // namespace udrive
