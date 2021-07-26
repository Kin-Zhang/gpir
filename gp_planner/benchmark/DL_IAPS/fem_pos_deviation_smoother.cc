/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#include "gp_planner/benchmark/DL_IAPS/fem_pos_deviation_smoother.h"

#include <glog/logging.h>

#include "gp_planner/benchmark/DL_IAPS/fem_pos_deviation_ipopt_interface.h"
#include "gp_planner/benchmark/DL_IAPS/fem_pos_deviation_sqp_osqp_interface.h"

namespace apollo {
namespace planning {
FemPosDeviationSmoother::FemPosDeviationSmoother() {}

bool FemPosDeviationSmoother::Solve(
    const std::vector<std::pair<double, double>>& raw_point2d,
    const std::vector<double>& bounds, std::vector<double>* opt_x,
    std::vector<double>* opt_y) {
  // return SqpWithOsqp(raw_point2d, bounds, opt_x, opt_y);
  return NlpWithIpopt(raw_point2d, bounds, opt_x, opt_y);
}

bool FemPosDeviationSmoother::SqpWithOsqp(
    const std::vector<std::pair<double, double>>& raw_point2d,
    const std::vector<double>& bounds, std::vector<double>* opt_x,
    std::vector<double>* opt_y) {
  if (opt_x == nullptr || opt_y == nullptr) {
    LOG(ERROR) << "opt_x or opt_y is nullptr";
    return false;
  }

  FemPosDeviationSqpOsqpInterface solver;

  solver.set_weight_fem_pos_deviation(1e8);
  solver.set_weight_path_length(0.0);
  solver.set_weight_ref_deviation(1e3);
  solver.set_weight_curvature_constraint_slack_var(1e8);

  solver.set_curvature_constraint(0.2);

  solver.set_sqp_sub_max_iter(100);
  solver.set_sqp_ftol(1e-4);
  solver.set_sqp_pen_max_iter(10);
  solver.set_sqp_ctol(1e-3);

  solver.set_max_iter(500);
  solver.set_time_limit(0.0);
  solver.set_verbose(false);
  solver.set_scaled_termination(true);
  solver.set_warm_start(true);

  solver.set_ref_points(raw_point2d);
  solver.set_bounds_around_refs(bounds);

  if (!solver.Solve()) {
    return false;
  }

  std::vector<std::pair<double, double>> opt_xy = solver.opt_xy();

  // TODO(Jinyun): unify output data container
  opt_x->resize(opt_xy.size());
  opt_y->resize(opt_xy.size());
  for (size_t i = 0; i < opt_xy.size(); ++i) {
    (*opt_x)[i] = opt_xy[i].first;
    (*opt_y)[i] = opt_xy[i].second;
  }
  return true;
}

bool FemPosDeviationSmoother::NlpWithIpopt(
    const std::vector<std::pair<double, double>>& raw_point2d,
    const std::vector<double>& bounds, std::vector<double>* opt_x,
    std::vector<double>* opt_y) {
  if (opt_x == nullptr || opt_y == nullptr) {
    LOG(ERROR) << "opt_x or opt_y is nullptr";
    return false;
  }

  FemPosDeviationIpoptInterface* smoother =
      new FemPosDeviationIpoptInterface(raw_point2d, bounds);

  smoother->set_weight_fem_pos_deviation(1e8);
  smoother->set_weight_path_length(0.0);
  smoother->set_weight_ref_deviation(1e3);
  smoother->set_weight_curvature_constraint_slack_var(1e8);
  smoother->set_curvature_constraint(0.2);

  Ipopt::SmartPtr<Ipopt::TNLP> problem = smoother;

  // Create an instance of the IpoptApplication
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

  app->Options()->SetIntegerValue("print_level", 1);
  app->Options()->SetIntegerValue("max_iter", 500);
  app->Options()->SetIntegerValue("acceptable_iter", 15);
  app->Options()->SetNumericValue("tol", 1e-3);
  app->Options()->SetNumericValue("acceptable_tol", 1e-2);

  Ipopt::ApplicationReturnStatus status = app->Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    LOG(ERROR) << "*** Error during initialization!";
    return false;
  }

  status = app->OptimizeTNLP(problem);

  if (status == Ipopt::Solve_Succeeded ||
      status == Ipopt::Solved_To_Acceptable_Level) {
    // Retrieve some statistics about the solve
    Ipopt::Index iter_count = app->Statistics()->IterationCount();
    LOG(INFO) << "*** The problem solved in " << iter_count << " iterations!";
  } else {
    LOG(ERROR) << "Solver fails with return code: " << static_cast<int>(status);
    return false;
  }
  smoother->get_optimization_results(opt_x, opt_y);
  return true;
}

}  // namespace planning
}  // namespace apollo
