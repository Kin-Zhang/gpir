/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include "gp_planner/initializer/gp_initializer.h"

#include "common/smoothing/osqp_spline1d_solver.h"
#include "gp_planner/initializer/piecewise_jerk_path_problem.h"

namespace planning {

void GPInitializer::SetBoundary(
    std::vector<std::vector<std::pair<double, double>>> boundary) {
  // tmp test
  boundary_.clear();
  for (const auto b : boundary) {
    auto a = b.front();
    a.first += 2.5;
    a.second -= 2.5;
    // std::cout << a.first << ", " << a.second << std::endl;
    boundary_.emplace_back(a);
  }
}

bool GPInitializer::Solve(std::array<double, 3> x0, std::array<double, 3> xn,
                          const double interval,
                          std::vector<Eigen::Vector3d>* init_value,
                          std::vector<double>* l) {
  const size_t num_of_knots = boundary_.size();
  udrive::planning::PiecewiseJerkPathProblem prob(num_of_knots, interval, x0);

  prob.set_weight_x(1.0);
  prob.set_weight_dx(20.0);
  prob.set_weight_ddx(1000.0);
  prob.set_weight_dddx(50000.0);

  // prob.set_x_bounds(-100, 100.0);
  prob.set_dx_bounds(-3.0, 3.0);
  prob.set_ddx_bounds(-2.0, 2.0);
  prob.set_dddx_bound(-0.5, 0.5);

  // prob.set_weight_x(1.0);
  // prob.set_weight_dx(20.0);
  // prob.set_weight_ddx(1000.0);
  // prob.set_weight_dddx(50000.0);

  // prob.set_scale_factor({1.0, 10.0, 100.0});
  prob.set_x_bounds(boundary_);
  // prob.set_dx_bounds(-10, 10);
  prob.set_end_state_ref(std::array<double, 3>{1000.0, 100000.0, 1000000.0},
                         std::array<double, 3>{xn[0], 0, 0});
  // prob.set_x_ref(100, std::vector<double>(num_of_knots, 5));
  // prob.set_dx_bounds(-2, 2);

  if (!prob.Optimize()) {
    return false;
  }

  auto d = prob.opt_x();
  *l = prob.opt_x();
  auto d_prime = prob.opt_dx();
  auto d_prime_prime = prob.opt_ddx();

  for (int i = 0; i < num_of_knots; ++i) {
    init_value->emplace_back(
        Eigen::Vector3d(d[i], d_prime[i], d_prime_prime[i]));
  }

  return true;
}

bool GPInitializer::Solve2(Eigen::Vector3d x0, Eigen::Vector3d xn,
                           std::vector<double> s_refs, const double s,
                           const double lb, const double ub,
                           std::vector<Eigen::Vector3d>* res,
                           std::vector<double>* l) {
  double start = s_refs.front();
  std::vector<double> knots{start, start + 20, start + 40, start + 60,
                            start + 90};

  common::OsqpSpline1dSolver spline2d_solver(knots, 5);

  std::vector<double> ref(s_refs.size(), 0);

  auto mutable_kernel = spline2d_solver.mutable_kernel();
  auto mutable_constraint = spline2d_solver.mutable_constraint();
  mutable_kernel->AddReferenceLineKernelMatrix(s_refs, ref, 1);
  mutable_kernel->AddSecondOrderDerivativeMatrix(200);
  mutable_kernel->AddThirdOrderDerivativeMatrix(1000);
  mutable_constraint->AddPointConstraint(start, x0(0));
  mutable_constraint->AddPointDerivativeConstraint(start, x0(1));
  mutable_constraint->AddPointSecondDerivativeConstraint(start, x0(2));
  mutable_constraint->AddPointConstraint(start + 90, xn(0));
  mutable_constraint->AddPointDerivativeConstraint(start + 90, xn(1));
  mutable_constraint->AddPointSecondDerivativeConstraint(start + 90, xn(2));
  mutable_constraint->AddSecondDerivativeSmoothConstraint();

  if (s > 0) {
    double true_ub = std::max(ub - 1.5, lb + 0.1);
    LOG(WARNING) << "lb: " << lb << ", ub: " << ub;
    mutable_constraint->AddBoundary(std::vector<double>{s},
                                    std::vector<double>{lb},
                                    std::vector<double>{true_ub});
  }

  if (!spline2d_solver.Solve()) {
    LOG(ERROR) << "solve failed";
    return false;
  }

  auto spline = spline2d_solver.spline();

  for (int i = 0; i < s_refs.size(); ++i) {
    Eigen::Vector3d val(spline(s_refs[i]), spline.Derivative(s_refs[i]),
                        spline.SecondOrderDerivative(s_refs[i]));

    res->emplace_back(val);
    l->emplace_back(val.x());
  }

  return true;
}

bool GPInitializer::GenerateInitialPath(
    const Eigen::Vector3d& x0, const Eigen::Vector3d& xn,
    const std::vector<double> s_refs,
    const std::vector<double>& obstacle_location_hint,
    const std::vector<double>& lb, std::vector<double>& ub,
    vector_Eigen3d* result) {
  double start = s_refs.front();
  double length = s_refs.back() - start;
  std::vector<double> knots{start, start + length / 4.0, start + length / 2.0,
                            start + length * 3.0 / 4.0, start + length};

  common::OsqpSpline1dSolver spline1d_solver(knots, 5);

  std::vector<double> ref(s_refs.size(), 0);

  auto mutable_kernel = spline1d_solver.mutable_kernel();
  auto mutable_constraint = spline1d_solver.mutable_constraint();
  mutable_kernel->AddReferenceLineKernelMatrix(s_refs, ref, 1);
  mutable_kernel->AddSecondOrderDerivativeMatrix(200);
  mutable_kernel->AddThirdOrderDerivativeMatrix(1000);
  mutable_constraint->AddPointConstraint(start, x0(0));
  mutable_constraint->AddPointDerivativeConstraint(start, x0(1));
  mutable_constraint->AddPointSecondDerivativeConstraint(start, x0(2));
  mutable_constraint->AddPointConstraint(start + length, xn(0));
  mutable_constraint->AddPointDerivativeConstraint(start + length, xn(1));
  mutable_constraint->AddPointSecondDerivativeConstraint(start + length, xn(2));
  mutable_constraint->AddSecondDerivativeSmoothConstraint();

  if (!obstacle_location_hint.empty()) {
    mutable_constraint->AddBoundary(obstacle_location_hint, lb, ub);
  }

  if (!spline1d_solver.Solve()) {
    LOG(ERROR) << "solve failed";
    return false;
  }

  auto spline = spline1d_solver.spline();
  for (int i = 0; i < s_refs.size(); ++i) {
    result->emplace_back(
        Eigen::Vector3d(spline(s_refs[i]), spline.Derivative(s_refs[i]),
                        spline.SecondOrderDerivative(s_refs[i])));
  }
  return true;
}
}  // namespace planning
