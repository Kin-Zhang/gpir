find_package(osqp REQUIRED)

add_library(gp_planner_initializer)
add_library(gp_planner::initializer ALIAS gp_planner_initializer)

target_sources(gp_planner_initializer PRIVATE
  gp_initializer.cc
  piecewise_jerk_path_problem.cc
  piecewise_jerk_problem.cc
)
target_link_libraries(gp_planner_initializer
  osqp::osqp
  common::smoothing
)