find_package(osqp REQUIRED)

add_library(common_solver)
add_library(common::solver ALIAS common_solver)

target_sources(common_solver PRIVATE
  osqp/osqp_interface.cc
  qp_problem.cc
)

target_link_libraries(common_solver 
  glog
  osqp::osqp
)
