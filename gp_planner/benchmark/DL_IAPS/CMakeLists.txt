add_subdirectory(proto)

find_package(PkgConfig)
pkg_search_module(IPOPT ipopt)
pkg_search_module(ADOLC adolc)

add_library(benchmark_dl_iaps)
add_library(benchmark::dl_iaps ALIAS benchmark_dl_iaps)
target_sources(benchmark_dl_iaps PRIVATE
	aabox2d.cc
	box2d.cc
	discrete_points_math.cc
	discretized_path.cc
	fem_pos_deviation_smoother.cc
	fem_pos_deviation_sqp_osqp_interface.cc
	fem_pos_deviation_ipopt_interface.cc
	iterative_anchoring_smoother.cc
	line_segment2d.cc
	linear_interpolation.cc
	math_utils.cc
	polygon2d.cc
	vec2d.cc

	coarse_trajectory_generator/grid_search.cc
	coarse_trajectory_generator/hybrid_a_star.cc
	coarse_trajectory_generator/node3d.cc
	coarse_trajectory_generator/piecewise_jerk_problem.cc
	coarse_trajectory_generator/piecewise_jerk_speed_problem.cc
	coarse_trajectory_generator/reeds_shepp_path.cc
	coarse_trajectory_generator/speed_data.cc
)

target_link_libraries(benchmark_dl_iaps 
	osqp
	glog
	ipopt
	adolc
	benchmark::proto
)

add_executable(dl_iaps_test dl_iaps_test.cc)
target_link_libraries(dl_iaps_test benchmark_dl_iaps)