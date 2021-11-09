find_package(PkgConfig)
pkg_search_module(IPOPT ipopt)
pkg_search_module(ADOLC adolc)

add_subdirectory(proto)

add_library(benchmark_tdr_obca)
add_library(benchmark::tdr_obca ALIAS benchmark_tdr_obca)
target_sources(benchmark_tdr_obca PRIVATE
	file.cc
	distance_approach_ipopt_interface.cc
	distance_approach_ipopt_relax_end_interface.cc
	distance_approach_problem.cc
	interface.cc
	dual_variable_warm_start_ipopt_interface.cc
	dual_variable_warm_start_osqp_interface.cc
	dual_variable_warm_start_problem.cc
)

target_link_libraries(benchmark_tdr_obca 
	ipopt
	adolc
	benchmark::dl_iaps
	benchmark::proto_tdr_obca
)

add_executable(tdr_obca_test tdr_obca_test.cc)
target_link_libraries(tdr_obca_test benchmark_tdr_obca)
