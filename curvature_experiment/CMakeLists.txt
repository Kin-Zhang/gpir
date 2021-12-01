cmake_minimum_required(VERSION 3.0.2)
project(curvature_experiment)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED true)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O0")

find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(osqp REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp tf
  common
)

catkin_package()

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty
  ${catkin_INCLUDE_DIRS}
)

add_executable(curvature_experiment
  simulator/car_simulator.cc
  controller/mpc_controller.cc
  controller/lmpc_osqp_solver.cc
  main.cc
)
target_link_libraries(curvature_experiment
  ${catkin_LIBRARIES}
  osqp::osqp
  glog::glog
  common
)

