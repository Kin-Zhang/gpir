find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
)

add_library(planning_core_simulation)
add_library(planning_core::simulation ALIAS planning_core_simulation)

target_sources(planning_core_simulation PRIVATE
  simulator_adapter_factory.cc
)

add_subdirectory(carla)
add_subdirectory(controller)

target_link_libraries(planning_core_simulation
  planning_core::carla
  planning_core::controller
)