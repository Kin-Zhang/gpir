add_library(common_frenet)
add_library(common::frenet ALIAS common_frenet)

target_sources(common_frenet PRIVATE
  frenet_transform.cc  
)
target_link_libraries(common_frenet
  common::utils
)

if (BUILD_TEST)
  add_gtest(frenet_transform_test frenet_transform_test.cc common_frenet)
endif()