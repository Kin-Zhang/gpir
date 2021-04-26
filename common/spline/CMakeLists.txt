add_library(common_spline)
add_library(common::spline ALIAS common_spline)

target_sources(common_spline PRIVATE
  quintic_spline2d.cc
)
target_link_libraries(common_spline
  common::solver
  common::utils
)

if(BUILD_TEST)
  # add_gtest(quintic_polynomial_test quintic_polynomial_test.cc common_spline)
  # add_gtest(quintic_spline2d_test quintic_spline2d_test.cc common_spline)
endif()