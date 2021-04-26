find_package(Python2 COMPONENTS Development NumPy)

add_library(matplotlibcpp INTERFACE)
target_include_directories(matplotlibcpp INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(matplotlibcpp INTERFACE
  Python2::Python Python2::NumPy
)