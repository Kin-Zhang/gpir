find_package(Python3 COMPONENTS NumPy)# 
add_subdirectory(DL_IAPS)
add_subdirectory(TDR_OBCA)

add_benchmark(curvature_benchmark)
add_benchmark(additional_benchmark)