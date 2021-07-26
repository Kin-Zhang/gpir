function(add_example name)
  add_executable(${name}
    ${name}.cc
  )
  target_link_libraries(${name}
    gp_planner::gp
    gp_planner::sdf
    gp_planner::initializer
    gp_planner::st_plan
    common::utils
    Python2::Python Python2::NumPy
  ) 
endfunction()

function(add_benchmark name)
  add_executable(${name}
    ${name}.cc
  )
  target_link_libraries(${name}
    gp_planner::gp
    gp_planner::sdf
    gp_planner::initializer
    gp_planner::st_plan
    benchmark::dl_iaps
    common::utils
    Python2::Python Python2::NumPy
  ) 
endfunction()
