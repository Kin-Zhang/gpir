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
    Python3::Python Python3::NumPy
  ) 
endfunction()

function(add_benchmark name)
  add_executable(${name}
    ${name}.cc
    vehicle_config.cc
  )
  target_link_libraries(${name}
    gp_planner::gp
    gp_planner::sdf
    gp_planner::initializer
    gp_planner::st_plan
    benchmark::dl_iaps
    benchmark::tdr_obca
    common::utils
    Python3::Python Python3::NumPy
  ) 
endfunction()
