function(add_example name)
  add_executable(${name}
    ${name}.cc
  )
  target_link_libraries(${name}
    gp_planner::gp
    gp_planner::sdf
    gp_planner::initializer
    gp_planner::st_plan
    Python2::Python Python2::NumPy
  ) 
endfunction()
