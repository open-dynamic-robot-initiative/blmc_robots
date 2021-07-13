set(all_src_targets)
set(all_exe_targets)

#
# Declare the project library.
#
add_library(${PROJECT_NAME} INTERFACE)
# Add the include dependencies.
target_include_directories(
  ${PROJECT_NAME} INTERFACE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>)
# Link the dependencies.
target_link_libraries(${PROJECT_NAME} INTERFACE blmc_drivers::blmc_drivers)
target_link_libraries(${PROJECT_NAME}
                      INTERFACE odri_control_interface::odri_control_interface)
target_link_libraries(${PROJECT_NAME}
                      INTERFACE master_board_sdk::master_board_sdk)
target_link_libraries(${PROJECT_NAME}
                      INTERFACE real_time_tools::real_time_tools)
target_link_libraries(${PROJECT_NAME} INTERFACE yaml_utils::yaml_utils)
target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)
# Export the target.
list(APPEND all_src_targets ${PROJECT_NAME})

#
# `create_robots_library` builds a robot driver library.
#
macro(create_robots_library lib_name)
  add_library(${lib_name} SHARED ${lib_name}.cpp)
  # Add the include dependencies.
  target_include_directories(
    ${lib_name} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                       $<INSTALL_INTERFACE:include>)
  # Link the dependencies.
  target_link_libraries(${lib_name} ${PROJECT_NAME})
  # Export the target.
  list(APPEND all_src_targets ${lib_name})
endmacro()
create_robots_library(solo8)
create_robots_library(solo8ti)
create_robots_library(solo12)

#
# Build executables like the calibration programs.
#
macro(build_programs program_name robot_name)
  add_executable(${program_name} programs/${program_name}.cpp)
  # Add the include dependencies.
  target_include_directories(
    ${program_name} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include>)
  # Link the dependencies.
  target_link_libraries(${program_name} ${robot_name})
  # Export the target.
  list(APPEND all_src_targets ${program_name})
  list(APPEND all_exe_targets ${program_name})
endmacro()
build_programs(solo8_hardware_calibration solo8)
build_programs(solo8ti_hardware_calibration solo8ti)
build_programs(solo12_hardware_calibration solo12)

#
# Optionally build the DynamiGraphManager main programs.
#
if(${dynamic_graph_manager_FOUND} AND BUILD_ROS_DYNAMIC_GRAPH)
  message(STATUS "Build the dynamic_graph_manager main programs.")
  macro(build_dgm_programs robot_name)
    add_executable(dg_main_${robot_name}
      dynamic_graph_manager/dgm_${robot_name}.cpp
      dynamic_graph_manager/dg_main_${robot_name}.cpp
    )
    # Add the include dependencies.
    target_include_directories(
      dg_main_${robot_name} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>)
    # Link the dependencies.
    target_link_libraries(dg_main_${robot_name}
      ${robot_name} dynamic_graph_manager::dynamic_graph_manager)
    # Get the yaml configuration for the DGM.
    string(
    CONCAT dgm_yaml_path
         "${PythonModules_robot_properties_solo_PATH}/"
         "robot_properties_solo/robot_properties_solo/dynamic_graph_manager/")
    target_compile_definitions(dg_main_${robot_name}
      PUBLIC ROBOT_PROPERTIES_YAML_PATH="${dgm_yaml_path}")
    # Export the target.
    list(APPEND all_src_targets dg_main_${robot_name})
    list(APPEND all_exe_targets dg_main_${robot_name})
  endmacro()
  build_dgm_programs(solo8)
  build_dgm_programs(solo8ti)
  build_dgm_programs(solo12)
endif(${dynamic_graph_manager_FOUND} AND BUILD_ROS_DYNAMIC_GRAPH)

#
# Install and Export the libraries.
#
install(
  TARGETS ${all_src_targets}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES
  DESTINATION include)

foreach(target ${all_exe_targets})
  # install a symlink of the executable in lib/bolt
  string(
    CONCAT symlink_command
           "execute_process("
           "  COMMAND ${CMAKE_COMMAND} -E make_directory "
           "  ${CMAKE_INSTALL_PREFIX}/bin/)\n"
           "execute_process("
           "  COMMAND ${CMAKE_COMMAND} -E create_symlink "
           "  ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/${target}"
           "  ${CMAKE_INSTALL_PREFIX}/bin/${target} )"
  )
  install(CODE ${symlink_command})
endforeach()