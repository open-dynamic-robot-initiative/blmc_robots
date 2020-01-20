#####################
# create some demos #
#####################


include_directories(
  ${PROJECT_SOURCE_DIR}/demos
)

#############################################
# Declare a simple macro to build the demos #
#############################################

macro(create_demo demo_name)
  # Cannot use ARGN directly with list() command,
  # so copy it to a variable first.
  set (extra_macro_args ${ARGN})

  # Did we get any optional args?
  list(LENGTH extra_macro_args num_extra_args)
  if (${num_extra_args} GREATER 0)
      list(GET extra_macro_args 0 robot_name)
  endif ()
  if (${num_extra_args} GREATER 1)
      list(GET extra_macro_args 1 config_file)
  endif ()

  add_executable(demo_${demo_name}
    demo_${demo_name}.cpp
    common_demo_header.hpp
  )

  target_link_libraries(demo_${demo_name}
    ${robot_name}
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
  )

  # first look if a robot_properties_${robot_name} package contain a config file.
  if(config_file)
    if(NOT EXISTS ${config_file})
      message(FATAL_ERROR "demo_${demo_name}: Error cannot find ${config_file}")
    endif()
    target_compile_definitions(demo_${demo_name} PUBLIC
      YAML_PARAMS="${config_file}")
  endif()

endmacro(create_demo demo_name)

###############
# build demos #
###############

create_demo(test_bench_8_motors test_bench_8_motors)
create_demo(solo8 solo8)
create_demo(solo12 solo12)
create_demo(teststand teststand)
create_demo(stuggihop stuggihop)

create_demo(solo8_calibration solo8 "${robot_properties_solo_SOURCE_PREFIX}/config/dgm_parameters_solo8.yaml")
create_demo(solo12_calibration solo12 "${robot_properties_solo_SOURCE_PREFIX}/config/dgm_parameters_solo12.yaml")
create_demo(teststand_calibration teststand "${robot_properties_teststand_SOURCE_PREFIX}/config/dgm_parameters.yaml")

create_demo(finger_position_control)
create_demo(finger_torque_control)
create_demo(foobar)
