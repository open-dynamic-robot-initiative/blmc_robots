#
# Copyright (c) 2019, New York University and Max Planck Gesellschaft.
#
# License BSD-3 clause
#

#
# set up the project
#
cmake_minimum_required(VERSION 3.10.2)

project(solo)

# Using C++17
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

OPTION(BUILD_ROS_DYNAMIC_GRAPH "Build with ROS and DynamicGraph" ON)

#
# Dependencies
#

# build tools
find_package(ament_cmake REQUIRED)
find_package(mpi_cmake_modules REQUIRED)

# depend on ament macros
find_package(blmc_drivers REQUIRED)
find_package(odri_control_interface REQUIRED)
find_package(master_board_sdk REQUIRED)
find_package(real_time_tools REQUIRED)
find_package(pybind11 REQUIRED)
find_package(yaml_utils REQUIRED)
find_package(Eigen3 REQUIRED)
# Find resources from robot_properties packages.
find_package(PythonModules COMPONENTS robot_properties_solo)

find_package(dynamic_graph_manager QUIET)


# export de dependencies
ament_export_dependencies(
  ament_cmake
  mpi_cmake_modules
  ament_index_cpp
  blmc_drivers
  odri_control_interface
  master_board_sdk
  real_time_tools
  pybind11
  yaml_utils
  Eigen3
  dynamic_graph_manager)

# prepare the final export
ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

#
# manage the creation of the libraries and executables
#
add_subdirectory(src)

#
# Manage the demos.
#
add_subdirectory(demos)

#
# Python bindings.
#
add_subdirectory(srcpy)

#
# Install the package
#

# Install the include files
install(DIRECTORY include/ DESTINATION include)

# Install python files.
get_python_install_dir(python_install_dir)
install(
  DIRECTORY python/${PROJECT_NAME}
  DESTINATION "${python_install_dir}"
  PATTERN "*.pyc" EXCLUDE
  PATTERN "__pycache__" EXCLUDE)

#
# Building documentation.
#
add_documentation()

#
# Export as an ament_package
#
ament_package()
