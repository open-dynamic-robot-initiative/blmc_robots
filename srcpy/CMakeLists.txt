#
# Python bindings.
#

# solo12 python module
add_library(py_solo12 MODULE py_solo12.cpp)
target_link_libraries(py_solo12 PRIVATE pybind11::module)
target_link_libraries(py_solo12 PRIVATE ${PYTHON_LIBRARIES})
target_link_libraries(py_solo12 PRIVATE ${PROJECT_NAME})
target_link_libraries(py_solo12 PRIVATE solo12)
set_target_properties(py_solo12 PROPERTIES PREFIX ""
                                           SUFFIX "${PYTHON_MODULE_EXTENSION}")
target_include_directories(
  py_solo12
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include> SYSTEM
  PUBLIC ${PYTHON_INCLUDE_DIRS})
_ament_cmake_python_get_python_install_dir()
install(TARGETS py_solo12 DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME})

if(${dynamic_graph_manager_FOUND})
  # solo12_hwp python module
  add_library(solo12_hwp_cpp MODULE
    solo12_hwp.cpp
    ../src/dynamic_graph_manager/hwp_solo12.cpp
  )
  target_link_libraries(solo12_hwp_cpp PRIVATE pybind11::module)
  target_link_libraries(solo12_hwp_cpp PRIVATE ${PYTHON_LIBRARIES})
  target_link_libraries(solo12_hwp_cpp PRIVATE ${PROJECT_NAME})
  target_link_libraries(solo12_hwp_cpp PRIVATE solo12)
    target_link_libraries(solo12_hwp_cpp PRIVATE dynamic_graph_manager::dynamic_graph_manager)
  set_target_properties(solo12_hwp_cpp PROPERTIES PREFIX ""
                                            SUFFIX "${PYTHON_MODULE_EXTENSION}")
  target_include_directories(
    solo12_hwp_cpp
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include> SYSTEM
    PUBLIC ${PYTHON_INCLUDE_DIRS})
  _ament_cmake_python_get_python_install_dir()
  install(TARGETS solo12_hwp_cpp DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME})
endif(${dynamic_graph_manager_FOUND})
