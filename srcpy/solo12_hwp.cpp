/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2021, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Python bindings for the solo12 hardware process.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>

#include <solo/dynamic_graph_manager/hwp_solo12.hpp>

namespace py = pybind11;
using namespace solo;

PYBIND11_MODULE(solo12_hwp_cpp, m)
{
    py::class_<HWPSolo12>(m, "Solo12HWP")
        .def(py::init<>())
        .def("initialize", &HWPSolo12::initialize)
        .def("run", &HWPSolo12::run)
        .def("calibrate", &HWPSolo12::calibrate_joint_position)
        .def("calibrate_from_yaml", &HWPSolo12::calibrate_joint_position_from_yaml);
}
