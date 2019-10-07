/*
 * Copyright [2017] Max Planck Society. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <pybind11/eigen.h>
#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>

#include <blmc_robots/random_finger_driver.hpp>
#include <blmc_robots/real_finger.hpp>

using namespace blmc_robots;

PYBIND11_MODULE(py_real_finger, m)
{
    pybind11::module::import("py_finger");
    m.def("create_real_finger_backend", &create_real_finger_backend);

    pybind11::module::import("py_finger");
    m.def("create_random_finger_backend", &create_random_finger_backend);

    pybind11::module::import("py_finger");
    m.def("create_random_finger_backend", &create_random_finger_backend);
}
