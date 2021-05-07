/**
 * \file slider.hpp
 * \author Manuel Wuthrich
 * \date 2018
 * \copyright Copyright (c) 2019, New York University & Max Planck Gesellschaft.
 */

#pragma once

#include <array>

#include <math.h>
#include <Eigen/Eigen>
#include <blmc_drivers/devices/analog_sensor.hpp>
#include <solo/common_header.hpp>

namespace solo
{
class Slider
{
public:
    Slider(std::shared_ptr<blmc_drivers::AnalogSensorInterface> analog_sensor,
           const double& min_position = 0,
           const double& max_position = 1.0)
    {
        analog_sensor_ = analog_sensor;

        min_position_ = min_position;
        max_position_ = max_position;
    }

    double get_position() const
    {
        auto measurement_history = analog_sensor_->get_measurement();

        if (measurement_history->length() == 0)
        {
            return std::numeric_limits<double>::quiet_NaN();
        }

        double measurement = measurement_history->newest_element();
        return min_position_ + measurement * (max_position_ - min_position_);
    }

private:
    std::shared_ptr<blmc_drivers::AnalogSensorInterface> analog_sensor_;

    double min_position_;
    double max_position_;
};

template <int COUNT>
class Sliders
{
public:
    typedef Eigen::Matrix<double, COUNT, 1> Vector;

    typedef std::array<std::shared_ptr<blmc_drivers::AnalogSensorInterface>,
                       COUNT>
        AnalogSensors;

    typedef std::array<std::shared_ptr<blmc_drivers::CanBusMotorBoard>,
                       (COUNT + 1) / 2>
        MotorBoards;

    Sliders(const AnalogSensors& analog_sensors,
            const Vector& min_positions,
            const Vector& max_positions)
    {
        for (size_t i = 0; i < COUNT; i++)
        {
            sliders_[i] = std::make_shared<Slider>(
                analog_sensors[i], min_positions[i], max_positions[i]);
        }
    }

    Sliders(const MotorBoards& motor_boards,
            const Vector& min_positions,
            const Vector& max_positions)
        : Sliders(
              create_analog_sensors(motor_boards), min_positions, max_positions)
    {
    }

    Vector get_positions() const
    {
        Vector positions;

        for (size_t i = 0; i < COUNT; i++)
        {
            positions(i) = sliders_[i]->get_position();
        }
        return positions;
    }

private:
    std::array<std::shared_ptr<Slider>, COUNT> sliders_;

    static AnalogSensors create_analog_sensors(const MotorBoards& motor_boards)
    {
        AnalogSensors analog_sensors;

        for (size_t i = 0; i < COUNT; i++)
        {
            analog_sensors[i] = std::make_shared<blmc_drivers::AnalogSensor>(
                motor_boards[i / 2], i % 2);
        }

        return analog_sensors;
    }
};

}  // namespace solo
