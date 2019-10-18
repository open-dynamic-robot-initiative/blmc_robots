/**
 * \file
 * \brief Driver for a "one-joint" robot.
 * \copyright Copyright (c) 2019, New York University and Max Planck
 *            Gesellschaft.
 */

#pragma once

#include <math.h>
#include <Eigen/Eigen>
#include <blmc_robots/common_header.hpp>

#include <blmc_robots/n_joint_blmc_robot_driver.hpp>
#include <robot_interfaces/n_joint_robot_types.hpp>

namespace blmc_robots
{

constexpr double MAX_CURRENT_A = 15.0;

/**
 * @brief Driver for a single joint.
 *
 * Driver for a single BLMC joint.  Mostly intended for testing purposes.
 */
class OneJointDriver : public NJointBlmcRobotDriver<1, 1>
{
public:
    /**
     * @brief Constructor
     *
     * @param can_port  Name of the CAN port to which the joint is connected
     *     (e.g. "can0")
     * @param home_offset_rad  Home offset of the joint.  This is the offset
     *     between the home position (= encoder index found during homing) and
     *     the desired zero position.
     */
    OneJointDriver(const std::string &can_port,
                   const double home_offset_rad = 0.0)
        : OneJointDriver(create_motor_boards({can_port}), home_offset_rad)
    {
    }

private:
    OneJointDriver(const MotorBoards &motor_boards,
                   const double home_offset_rad)
        : NJointBlmcRobotDriver<1, 1>(motor_boards,
                                      create_motors(motor_boards),
                                      {
                                          // MotorParameters
                                          //.max_current_A = 2.0,
                                          .max_current_A = MAX_CURRENT_A,
                                          .torque_constant_NmpA = 0.02,
                                          .gear_ratio = 9.0,
                                      },
                                      {
                                          // CalibrationParameters
                                          .torque_ratio = 0.05,
                                          .control_gain_kp = 3.0,
                                          .control_gain_kd = 0.03,
                                          .position_tolerance_rad = 0.05,
                                          .move_timeout = 2000,
                                      },
                                      //make_vector(0.08),  // safety kd
                                      make_vector(0.00),  // safety kd
                                      true)
    {
        home_offset_rad_ << home_offset_rad;
        initial_position_rad_ << -home_offset_rad;
    }

    static Motors create_motors(const MotorBoards &motor_boards)
    {
        // set up motors
        Motors motors;
        motors[0] = std::make_shared<blmc_drivers::Motor>(motor_boards[0], 0);

        return motors;
    }

    /**
     * @brief Create a Vector with the given value.
     *
     * This is a helper function needed for initializing Vectors with a
     * specific value upon creation.
     *
     * @param value Value to be set in the vector.
     *
     * @return The Vector.
     */
    static Vector make_vector(double value)
    {
        Vector vec;
        vec << value;
        return vec;
    }
};

robot_interfaces::NJointRobotTypes<1>::BackendPtr create_one_joint_backend(
    const std::string &can_0,
    const double home_offset_rad,
    robot_interfaces::NJointRobotTypes<1>::DataPtr robot_data)
{
    constexpr double MAX_ACTION_DURATION_S = 0.003;
    constexpr double MAX_INTER_ACTION_DURATION_S = 0.005;

    std::shared_ptr<robot_interfaces::RobotDriver<
        robot_interfaces::NJointRobotTypes<1>::Action,
        robot_interfaces::NJointRobotTypes<1>::Observation>>
        robot = std::make_shared<OneJointDriver>(can_0, home_offset_rad);

    auto backend =
        std::make_shared<robot_interfaces::NJointRobotTypes<1>::Backend>(
            robot,
            robot_data,
            MAX_ACTION_DURATION_S,
            MAX_INTER_ACTION_DURATION_S);
    backend->set_max_action_repetitions(-1);

    return backend;
}

}  // namespace blmc_robots
