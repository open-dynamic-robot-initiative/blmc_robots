/**
 * \author Manuel Wuthrich
 * \date 2018
 *
 * This file declares the OldRealDisentanglementPlatform class which defines the test
 * bench with 8 motors.
 */

#pragma once


#include <Eigen/Eigen>
#include <blmc_robots/common_header.hpp>
#include <math.h>

#include <blmc_robots/blmc_joint_module.hpp>
#include <blmc_robots/slider.hpp>

#include <robot_interfaces/disentanglement_platform.hpp>



namespace blmc_robots
{

/**
 * @brief The OldRealDisentanglementPlatform class implements the control of the test bench
 * containing 8 motors and 8 sliders using the blmc drivers.
 */
class OldDisentanglementPlatform: public BlmcJointModules<2>
{
public:
    enum JointIndexing {base, tip, joint_count};

    OldDisentanglementPlatform(const std::array<std::shared_ptr<blmc_drivers::MotorInterface>, 2>& motors,
           const std::array<std::shared_ptr<blmc_drivers::AnalogSensorInterface>, 2>& sliders):
        BlmcJointModules<2>(motors,
                0.02 * Eigen::Vector2d::Ones(),
                9.0 * Eigen::Vector2d::Ones(),
                Eigen::Vector2d::Zero(),
                Eigen::Vector2d::Ones() * 2),
        sliders_(sliders,
                Eigen::Vector2d::Zero(),
                Eigen::Vector2d::Ones()) {}

    const Eigen::Vector2d get_slider_positions()
    {
        return sliders_.get_positions();
    }

private:

    Sliders<2> sliders_;
};





class RealDisentanglementPlatform:
        public BlmcJointModules<3>,
        public robot_interfaces::DisentanglementPlatform
{
public:
    typedef Eigen::Vector3d Vector;


    typedef std::array<std::shared_ptr<blmc_drivers::MotorInterface>, 3> Motors;
    typedef std::array<std::shared_ptr<blmc_drivers::CanBusMotorBoard>, 2>
    MotorBoards;

    RealDisentanglementPlatform(const MotorBoards& motor_boards):
        RealDisentanglementPlatform(create_motors(motor_boards))
    {
        motor_boards_ = motor_boards;

        calibrate();
        pause_motors();
    }

private:
    RealDisentanglementPlatform(const Motors& motors):
        BlmcJointModules<3>(motors,
                0.02 * Vector::Ones(),
                Vector(9.79 * 9, 9, 9),
                Vector::Zero(),
                Vector::Ones() * 2)
    {


    }

public:
    Vector get_measured_torques() const
    {
        return BlmcJointModules<3>::get_measured_torques();
    }

    Vector get_measured_angles() const
    {
        return BlmcJointModules<3>::get_measured_angles();
    }

    Vector get_measured_velocities() const
    {
        return BlmcJointModules<3>::get_measured_velocities();
    }

    void pause_motors()
    {
        motor_boards_[0]->pause_motors();
        motor_boards_[1]->pause_motors();
    }

    void wait_for_execution() const
    {
        /// \todo: this needs to be filled in
    }


    /// \todo: this is identical to the finger robot and could be taken out of
    /// class
    static MotorBoards
    create_motor_boards(const std::string& can_0, const std::string& can_1)
    {
        // setup can buses -----------------------------------------------------
        std::array<std::shared_ptr<blmc_drivers::CanBus>, 2> can_buses;
        can_buses[0] = std::make_shared<blmc_drivers::CanBus>(can_0);
        can_buses[1] = std::make_shared<blmc_drivers::CanBus>(can_1);

        // set up motor boards -------------------------------------------------
        MotorBoards motor_boards;
        motor_boards[0] =
                std::make_shared<blmc_drivers::CanBusMotorBoard>(can_buses[0]);
        motor_boards[1] =
                std::make_shared<blmc_drivers::CanBusMotorBoard>(can_buses[1]);

        motor_boards[0]->wait_until_ready();
        motor_boards[1]->wait_until_ready();

        return motor_boards;
    }


private:
    void apply_torques(const Vector& desired_torques)
    {
        BlmcJointModules<3>::set_torques(desired_torques);
        BlmcJointModules<3>::send_torques();
    }


    MotorBoards motor_boards_;

    /// \todo: this is identical to the finger robot and could be taken out of
    /// class
    static Motors
    create_motors(const MotorBoards& motor_boards)
    {
        // set up motors -------------------------------------------------------
        Motors motors;
        motors[0]  = std::make_shared<blmc_drivers::Motor>(motor_boards[0], 0);
        motors[1]  = std::make_shared<blmc_drivers::Motor>(motor_boards[0], 1);
        motors[2]  = std::make_shared<blmc_drivers::Motor>(motor_boards[1], 0);

        return motors;
    }


    /// \todo: calibrate needs to be cleaned and should probably not be here
    /// there are two identical copies in disentanglement_platform and finger,
    /// which is disgusting.

    /**
     * @brief this is an initial calibration procedure executed before the actual
     * control loop to have angle readings in an absolute coordinate frame.
     *
     * The finger reaches its joint limits by applying a constant negative torque
     * simultaneously in all the joints. Be aware that this procedure assumes safe
     * motors where the maximum velocities are constrained.
     *
     * The calibration procedure is finished once the joint velocities are zero
     * according to a moving average estimation of them.
     */
    void calibrate()
    {
        /// \todo: this relies on the safety check in the motor right now,
        /// which is maybe not the greatest idea. without the velocity and
        /// torque limitation in the motor this would be very unsafe
        real_time_tools::Spinner spinner;
        spinner.set_period(0.001);
        std::vector<Vector> running_velocities(1000);
        int running_index = 0;
        Vector sum = Vector::Zero();
        while(running_index < 3000 || (sum.maxCoeff() / 1000.0 > 0.001))
        {
            Vector torques = -1 * get_max_torques();
            torques(table) = 0; // we do not calibrate the table since it has no stop
            torques(tip) = -torques(tip);

            constrain_and_apply_torques(torques);
            Vector velocities = get_measured_velocities();
            if (running_index >= 1000)
                sum = sum - running_velocities[running_index % 1000];
            running_velocities[running_index % 1000] = velocities;
            sum = sum + velocities;
            running_index++;
            spinner.spin();
        }
        int count = 0;
        int linearly_decrease_time_steps = 1000;
        int zero_torque_time_steps = 500;
        while(count < linearly_decrease_time_steps)
        {
            Vector torques = ((linearly_decrease_time_steps -
                    count + 0.0) / linearly_decrease_time_steps) *
                    get_max_torques() * -1;
            torques(table) = 0; // we do not calibrate the table since it has no stop
            torques(tip) = -torques(tip);


            constrain_and_apply_torques(torques);
            count++;
            spinner.spin();
        }
        count = 0;
        while(count < zero_torque_time_steps)
        {
            Vector torques = Vector::Zero();
            constrain_and_apply_torques(torques);
            count++;
            spinner.spin();
        }
        Vector angle_offsets = get_measured_angles();
        angle_offsets(table) = 0;

        Vector zero_angles_relative_to_joint_stops(0.0, 0.0, -0.506);
        set_zero_angles(angle_offsets + zero_angles_relative_to_joint_stops);
    }
};

} // namespace blmc_robots
