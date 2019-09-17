/**
 * \file test_bench_8_motors.hh
 * \brief The hardware wrapper of the real Finger robot.
 * \author Manuel Wuthrich
 * \date 2018
 * \copyright Copyright (c) 2019, New York University and Max Planck
 *            Gesellschaft.
 */

#pragma once

#include <math.h>
#include <Eigen/Eigen>
#include <blmc_robots/common_header.hpp>
#include <tuple>

#include <blmc_robots/blmc_joint_module.hpp>
#include <real_time_tools/spinner.hpp>
#include <real_time_tools/timer.hpp>

#include <robot_interfaces/finger.hpp>

namespace blmc_robots
{

constexpr double MAX_CURRENT_A = 20.1;

template <size_t N_JOINTS, size_t N_MOTOR_BOARDS>
class NJointRobotDriver
    : public robot_interfaces::RobotDriver<
          typename robot_interfaces::NJointRobotTypes<N_JOINTS>::Action,
          typename robot_interfaces::NJointRobotTypes<N_JOINTS>::Observation>
{
public:
    typedef
        typename robot_interfaces::NJointRobotTypes<N_JOINTS>::Action Action;
    typedef typename robot_interfaces::NJointRobotTypes<N_JOINTS>::Observation
        Observation;
    typedef
        typename robot_interfaces::NJointRobotTypes<N_JOINTS>::Vector Vector;
    typedef std::array<std::shared_ptr<blmc_drivers::MotorInterface>, N_JOINTS>
        Motors;
    typedef std::array<std::shared_ptr<blmc_drivers::CanBusMotorBoard>,
                       N_MOTOR_BOARDS>
        MotorBoards;

    NJointRobotDriver(const std::array<std::string, N_MOTOR_BOARDS> &can_ports,
                      const double max_action_duration_s,
                      const double max_inter_action_duration_s)
        : NJointRobotDriver(create_motor_boards(can_ports),
                            max_action_duration_s, max_inter_action_duration_s)
    {
    }

    NJointRobotDriver(const MotorBoards &motor_boards,
                      const double max_action_duration_s,
                      const double max_inter_action_duration_s)
        : NJointRobotDriver(create_motors(motor_boards), max_action_duration_s,
                            max_inter_action_duration_s)
    {
        motor_boards_ = motor_boards;

        // TODO do not hard-code this!
        max_torque_ = MAX_CURRENT_A * 0.02 * 9.0;

        // TODO do not calibrate in the constructor (this will move the robot!)
        //calibrate();

        for (size_t i = 0; i < motor_boards_.size(); i++)
        {
            motor_boards_[i]->pause_motors();
        }
    }

    static MotorBoards create_motor_boards(
        const std::array<std::string, N_MOTOR_BOARDS> &can_ports)
    {
        // setup can buses -----------------------------------------------------
        std::array<std::shared_ptr<blmc_drivers::CanBus>, N_MOTOR_BOARDS>
            can_buses;
        for (size_t i = 0; i < can_buses.size(); i++)
        {
            can_buses[i] = std::make_shared<blmc_drivers::CanBus>(can_ports[i]);
        }

        // set up motor boards -------------------------------------------------
        MotorBoards motor_boards;
        for (size_t i = 0; i < motor_boards.size(); i++)
        {
            motor_boards[i] = std::make_shared<blmc_drivers::CanBusMotorBoard>(
                can_buses[i], 1000, 10);
            /// \TODO: reduce the timeout further!!
        }

        for (size_t i = 0; i < motor_boards.size(); i++)
        {
            motor_boards[i]->wait_until_ready();
        }

        return motor_boards;
    }

    Vector get_measured_index_angles() const
    {
        return joint_modules_.get_measured_index_angles();
    }

public:
    Observation get_latest_observation() override
    {
        Observation observation;
        observation.angle = joint_modules_.get_measured_angles();
        observation.velocity = joint_modules_.get_measured_velocities();
        observation.torque = joint_modules_.get_measured_torques();
        return observation;
    }

protected:
    static Motors create_motors(const MotorBoards &motor_boards)
    {
        // set up motors -------------------------------------------------------
        Motors motors;
        motors[0] = std::make_shared<blmc_drivers::Motor>(motor_boards[0], 0);
        // FIXME !!!!!! 
        //motors[1] = std::make_shared<blmc_drivers::Motor>(motor_boards[0], 1);
        //motors[2] = std::make_shared<blmc_drivers::Motor>(motor_boards[1], 0);

        return motors;
    }

    NJointRobotDriver(const Motors &motors, const double max_action_duration_s,
                      const double max_inter_action_duration_s)
        : robot_interfaces::RobotDriver<Action, Observation>(
              max_action_duration_s, max_inter_action_duration_s),
          // TODO hard-coded numbers!
          joint_modules_(motors,
                         0.02 * Vector::Ones(),
                         9.0 * Vector::Ones(),
                         Vector::Zero(),
                         Vector::Ones() * MAX_CURRENT_A)
    {
    }

    Action apply_action(const Action &desired_action) override
    {
        double start_time_sec = real_time_tools::Timer::get_current_time_sec();

        Observation observation = get_latest_observation();
        Action applied_action =
            mct::clamp(desired_action, -max_torque_, max_torque_);
        applied_action = applied_action - safety_kd_.cwiseProduct(observation.velocity);
        applied_action = mct::clamp(applied_action, -max_torque_, max_torque_);

        joint_modules_.set_torques(applied_action);
        joint_modules_.send_torques();
        real_time_tools::Timer::sleep_until_sec(start_time_sec + 0.001);

        return applied_action;
    }

    void shutdown() override
    {
        for (size_t i = 0; i < motor_boards_.size(); i++)
        {
            motor_boards_[i]->pause_motors();
        }
    }

    /// \todo: calibrate needs to be cleaned and should probably not be here
    /// there are two identical copies in disentanglement_platform and finger,
    /// which is disgusting.

    /**
     * @brief Homing on negative end stop and encoder index.
     *
     * Procedure for finding an absolute zero position (or "home" position) when
     * using relative encoders.
     *
     * All joints first move in positive direction until the index of each
     * encoder is found.  Then all joints move in negative direction until they
     * hit the end stop.  Home position is set to the positions of encoder
     * indices closest to the end stop.
     *
     * By default, the zero position is the same as the home position.  The
     * optional argument home_offset_rad provides a means to move the zero
     * position
     * relative to the home position.  The zero position is computed as
     *
     *     zero position = encoder index position + home offset
     *
     * Movement is done by simply applying a constant torque to the joints.  The
     * amount of torque is a ratio of the configured maximum torque defined by
     * `torque_ratio`.
     *
     * @param torque_ratio Ratio of max. torque that is used to move the joints.
     * @param home_offset_rad Offset between the home position and the desired
     * zero
     *                    position.
     */
    bool home_on_index_after_negative_end_stop(
        double torque_ratio, Vector home_offset_rad = Vector::Zero())
    {
        // TODO this can be dangerous in generic NJointRobotDriver as not all
        // robots have end stops!

        /// \todo: this relies on the safety check in the motor right now,
        /// which is maybe not the greatest idea. Without the velocity and
        /// torque limitation in the motor this would be very unsafe

        //! Min. number of steps when moving to the end stop.
        constexpr uint32_t MIN_STEPS_MOVE_TO_END_STOP = 1000;
        //! Size of the window when computing average velocity.
        constexpr uint32_t SIZE_VELOCITY_WINDOW = 100;
        //! Velocity limit at which the joints are considered to be stopped.
        constexpr double STOP_VELOCITY = 0.001;
        //! Distance after which encoder index search is aborted.
        constexpr double SEARCH_DISTANCE_LIMIT_RAD = 2.0;

        static_assert(MIN_STEPS_MOVE_TO_END_STOP > SIZE_VELOCITY_WINDOW,
                      "MIN_STEPS_MOVE_TO_END_STOP has to be bigger than"
                      " SIZE_VELOCITY_WINDOW to ensure correct computation"
                      " of average velocity.");


        // FIXME !!!!!!!
        if (has_endstop_)
        {
            // Move until velocity drops to almost zero (= joints hit the end stops)
            // but at least for MIN_STEPS_MOVE_TO_END_STOP time steps.
            // TODO: add timeout to this loop?
            std::vector<Vector> running_velocities(SIZE_VELOCITY_WINDOW);
            Vector summed_velocities = Vector::Zero();
            int step_count = 0;
            while (step_count < MIN_STEPS_MOVE_TO_END_STOP ||
                   (summed_velocities.maxCoeff() / SIZE_VELOCITY_WINDOW >
                    STOP_VELOCITY))
            {
                Vector torques = -1 * torque_ratio * get_max_torques();
                apply_action(torques);
                Vector abs_velocities =
                    get_latest_observation().velocity.cwiseAbs();

                uint32_t running_index = step_count % SIZE_VELOCITY_WINDOW;
                if (step_count >= SIZE_VELOCITY_WINDOW)
                {
                    summed_velocities -= running_velocities[running_index];
                }
                running_velocities[running_index] = abs_velocities;
                summed_velocities += abs_velocities;
                step_count++;
            }
        }

        // need to "pause" as the desired actions queue is not filled while
        // homing is running.

        // Home on encoder index
        HomingReturnCode homing_status = joint_modules_.execute_homing(
            SEARCH_DISTANCE_LIMIT_RAD, home_offset_rad);

        rt_printf("Finished homing.\n");

        return homing_status == HomingReturnCode::SUCCEEDED;
    }

    /**
     * @brief Move to given goal position using PD control.
     *
     * @param goal_pos Angular goal position for each joint.
     * @param kp Gain K_p for the PD controller.
     * @param kd Gain K_d for the PD controller.
     * @param tolerance Allowed position error for reaching the goal.  This is
     *     checked per joint, that is the maximal possible error is +/-tolerance
     *     on each joint.
     * @param timeout_cycles Timeout.  If exceeded before goal is reached, the
     *     procedure is aborted. Unit: Number of control loop cycles.
     * @return True if goal position is reached, false if timeout is exceeded.
     */
    bool move_to_position(const Vector &goal_pos, const double kp,
                          const double kd, const double tolerance,
                          const uint32_t timeout_cycles)
    {
        /// \todo: this relies on the safety check in the motor right now,
        /// which is maybe not the greatest idea. Without the velocity and
        /// torque limitation in the motor this would be very unsafe

        bool reached_goal = false;
        int cycle_count = 0;
        Vector desired_torque = Vector::Zero();
        Eigen::Vector3d last_diff(std::numeric_limits<double>::max(),
                                  std::numeric_limits<double>::max(),
                                  std::numeric_limits<double>::max());

        while (!reached_goal && cycle_count < timeout_cycles)
        {
            apply_action(desired_torque);

            const Vector position_error =
                goal_pos - get_latest_observation().angle;
            const Vector velocity = get_latest_observation().velocity;

            // we implement here a small PD control at the current level
            desired_torque = kp * position_error - kd * velocity;

            // Check if the goal is reached (position error below tolerance and
            // velocity close to zero).
            constexpr double ZERO_VELOCITY = 1e-4;
            reached_goal = ((position_error.array().abs() < tolerance).all() &&
                            (velocity.array().abs() < ZERO_VELOCITY).all());

            cycle_count++;
        }

        return reached_goal;
    }

    /**
     * @brief this is an initial calibration procedure executed before the
     * actual control loop to have angle readings in an absolute coordinate
     * frame.
     *
     * The finger reaches its joint limits by applying a constant negative
     * torque simultaneously in all the joints. Be aware that this procedure
     * assumes safe motors where the maximum velocities are constrained.
     *
     * The calibration procedure is finished once the joint velocities are zero
     * according to a moving average estimation of them.
     */
    void calibrate() override
    {
        // TODO no hard-coded numbers in NJointRobotDriver
        //constexpr double TORQUE_RATIO = 0.1;
        const double TORQUE_RATIO = 0.2 / max_torque_;
        constexpr double CONTROL_GAIN_KP = 0.2;
        constexpr double CONTROL_GAIN_KD = 0.0025;
        constexpr double POSITION_TOLERANCE_RAD = 0.1;
        constexpr double MOVE_TIMEOUT = 2000;

        joint_modules_.set_position_control_gains(CONTROL_GAIN_KP,
                                                  CONTROL_GAIN_KD);

        bool is_homed = home_on_index_after_negative_end_stop(TORQUE_RATIO,
                                                              home_offset_rad_);

        if (is_homed)
        {
            bool reached_goal = move_to_position(
                starting_position_rad_, CONTROL_GAIN_KP, CONTROL_GAIN_KD,
                POSITION_TOLERANCE_RAD, MOVE_TIMEOUT);
            if (!reached_goal)
            {
                rt_printf("Failed to reach goal, timeout exceeded.\n");
            }
        }
    }

protected:
    const bool has_endstop_ = true;  // FIXME !!!!

    /**
     * \brief Offset between home position and zero.
     *
     * Defined such that the zero position is at the negative end stop (for
     * compatibility with old homing method).
     */
    Vector home_offset_rad_ = Vector::Zero();

    //! \brief  Start position to which the robot moves after homing.
    Vector starting_position_rad_ = Vector::Zero();

    Vector safety_kd_ = Vector::Zero();  // TODO this means safety off?!

    /// todo: this should probably go away
    double max_torque_;

    BlmcJointModules<N_JOINTS> joint_modules_;
    MotorBoards motor_boards_;

    /// todo: this should probably go away

public:
    Vector get_max_torques() const { return max_torque_ * Vector::Ones(); }
};

class RealFingerDriver : public NJointRobotDriver<3, 2>
{
public:
    // typedef std::array<std::shared_ptr<blmc_drivers::MotorInterface>, 3>
    // Motors; typedef
    // std::array<std::shared_ptr<blmc_drivers::CanBusMotorBoard>, 2>
    //    MotorBoards;

    RealFingerDriver(const std::string &can_0, const std::string &can_1)
        : NJointRobotDriver<3, 2>({can_0, can_1}, 0.003, 0.005)
    {
        initialize();
    }

    // TODO do we need this constructor?
    // RealFingerDriver(const MotorBoards &motor_boards)
    //    : NJointRobotDriver<3, 2>(motor_boards, 0.003, 0.005)
    //{
    //    initialize();
    //}

protected:
    void initialize()
    {
        safety_kd_ << 0.08, 0.08, 0.04;
        home_offset_rad_ << -0.54, -0.17, 0.0;
        starting_position_rad_ << 1.5, 1.5, 3.0;
    }
};

robot_interfaces::FingerTypes::BackendPtr create_real_finger_backend(
    const std::string &can_0, const std::string &can_1,
    robot_interfaces::FingerTypes::DataPtr robot_data)
{
    std::shared_ptr<robot_interfaces::RobotDriver<
        robot_interfaces::FingerTypes::Action,
        robot_interfaces::FingerTypes::Observation>>
        robot = std::make_shared<RealFingerDriver>(can_0, can_1);

    auto backend = std::make_shared<robot_interfaces::FingerTypes::Backend>(
        robot, robot_data);
    backend->set_max_action_repetitions(-1);

    return backend;
}


// FIXME move this out of here!

class OneJointDriver : public NJointRobotDriver<1, 1>
{
public:

    OneJointDriver(const std::string &can_port, double home_offset_rad)
        : NJointRobotDriver<1, 1>({can_port}, 0.003, 0.005)
    {
        //safety_kd_ << 0.03;
        safety_kd_ << 0.0;
        //home_offset_rad_ << (2.585 - 0.337 + 0.2 + 0.13);
        home_offset_rad_ << home_offset_rad;
        starting_position_rad_ << -home_offset_rad_[0];
    }

};

robot_interfaces::NJointRobotTypes<1>::BackendPtr create_one_joint_backend(
    const std::string &can_0,
    double home_offset_rad,
    robot_interfaces::NJointRobotTypes<1>::DataPtr robot_data)
{
    std::shared_ptr<robot_interfaces::RobotDriver<
        robot_interfaces::NJointRobotTypes<1>::Action,
        robot_interfaces::NJointRobotTypes<1>::Observation>>
        robot = std::make_shared<OneJointDriver>(can_0, home_offset_rad);

    auto backend = std::make_shared<robot_interfaces::NJointRobotTypes<1>::Backend>(
        robot, robot_data);
    backend->set_max_action_repetitions(-1);

    return backend;
}

}  // namespace blmc_robots
