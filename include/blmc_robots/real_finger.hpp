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
/**
 * @brief Parameters related to the motor.
 */
struct MotorParameters
{
    //! @brief Maximum current that can be sent to the motor [A].
    double max_current_A;

    //! @brief Torque constant K_t of the motor [Nm/A].
    double torque_constant_NmpA;

    /**
     * @brief Gear ratio between motor and joint.
     *
     * For a `n:1` ratio (i.e. one joint revolution requires n motor
     * revolutions) set this value to `n`.
     */
    double gear_ratio;
};

/**
 * @brief Parameters for the joint calibration (homing and go to initial pose).
 */
struct CalibrationParameters
{
    //! @brief Ratio of the max. torque that is used to find the end stop.
    double torque_ratio;  // TODO better used fixed torque
    //! @brief P-gain for the position controller.
    double control_gain_kp;
    //! @brief D-gain for the position controller.
    double control_gain_kd;
    //! @brief Tolerance for reaching the starting position.
    double position_tolerance_rad;
    //! @brief Timeout for reaching the starting position.
    double move_timeout;
};

/**
 * @brief Base class for simple n-joint BLMC robots.
 *
 * This is a generic base class to easily implement drivers for simple BLMC
 * robots that consist of a chain of joints.
 *
 * @tparam N_JOINTS Number of joints.
 * @tparam N_MOTOR_BOARDS Number of motor control boards that are used.
 */
template <size_t N_JOINTS, size_t N_MOTOR_BOARDS>
class NJointBlmcRobotDriver
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

    NJointBlmcRobotDriver(const MotorBoards &motor_boards,
                          const Motors &motors,
                          const MotorParameters &motor_parameters,
                          const double max_action_duration_s,
                          const double max_inter_action_duration_s)
        : robot_interfaces::RobotDriver<Action, Observation>(
              max_action_duration_s, max_inter_action_duration_s),
          joint_modules_(motors,
                         motor_parameters.torque_constant_NmpA * Vector::Ones(),
                         motor_parameters.gear_ratio * Vector::Ones(),
                         Vector::Zero()),
          motor_boards_(motor_boards),
          max_torque_Nm_(motor_parameters.max_current_A *
                         motor_parameters.torque_constant_NmpA *
                         motor_parameters.gear_ratio)
    {
        pause_motors();
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

    void pause_motors()
    {
        for (size_t i = 0; i < motor_boards_.size(); i++)
        {
            motor_boards_[i]->pause_motors();
        }
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
    Action apply_action(const Action &desired_action) override
    {
        if (!is_calibrated_)
        {
            throw std::runtime_error(
                "Robot needs to be calibrated before applying actions.  Run "
                "the `calibrate()` method.");
        }

        double start_time_sec = real_time_tools::Timer::get_current_time_sec();

        Observation observation = get_latest_observation();
        Action applied_action =
            mct::clamp(desired_action, -max_torque_Nm_, max_torque_Nm_);
        applied_action =
            applied_action - safety_kd_.cwiseProduct(observation.velocity);
        applied_action =
            mct::clamp(applied_action, -max_torque_Nm_, max_torque_Nm_);

        joint_modules_.set_torques(applied_action);
        joint_modules_.send_torques();
        real_time_tools::Timer::sleep_until_sec(start_time_sec + 0.001);

        return applied_action;
    }

    void shutdown() override
    {
        pause_motors();
    }

    virtual CalibrationParameters get_calibration_parameters() = 0;

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
     *     zero position.
     */
    bool home_on_index_after_negative_end_stop(
        double torque_ratio, Vector home_offset_rad = Vector::Zero())
    {
        // TODO this can be dangerous in generic NJointBlmcRobotDriver as not
        // all robots have end stops!

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
    bool move_to_position(const Vector &goal_pos,
                          const double kp,
                          const double kd,
                          const double tolerance,
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
     * @brief Find home position of all joints and move to start position.
     *
     * Homes all joints using home_on_index_after_negative_end_stop.  When
     * finished, move the joint to the starting position (defined in
     * `starting_position_rad_`).
     *
     */
    void calibrate()
    {
        const CalibrationParameters params = get_calibration_parameters();

        joint_modules_.set_position_control_gains(params.control_gain_kp,
                                                  params.control_gain_kd);

        // For the calibration procedure we need to set the is_calibrated_ flag
        // otherwise the robot cannot move during calibration.
        // TODO: this is dirty, find a better solution.
        is_calibrated_ = true;

        bool is_homed = home_on_index_after_negative_end_stop(
            params.torque_ratio, home_offset_rad_);

        if (is_homed)
        {
            bool reached_goal = move_to_position(starting_position_rad_,
                                                 params.control_gain_kp,
                                                 params.control_gain_kd,
                                                 params.position_tolerance_rad,
                                                 params.move_timeout);
            if (!reached_goal)
            {
                rt_printf("Failed to reach goal, timeout exceeded.\n");
            }

            is_calibrated_ = reached_goal;
        }
        else
        {
            // calibration failed
            is_calibrated_ = false;
        }

        pause_motors();
    }

protected:
    /**
     * \brief Offset between home position and zero.
     *
     * Defined such that the zero position is at the negative end stop (for
     * compatibility with old homing method).
     */
    Vector home_offset_rad_ = Vector::Zero();

    //! \brief Start position to which the robot moves after homing.
    Vector starting_position_rad_ = Vector::Zero();

    //! \brief D-gain to dampen velocity.  Set to zero to disable damping.
    Vector safety_kd_ = Vector::Zero();

    /// todo: this should probably go away
    double max_torque_Nm_;

    BlmcJointModules<N_JOINTS> joint_modules_;
    MotorBoards motor_boards_;

    bool is_calibrated_ = false;

public:
    Vector get_max_torques() const
    {
        return max_torque_Nm_ * Vector::Ones();
    }
};

class RealFingerDriver : public NJointBlmcRobotDriver<3, 2>
{
public:
    RealFingerDriver(const std::string &can_0, const std::string &can_1)
        : RealFingerDriver(create_motor_boards({can_0, can_1}))
    {
    }

private:
    RealFingerDriver(const MotorBoards &motor_boards)
        : NJointBlmcRobotDriver<3, 2>(motor_boards,
                                      create_motors(motor_boards),
                                      {.max_current_A = 2.0,
                                       .torque_constant_NmpA = 0.02,
                                       .gear_ratio = 9.0},
                                      0.003,
                                      0.005)
    {
        initialize();
    }

    static Motors create_motors(const MotorBoards &motor_boards)
    {
        // set up motors
        Motors motors;
        motors[0] = std::make_shared<blmc_drivers::Motor>(motor_boards[0], 0);
        motors[1] = std::make_shared<blmc_drivers::Motor>(motor_boards[0], 1);
        motors[2] = std::make_shared<blmc_drivers::Motor>(motor_boards[1], 0);

        return motors;
    }

    void initialize()
    {
        safety_kd_ << 0.08, 0.08, 0.04;
        home_offset_rad_ << -0.54, -0.17, 0.0;
        starting_position_rad_ << 1.5, 1.5, 3.0;
    }

    CalibrationParameters get_calibration_parameters()
    {
        return {
            .torque_ratio = 0.6,
            .control_gain_kp = 3.0,
            .control_gain_kd = 0.03,
            .position_tolerance_rad = 0.05,
            .move_timeout = 2000,
        };
    }
};

robot_interfaces::FingerTypes::BackendPtr create_real_finger_backend(
    const std::string &can_0,
    const std::string &can_1,
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

}  // namespace blmc_robots
