/**
 * \file
 * \brief Base driver for a generic n-joint BLMC robot.
 * \copyright Copyright (c) 2019, New York University and Max Planck
 *            Gesellschaft.
 */

#define TPL_NJBRD \
    template <typename Observation, size_t N_JOINTS, size_t N_MOTOR_BOARDS>
#define NJBRD NJointBlmcRobotDriver<Observation, N_JOINTS, N_MOTOR_BOARDS>

namespace blmc_robots
{
TPL_NJBRD
void NJBRD::Config::print() const
{
    std::cout << "Configuration:\n"
              << "\t can_ports:";
    for (const auto &port : can_ports)
    {
        std::cout << " " << port;
    }
    std::cout << "\n"
              << "\t max_current_A: " << max_current_A << "\n"
              << "\t has_endstop: " << has_endstop << "\n"
              << "\t calibration: "
              << "\n"
              << "\t\t endstop_search_torques_Nm: "
              << calibration.endstop_search_torques_Nm.transpose() << "\n"
              << "\t\t position_tolerance_rad: "
              << calibration.position_tolerance_rad << "\n"
              << "\t\t move_timeout: " << calibration.move_timeout << "\n"
              << "\t safety_kd: " << safety_kd.transpose() << "\n"
              << "\t position_control_gains: "
              << "\n"
              << "\t\t kp: " << position_control_gains.kp.transpose() << "\n"
              << "\t\t kd: " << position_control_gains.kd.transpose() << "\n"
              << "\t home_offset_rad: " << home_offset_rad.transpose() << "\n"
              << "\t initial_position_rad: " << initial_position_rad.transpose()
              << "\n"
              << std::endl;
}

TPL_NJBRD
typename NJBRD::Config NJBRD::Config::load_config(
    const std::string &config_file_name)
{
    NJBRD::Config config;
    YAML::Node user_config;

    try
    {
        user_config = YAML::LoadFile(config_file_name);
    }
    catch (...)
    {
        std::cout << "FATAL: Failed to load configuration from '"
                  << config_file_name << "'." << std::endl;
        std::exit(1);
    }

    // replace values from the default config with the ones given in the
    // users config file

    // TODO: for some reason direct conversion is not working (despite
    // yaml-cpp implementing a generic conversion for std::array)
    // set_config_value<CanPortArray>(user_config, "can_ports",
    // &config.can_ports);
    try
    {
        for (size_t i = 0; i < config.can_ports.size(); i++)
        {
            config.can_ports[i] = user_config["can_ports"][i].as<std::string>();
        }
    }
    catch (...)
    {
        std::cerr << "FATAL: Failed to load parameter 'can_ports' from "
                     "configuration file"
                  << std::endl;
        std::exit(1);
    }

    set_config_value(user_config, "max_current_A", &config.max_current_A);
    set_config_value(user_config, "has_endstop", &config.has_endstop);

    if (user_config["calibration"])
    {
        YAML::Node calib = user_config["calibration"];

        set_config_value(calib,
                         "endstop_search_torques_Nm",
                         &config.calibration.endstop_search_torques_Nm);
        set_config_value(calib,
                         "position_tolerance_rad",
                         &config.calibration.position_tolerance_rad);
        set_config_value(
            calib, "move_timeout", &config.calibration.move_timeout);
    }

    set_config_value(user_config, "safety_kd", &config.safety_kd);

    if (user_config["position_control_gains"])
    {
        YAML::Node pos_ctrl = user_config["position_control_gains"];

        set_config_value(pos_ctrl, "kp", &config.position_control_gains.kp);
        set_config_value(pos_ctrl, "kd", &config.position_control_gains.kd);
    }

    set_config_value(user_config, "home_offset_rad", &config.home_offset_rad);
    set_config_value(
        user_config, "initial_position_rad", &config.initial_position_rad);

    return config;
}

TPL_NJBRD
template <typename T>
void NJBRD::Config::set_config_value(const YAML::Node &user_config,
                                     const std::string &name,
                                     T *var)
{
    try
    {
        *var = user_config[name].as<T>();
    }
    catch (const YAML::Exception &e)
    {
        std::cerr << "FATAL: Failed to load parameter '" << name
                  << "' from configuration file" << std::endl;
        std::exit(1);
    };
}

TPL_NJBRD
typename NJBRD::MotorBoards NJBRD::create_motor_boards(
    const std::array<std::string, N_MOTOR_BOARDS> &can_ports)
{
    // setup can buses -----------------------------------------------------
    std::array<std::shared_ptr<blmc_drivers::CanBus>, N_MOTOR_BOARDS> can_buses;
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

TPL_NJBRD
void NJBRD::pause_motors()
{
    for (size_t i = 0; i < motor_boards_.size(); i++)
    {
        motor_boards_[i]->pause_motors();
    }
}

TPL_NJBRD
typename NJBRD::Vector NJBRD::get_measured_index_angles() const
{
    return joint_modules_.get_measured_index_angles();
}

TPL_NJBRD
void NJBRD::initialize()
{
    // Initialization is moving the robot and thus needs to be executed in
    // a real-time thread.  This method only starts the thread and waits
    // for it to finish.  Actual implementation of initialization is in
    // `_initialize()`.

    real_time_tools::RealTimeThread realtime_thread;
    realtime_thread.create_realtime_thread(
        [](void *instance_pointer) {
            // instance_pointer = this, cast to correct type and call the
            // _initialize() method.
            ((NJBRD *)(instance_pointer))->_initialize();
            return (void *)nullptr;
        },
        this);
    realtime_thread.join();
}

TPL_NJBRD
typename NJBRD::Action NJBRD::apply_action(const NJBRD::Action &desired_action)
{
    if (!is_initialized_)
    {
        throw std::runtime_error(
            "Robot needs to be initialized before applying actions.  Run "
            "the `initialize()` method.");
    }

    return apply_action_uninitialized(desired_action);
}

TPL_NJBRD
std::string NJBRD::get_error()
{
    // Checks each board for errors and translates the error codes into
    // human-readable strings.  If multiple boards have errors, the messages
    // are concatenated.  Each message is prepended with the index of the
    // corresponding board.

    std::string error_msg = "";

    for (size_t i = 0; i < motor_boards_.size(); i++)
    {
        auto status_timeseries = motor_boards_[i]->get_status();
        if (status_timeseries->length() > 0)
        {
            std::string board_error_msg = "";
            using ErrorCodes = blmc_drivers::MotorBoardStatus::ErrorCodes;
            switch (status_timeseries->newest_element().error_code)
            {
                case ErrorCodes::NONE:
                    break;
                case ErrorCodes::ENCODER:
                    board_error_msg = "Encoder Error";
                    break;
                case ErrorCodes::CAN_RECV_TIMEOUT:
                    board_error_msg = "CAN Receive Timeout";
                    break;
                case ErrorCodes::CRIT_TEMP:
                    board_error_msg = "Critical Temperature";
                    break;
                case ErrorCodes::POSCONV:
                    board_error_msg =
                        "Error in SpinTAC Position Convert module";
                    break;
                case ErrorCodes::POS_ROLLOVER:
                    board_error_msg = "Position Rollover";
                    break;
                case ErrorCodes::OTHER:
                    board_error_msg = "Other Error";
                    break;
                default:
                    board_error_msg = "Unknown Error";
                    break;
            }

            if (!board_error_msg.empty())
            {
                if (!error_msg.empty())
                {
                    error_msg += "  ";
                }

                // error of the board with board index to the error message
                // string
                error_msg +=
                    "[Board " + std::to_string(i) + "] " + board_error_msg;
            }
        }
    }

    return error_msg;
}

TPL_NJBRD
void NJBRD::shutdown()
{
    pause_motors();
}

TPL_NJBRD
typename NJBRD::Action NJBRD::apply_action_uninitialized(
    const NJBRD::Action &desired_action)
{
    double start_time_sec = real_time_tools::Timer::get_current_time_sec();

    Observation observation = get_latest_observation();

    Action applied_action =
        process_desired_action(desired_action,
                               observation,
                               max_torque_Nm_,
                               config_.safety_kd,
                               config_.position_control_gains.kp,
                               config_.position_control_gains.kd);

    joint_modules_.set_torques(applied_action.torque);
    joint_modules_.send_torques();

    real_time_tools::Timer::sleep_until_sec(start_time_sec + 0.001);

    return applied_action;
}

TPL_NJBRD
void NJBRD::_initialize()
{
    joint_modules_.set_position_control_gains(
        config_.position_control_gains.kp, config_.position_control_gains.kd);

    is_initialized_ = homing(config_.calibration.endstop_search_torques_Nm,
                             config_.home_offset_rad);

    if (is_initialized_)
    {
        bool reached_goal =
            move_to_position(config_.initial_position_rad,
                             config_.calibration.position_tolerance_rad,
                             config_.calibration.move_timeout);
        if (!reached_goal)
        {
            rt_printf("Failed to reach goal, timeout exceeded.\n");
        }
    }

    pause_motors();
}

TPL_NJBRD
bool NJBRD::homing(NJBRD::Vector endstop_search_torques_Nm,
                   NJBRD::Vector home_offset_rad)
{
    //! Distance after which encoder index search is aborted.
    //! Computed based on gear ratio to be 1.5 motor revolutions.
    const double INDEX_SEARCH_DISTANCE_LIMIT_RAD =
        (1.5 / motor_parameters_.gear_ratio) * 2 * M_PI;
    //! Absolute step size when moving for encoder index search.
    constexpr double INDEX_SEARCH_STEP_SIZE_RAD = 0.001;

    //! Distance travelled during homing (useful for home offset calibration)
    Vector travelled_distance = Vector::Zero();

    rt_printf("Start homing.\n");
    if (has_endstop_)
    {
        //! Min. number of steps when moving to the end stop.
        constexpr uint32_t MIN_STEPS_MOVE_TO_END_STOP = 1000;
        //! Size of the window when computing average velocity.
        constexpr uint32_t SIZE_VELOCITY_WINDOW = 100;
        //! Velocity limit at which the joints are considered to be stopped.
        constexpr double STOP_VELOCITY = 0.01;

        static_assert(MIN_STEPS_MOVE_TO_END_STOP > SIZE_VELOCITY_WINDOW,
                      "MIN_STEPS_MOVE_TO_END_STOP has to be bigger than"
                      " SIZE_VELOCITY_WINDOW to ensure correct computation"
                      " of average velocity.");

        // Move until velocity drops to almost zero (= joints hit the end
        // stops) but at least for MIN_STEPS_MOVE_TO_END_STOP time steps.
        // TODO: add timeout to this loop?
        std::vector<Vector> running_velocities(SIZE_VELOCITY_WINDOW);
        Vector summed_velocities = Vector::Zero();
        Vector start_position = get_latest_observation().position;
        uint32_t step_count = 0;
        while (step_count < MIN_STEPS_MOVE_TO_END_STOP ||
               (summed_velocities.maxCoeff() / SIZE_VELOCITY_WINDOW >
                STOP_VELOCITY))
        {
            apply_action_uninitialized(endstop_search_torques_Nm);
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

#ifdef VERBOSE
            Eigen::IOFormat commainitfmt(
                4, Eigen::DontAlignCols, " ", " ", "", "", "", "");
            std::cout << ((summed_velocities / SIZE_VELOCITY_WINDOW).array() >
                          STOP_VELOCITY)
                             .format(commainitfmt)
                      << std::endl;
#endif
        }
        rt_printf("Reached end stop.\n");

        // compute distance travelled during end-stop search
        travelled_distance +=
            get_latest_observation().position - start_position;
    }

    // Home on encoder index

    // Set the search direction for each joint opposite to the end-stop search
    // direction.
    Vector index_search_step_sizes;
    for (unsigned int i = 0; i < N_JOINTS; i++)
    {
        index_search_step_sizes[i] = INDEX_SEARCH_STEP_SIZE_RAD;
        if (endstop_search_torques_Nm[i] > 0)
        {
            index_search_step_sizes[i] *= -1;
        }
    }

    HomingReturnCode homing_status =
        joint_modules_.execute_homing(INDEX_SEARCH_DISTANCE_LIMIT_RAD,
                                      home_offset_rad,
                                      index_search_step_sizes);

    rt_printf("Finished homing.  Offset between end and start position: ");
    travelled_distance += joint_modules_.get_distance_travelled_during_homing();
    for (size_t i = 0; i < N_JOINTS; i++)
    {
        // negate the travelled distance so the output can directly be used as
        // home offset
        rt_printf("%.3f, ", -travelled_distance[i]);
    }
    rt_printf("\n");

    return homing_status == HomingReturnCode::SUCCEEDED;
}

TPL_NJBRD
bool NJBRD::move_to_position(const NJBRD::Vector &goal_pos,
                             const double tolerance,
                             const uint32_t timeout_cycles)
{
    bool reached_goal = false;
    uint32_t cycle_count = 0;

    while (!reached_goal && cycle_count < timeout_cycles)
    {
        apply_action(Action::Position(goal_pos));

        const Vector position_error =
            goal_pos - get_latest_observation().position;
        const Vector velocity = get_latest_observation().velocity;

        // Check if the goal is reached (position error below tolerance and
        // velocity close to zero).
        constexpr double ZERO_VELOCITY = 1e-4;
        reached_goal = ((position_error.array().abs() < tolerance).all() &&
                        (velocity.array().abs() < ZERO_VELOCITY).all());

        cycle_count++;
    }

    return reached_goal;
}

template <size_t N_JOINTS, size_t N_MOTOR_BOARDS>
typename SimpleNJointBlmcRobotDriver<N_JOINTS, N_MOTOR_BOARDS>::Observation
SimpleNJointBlmcRobotDriver<N_JOINTS, N_MOTOR_BOARDS>::get_latest_observation()
{
    Observation observation;

    observation.position = this->joint_modules_.get_measured_angles();
    observation.velocity = this->joint_modules_.get_measured_velocities();
    observation.torque = this->joint_modules_.get_measured_torques();

    return observation;
}

}  // namespace blmc_robots
