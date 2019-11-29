/**
 * @file blmc_joint_module.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @author Manuel Wuthrich
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-07-11
 */

#include <cmath>
#include "real_time_tools/spinner.hpp"
#include "real_time_tools/iostream.hpp"
#include "blmc_robots/blmc_joint_module.hpp"

//#define VERBOSE

namespace blmc_robots{


BlmcJointModule::BlmcJointModule(std::shared_ptr<blmc_drivers::MotorInterface> motor,
                    const double& motor_constant,
                    const double& gear_ratio,
                    const double& zero_angle,
                    const bool& reverse_polarity,
                    const double& max_current)
{
    motor_ = motor;
    motor_constant_ = motor_constant;
    gear_ratio_ = gear_ratio;
    set_zero_angle(zero_angle);
    polarity_ = reverse_polarity ? -1.0:1.0;
    max_current_ = max_current;

    position_control_gain_p_ = 0;
    position_control_gain_d_ = 0;
}

void BlmcJointModule::set_torque(const double& desired_torque)
{
    double desired_current = joint_torque_to_motor_current(desired_torque);

    if(std::fabs(desired_current) > max_current_)
    {
        std::cout << "something went wrong, it should never happen"
                     " that desired_current > "
                  << max_current_
                  << ". desired_current: "
                  << desired_current
                  << std::endl;
        exit(-1);
    }
    motor_->set_current_target(polarity_ * desired_current);
}

void BlmcJointModule::set_zero_angle(const double& zero_angle)
{
    zero_angle_ = zero_angle;
}

void BlmcJointModule::set_joint_polarity(const bool& reverse_polarity)
{
    polarity_ = reverse_polarity ? -1.0 : 1.0;
}
void BlmcJointModule::send_torque()
{
    motor_->send_if_input_changed();
}

double BlmcJointModule::get_max_torque() const
{
    return motor_current_to_joint_torque(max_current_);
}

double BlmcJointModule::get_sent_torque() const
{
    auto measurement_history =
            motor_->get_sent_current_target();

    if(measurement_history->length() == 0)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return motor_current_to_joint_torque(measurement_history->newest_element());
}

double BlmcJointModule::get_measured_torque() const
{
    return motor_current_to_joint_torque(get_motor_measurement(mi::current));
}

double BlmcJointModule::get_measured_angle() const
{
    return get_motor_measurement(mi::position) / gear_ratio_ - zero_angle_;
}

double BlmcJointModule::get_measured_velocity() const
{
    return get_motor_measurement(mi::velocity) / gear_ratio_;
}

double BlmcJointModule::joint_torque_to_motor_current(double torque) const
{
    return torque / gear_ratio_ / motor_constant_;
}

double BlmcJointModule::motor_current_to_joint_torque(double current) const
{
    return current *  gear_ratio_ * motor_constant_;
}

double BlmcJointModule::get_measured_index_angle() const
{
    return get_motor_measurement(mi::encoder_index) / gear_ratio_;
}

double BlmcJointModule::get_zero_angle() const
{
    return zero_angle_;
}

double BlmcJointModule::get_motor_measurement(const mi& measurement_id) const
{
    auto measurement_history =
            motor_->get_measurement(measurement_id);

    if(measurement_history->length() == 0)
    {
        // rt_printf("get_motor_measurement returns NaN\n");
        return std::numeric_limits<double>::quiet_NaN();
    }
    return polarity_ * measurement_history->newest_element();
}

long int BlmcJointModule::get_motor_measurement_index(const mi& measurement_id) const
{
    auto measurement_history =
            motor_->get_measurement(measurement_id);

    if(measurement_history->length() == 0)
    {
        // rt_printf("get_motor_measurement_index returns NaN\n");
        return -1;
    }
    return measurement_history->newest_timeindex();
}

void BlmcJointModule::set_position_control_gains(double kp, double kd)
{
    position_control_gain_p_ = kp;
    position_control_gain_d_ = kd;
}

double BlmcJointModule::execute_position_controller(double target_position_rad) const
{
    double diff = target_position_rad - get_measured_angle();

    // simple PD control
    double desired_torque = position_control_gain_p_ * diff
        - position_control_gain_d_ * get_measured_velocity();

    // clamp torque
    const double max_torque =
        motor_current_to_joint_torque(max_current_) * 0.9;
    if (desired_torque > max_torque) {
        desired_torque = max_torque;
    } else if (desired_torque < -max_torque) {
        desired_torque = -max_torque;
    }

    return desired_torque;
}

bool BlmcJointModule::calibrate(double& angle_zero_to_index,
                                double& index_angle,
                                bool mechanical_calibration)
{
    // save the current position
    double starting_position = get_measured_angle();
    rt_printf("Starting pose is=%f\n", starting_position);

    // reset the ouput
    index_angle = 0.0;

    // we reset the internal zero angle.
    zero_angle_ = 0.0;

    long int last_index_time = get_motor_measurement_index(mi::encoder_index);
    if(std::isnan(last_index_time)){last_index_time = -1;}
    bool reached_next_index = false;
    real_time_tools::Spinner spinner;
    spinner.set_period(0.001);
    rt_printf("Search for the index\n");
    while(!reached_next_index)
    {
      // Small D gain
      double k_d = 0.2;
      // Small desired velocity
      double joint_vel_des = 0.8;
      // Velocity controller
      double actual_velocity = get_measured_velocity();
      double torque = + k_d * (joint_vel_des - actual_velocity);
      // rt_printf("error=%f, vel_des=%f, vel=%f, tau=%f\n", joint_vel_des - actual_velocity, joint_vel_des, actual_velocity, torque);
      // Send the torque command
      set_torque(torque);
      send_torque();
      // check stop
      long int actual_index_time = get_motor_measurement_index(mi::encoder_index);
      double actual_index_angle = get_measured_index_angle();

      reached_next_index = (actual_index_time > last_index_time);
      // rt_printf("last_index_time=%ld, actual_index_time=%ld, actual_index_angle=%f\n", last_index_time, actual_index_time, actual_index_angle);
      if(reached_next_index)
      {
          index_angle = actual_index_angle;
      }
      spinner.spin();
    }
    // reset the control to zero torque
    set_torque(0.0);
    send_torque();
    spinner.spin();

    // get the indexes and stuff
    if(mechanical_calibration)
    {
        angle_zero_to_index = index_angle - starting_position;
    }
    zero_angle_ = index_angle - angle_zero_to_index;

    rt_printf("Zero angle is=%f\n", zero_angle_);
    rt_printf("Zero angle to index angle is=%f\n", angle_zero_to_index);
    rt_printf("Index angle is=%f\n", index_angle);


    rt_printf("Position Control\n");
    // Go to 0
    double init_pose = get_measured_angle();
    double final_pose = 0.0;
    double final_angle = 0.0;
    int traj_time = 2.0 / 0.001;
    int counter = 0;
    double torque_int = 0.0;
    double torque_sat = 0.1; // Nm
    bool reached_zero_pose = 0;
    while(!reached_zero_pose)
    {
      // Small P gain
      double k_p = 2.5;
      // Integrale gain
      double k_i = 0.5;
      // desired pose
      double alpha = 1.0 - (double)((double)counter/(double)traj_time);
      double des_angle = alpha * init_pose + (1.0-alpha) * final_pose;
      // compute the error 
      double current_angle = get_measured_angle();
      double err = des_angle - current_angle;
      // small saturation in intensity
      torque_int += k_i * err * 0.001; // 1 ms sampling period
      if(torque_int > torque_sat){torque_int = torque_sat;}
      if(torque_int < -torque_sat){torque_int = -torque_sat;}
      // Position controller
      double torque = k_p * err + torque_int ;
      // rt_printf("error=%f, torque_int=%f, tau=%f\n", err, torque_int, torque);
      // Send the torque command
      set_torque(torque);
      send_torque();
      // check out
      reached_zero_pose = (std::fabs(current_angle) <= 1e-2); // nearly 0.1 degree
      if(reached_zero_pose)
      {
        final_angle = -err;
      }
      spinner.spin();
      ++counter;
      if(counter > traj_time){counter = traj_time;}
    }
    rt_printf("Final angle is=%f\n", final_angle);
    // reset the control to zero torque
    set_torque(0.0);
    send_torque();
    spinner.spin();

    // FIXME: always returns true as there is no error checking
    return true;
}


void BlmcJointModule::init_homing(int joint_id, double search_distance_limit_rad,
        double home_offset_rad, double profile_step_size_rad)
{
    // reset the internal zero angle.
    set_zero_angle(0.0);

    // TODO: would be nice if the joint instance had a `name` or `id` and class
    // level instead of storing it here (to make more useful debug prints).
    homing_state_.joint_id = joint_id;

    homing_state_.search_distance_limit_rad = search_distance_limit_rad;
    homing_state_.home_offset_rad = home_offset_rad;
    homing_state_.profile_step_size_rad = profile_step_size_rad;
    homing_state_.last_encoder_index_time_index = get_motor_measurement_index(
            mi::encoder_index);
    homing_state_.target_position_rad = get_measured_angle();
    homing_state_.step_count = 0;

    homing_state_.status = HomingReturnCode::RUNNING;
}

HomingReturnCode BlmcJointModule::update_homing()
{
    switch (homing_state_.status) {
        case HomingReturnCode::NOT_INITIALIZED:
            set_torque(0.0);
            send_torque();
            rt_printf("[%d] Homing is not initialized.  Abort.\n",
                      homing_state_.joint_id);
            break;

        case HomingReturnCode::FAILED:
            // when failed, send zero-torque commands
            set_torque(0.0);
            send_torque();
            break;

        case HomingReturnCode::SUCCEEDED: {
            // when succeeded, keep the motor at the home position
            double desired_torque = execute_position_controller(
                    homing_state_.target_position_rad);

            set_torque(desired_torque);
            send_torque();
            break;
        }

        case HomingReturnCode::RUNNING: {

            // number of steps after which the distance limit is reached
            const uint32_t max_step_count =
                std::abs(homing_state_.search_distance_limit_rad /
                         homing_state_.profile_step_size_rad);

            // abort if distance limit is reached
            if (homing_state_.step_count >= max_step_count) {
                set_torque(0.0);
                send_torque();

                homing_state_.status = HomingReturnCode::FAILED;

                rt_printf("[%d] ERROR: Failed to find index\n",
                          homing_state_.joint_id);
                break;
            }

            // -- EXECUTE ONE STEP

            homing_state_.step_count++;
            homing_state_.target_position_rad +=
                homing_state_.profile_step_size_rad;

#ifdef VERBOSE
            const double current_position = get_measured_angle();
            if (homing_state_.step_count % 100 == 0) {
                rt_printf("[%d] cur: %f,\t des: %f\n", homing_state_.joint_id,
                        current_position, homing_state_.target_position_rad);
            }
#endif

            // FIXME: add a safety check to stop if following error gets too big.

            const double desired_torque = execute_position_controller(
                    homing_state_.target_position_rad);
            set_torque(desired_torque);
            send_torque();

            // Check if new encoder index was observed
            const long int actual_index_time = get_motor_measurement_index(
                    mi::encoder_index);
            if (actual_index_time > homing_state_.last_encoder_index_time_index) {
                // -- FINISHED
                const double index_angle = get_measured_index_angle();

                // set the zero angle
                set_zero_angle(index_angle + homing_state_.home_offset_rad);

                // adjust target_position according to the new zero
                homing_state_.target_position_rad -= zero_angle_;

#ifdef VERBOSE
                rt_printf("[%d] Zero angle is=%f\n", homing_state_.joint_id,
                          zero_angle_);
                rt_printf("[%d] Index angle is=%f\n", homing_state_.joint_id,
                          index_angle);
#endif

                homing_state_.status = HomingReturnCode::SUCCEEDED;
            }

            break;
        }
    }

    return homing_state_.status;
}

} // namespace
