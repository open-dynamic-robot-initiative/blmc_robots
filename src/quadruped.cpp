#include <math.h>
#include "blmc_robots/quadruped.hpp"

namespace blmc_robots{

Quadruped::Quadruped()
{
  /**
    * Motor data
    */
  motor_positions_.setZero();
  motor_velocities_.setZero();
  motor_currents_.setZero();
  motor_torques_.setZero();
  motor_inertias_.setZero();
  motor_encoder_indexes_.setZero();
  motor_target_currents_.setZero();
  motor_target_torques_.setZero();
  motor_torque_constants_.setZero();
  target_motor_current_tmp_.setZero();

  /**
    * Joint data
    */
  joint_positions_.setZero();
  joint_velocities_.setZero();
  joint_torques_.setZero();
  joint_target_torques_.setZero();
  joint_gear_ratios_.setZero();
  joint_hardstop2zero_offsets_.setZero();
  joint_start2hardstop_offsets_.setZero();

  /**
    * Additional data
    */
  slider_positions_.setZero();
  contact_sensors_states_.setZero();
  motor_max_current_.setZero();

  /**
    * Setup some known data
    */

  // for now this value is very small but it is currently for debug mode
  motor_max_current_.fill(15.0);
  motor_torque_constants_.fill(0.025);
  motor_inertias_.fill(0.045);
  joint_gear_ratios_.fill(9.0);
  joint_max_torque_ = motor_max_current_.array() *
                      motor_torque_constants_.array() *
                      joint_gear_ratios_.array();

  for(unsigned i=0; i<polarity_.size(); ++i)
  {
    polarity_[i] = 0.0;
  }
}


void Quadruped::initialize()
{
  // initialize the communication with the can cards
  for(unsigned i=0 ; i<can_buses_.size() ; ++i)
  {
    std::ostringstream oss;
    oss << "can" << i;
    can_buses_[i] = std::make_shared<blmc_drivers::CanBus>(oss.str());
    can_motor_boards_[i] =
        std::make_shared<blmc_drivers::CanBusMotorBoard>(can_buses_[i]);
    sliders_[i] =
        std::make_shared<blmc_drivers::AnalogSensor>(can_motor_boards_[i], 1);
    contact_sensors_[i] =
        std::make_shared<blmc_drivers::AnalogSensor>(can_motor_boards_[i], 0);
  }

  // can 0
  // FR_HFE
  motors_[2] = std::make_shared<blmc_drivers::SafeMotor> (
                 can_motor_boards_[0], 1, motor_max_current_[0]);
  // FR_KFE
  motors_[3] = std::make_shared<blmc_drivers::SafeMotor> (
                 can_motor_boards_[0], 0, motor_max_current_[1]);

  // can 1
  // HR_HFE
  motors_[6] = std::make_shared<blmc_drivers::SafeMotor> (
                 can_motor_boards_[1], 1, motor_max_current_[2]);
  // HR_KFE
  motors_[7] = std::make_shared<blmc_drivers::SafeMotor> (
                 can_motor_boards_[1], 0, motor_max_current_[3]);

  // can 2
  // HL_HFE
  motors_[4] = std::make_shared<blmc_drivers::SafeMotor> (
                 can_motor_boards_[2], 1, motor_max_current_[4]);
  // HL_KFE
  motors_[5] = std::make_shared<blmc_drivers::SafeMotor> (
                 can_motor_boards_[2], 0, motor_max_current_[5]);

  // can 3
  // FL_HFE
  motors_[0] = std::make_shared<blmc_drivers::SafeMotor> (
                 can_motor_boards_[3], 1, motor_max_current_[6]);
  // FL_KFE
  motors_[1] = std::make_shared<blmc_drivers::SafeMotor> (
                 can_motor_boards_[3], 0, motor_max_current_[7]);

  // fix the polarity to be the same as the urdf model.
  polarity_[0] = -1.0; // FL_HFE
  polarity_[1] = -1.0; // FL_KFE
  polarity_[2] =  1.0; // FR_HFE
  polarity_[3] =  1.0; // FR_KFE
  polarity_[4] = -1.0; // HL_HFE
  polarity_[5] = -1.0; // HL_KFE
  polarity_[6] =  1.0; // HR_HFE
  polarity_[7] =  1.0; // HR_KFE

  real_time_tools::Timer::sleep_sec(0.01);
}

void Quadruped::acquire_sensors()
{
  /**
    * Motor data
    */
  for (unsigned i=0 ; i<motors_.size() ; ++i)
  {
    // acquire the motors positions
    motor_positions_(i) =
        polarity_[i] *
        (motors_[i]->get_measurement(mi::position)->newest_element()
        + (joint_start2hardstop_offsets_(i) - joint_hardstop2zero_offsets_(i))
           *joint_gear_ratios_(i));
    // acquire the motors velocities
    motor_velocities_(i) =
        polarity_[i] * motors_[i]->get_measurement(mi::velocity)->newest_element();
    // acquire the motors current
    motor_currents_(i) =
        polarity_[i] * motors_[i]->get_measurement(mi::current)->newest_element();
    // acquire the last sent current sent
    motor_target_currents_(i) =
        polarity_[i] * motors_[i]->get_sent_current_target()->newest_element();
    // acquire the encoder indexes
    motor_encoder_indexes_(i) =
        polarity_[i] *
        (motors_[i]->get_measurement(mi::encoder_index)->length() > 0 ?
          motors_[i]->get_measurement(mi::encoder_index)->newest_element() :
          std::nan(""));
  }
  // acquire the actual motor torques
  motor_torques_ = motor_currents_.array() * motor_torque_constants_.array();
  // acquire the last sent motor torques
  motor_target_torques_ = motor_target_currents_.array() *
                          motor_torque_constants_.array();

  /**
    * Joint data
    */
  // acquire the joint position
  joint_positions_ = motor_positions_.array() / joint_gear_ratios_.array();
  // acquire the joint velocities
  joint_velocities_ = motor_velocities_.array() / joint_gear_ratios_.array();
  // acquire the joint torques
  joint_torques_ = motor_torques_.array() * joint_gear_ratios_.array();
  // acquire the tqrget joint torques
  joint_target_torques_ = motor_target_torques_.array() *
                          joint_gear_ratios_.array();

  /**
    * Additional data
    */
  // acquire the slider positions
  for (unsigned i=0 ; i<slider_positions_.size() ; ++i)
  {
    // acquire the slider
    slider_positions_(i) = sliders_[i]->get_measurement()->newest_element();
    // acquire the current contact states
    contact_sensors_states_(i) =
        contact_sensors_[i]->get_measurement()->newest_element();
  }
}

void Quadruped::set_hardstop2zero_offsets(const Eigen::Ref<Vector8d> hardstop2zero_offsets)
{
  joint_hardstop2zero_offsets_ = hardstop2zero_offsets;
}

void Quadruped::set_start2hardstop_offsets(const Eigen::Ref<Vector8d> start2hardstop_offsets)
{
  joint_start2hardstop_offsets_ = start2hardstop_offsets;
}


void Quadruped::send_target_motor_current(
    const Eigen::Ref<Vector8d> target_motor_current)
{
  // set up the target current
  for(unsigned i=0 ; i<motors_.size() ; ++i)
  {
    motors_[i]->set_current_target(polarity_[i] * target_motor_current(i));
  }

  // actually send the torques to the robot
  for(unsigned i=0 ; i<motors_.size() ; ++i)
  {
    motors_[i]->send_if_input_changed();
  }
}

void Quadruped::send_target_joint_torque(
    const Eigen::Ref<Vector8d> target_joint_torque)
{
  target_motor_current_tmp_ = target_joint_torque.array() /
                              joint_gear_ratios_.array() /
                              motor_torque_constants_.array();
  // we do not use the polarity here because it is used in
  // send_target_motor_current
  send_target_motor_current(target_motor_current_tmp_);
}

} // namespace blmc_robots
