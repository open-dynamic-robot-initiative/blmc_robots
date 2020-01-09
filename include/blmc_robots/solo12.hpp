/**
 * \file solo12.hpp
 * \author Julian Viereck
 * \date 21 November 2019
 * \copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.

 */

#pragma onces

#include "blmc_robots/common_header.hpp"
#include <blmc_robots/blmc_joint_module.hpp>
#include <master_board_sdk/master_board_interface.h>

namespace blmc_robots {

class Solo12
{
public:

  /**
   * @brief Solo is the constructor of the class.
   */
  Solo12();

  /**
   * @brief initialize the robot by setting aligning the motors and calibrate the
   * sensors to 0
   * @param if_name Interface for connection to hardware.
   */
  void initialize(const std::string &if_name, const int n_active_motors=12);

  /**
   * @brief Sets the maximum joint torques.
   */
  void set_max_joint_torques(const double& max_joint_torques);

  /**
   * @brief send_target_torques sends the target currents to the motors
   */
  void send_target_joint_torque(
      const Eigen::Ref<Vector12d> target_joint_torque);

  /**
   * @brief acquire_sensors acquire all available sensors, WARNING !!!!
   * this method has to be called prior to any getter to have up to date data.
   */
  void acquire_sensors();

  /**
   * @brief Calibrate the joints by moving to the next joint index position.
   *
   * @param home_offset_rad This is the angle between the index and the zero
   * pose.
   * @return true
   * @return false
   */
  bool calibrate(const Vector12d& home_offset_rad);

  /**
   * Joint properties
   */

  /**
   * @brief get_motor_inertias
   * @return the motor inertias
   */
  const Eigen::Ref<Vector12d> get_motor_inertias()
  {
    return motor_inertias_;
  }

  /**
   * @brief get_motor_torque_constants
   * @return the torque constants of each motor
   */
  const Eigen::Ref<Vector12d> get_motor_torque_constants()
  {
    return motor_torque_constants_;
  }

  /**
   * @brief get_joint_gear_ratios
   * @return  the joint gear ratios
   */
  const Eigen::Ref<Vector12d> get_joint_gear_ratios()
  {
    return joint_gear_ratios_;
  }

  /**
   * @brief get_max_torque
   * @return the max torque that has been hardcoded in the constructor of this
   * class. TODO: parametrize this via yaml or something else.
   */
  const Eigen::Ref<Vector12d> get_motor_max_current()
  {
    return motor_max_current_;
  }

  /**
   * Sensor Data
   */

  /**
   * @brief get_joint_positions
   * @return  the joint angle of each module
   * WARNING !!!!
   * The method <acquire_sensors>"()" has to be called
   * prior to any getter to have up to date data.
   */
  const Eigen::Ref<Vector12d> get_joint_positions()
  {
    return joint_positions_;
  }

  /**
   * @brief get_joint_velocities
   * @return the joint velocities
   * WARNING !!!!
   * The method <acquire_sensors>"()" has to be called
   * prior to any getter to have up to date data.
   */
  const Eigen::Ref<Vector12d> get_joint_velocities()
  {
    return joint_velocities_;
  }

  /**
   * @brief get_joint_torques
   * @return the joint torques
   * WARNING !!!!
   * The method <acquire_sensors>"()" has to be called
   * prior to any getter to have up to date data.
   */
  const Eigen::Ref<Vector12d> get_joint_torques()
  {
    return joint_torques_;
  }

  /**
   * @brief get_joint_torques
   * @return the target joint torques
   * WARNING !!!!
   * The method <acquire_sensors>"()" has to be called
   * prior to any getter to have up to date data.

   */
  const Eigen::Ref<Vector12d> get_joint_target_torques()
  {
    return joint_target_torques_;
  }

  /**
   * @brief get_joint_encoder_index
   * @return the position of the index of the encoders a the motor level
   * WARNING !!!!
   * The method <acquire_sensors>"()" has to be called
   * prior to any getter to have up to date data.
   */
  const Eigen::Ref<Vector12d> get_joint_encoder_index()
  {
    return joint_encoder_index_;
  }

  /**
   * @brief get_contact_sensors_states
   * @return the state of the contacts states
   * WARNING !!!!
   * The method <acquire_sensors>"()" has to be called
   * prior to any getter to have up to date data.
   */
  const Eigen::Ref<Eigen::Vector4d> get_contact_sensors_states()
  {
    return contact_sensors_states_;
  }

  /**
   * @brief get_slider_positions
   * @return the current sliders positions.
   * WARNING !!!!
   * The method <acquire_sensors>"()" has to be called
   * prior to any getter to have up to date data.
   */
  const Eigen::Ref<Eigen::Vector4d> get_slider_positions()
  {
    return slider_positions_;
  }

  /**
   * Hardware Status
   */

  /**
   * @brief get_motor_enabled
   * @return This gives the status (enabled/disabled) of each motors using the
   * joint ordering convention.
   */
  const std::array<bool, 12>& get_motor_enabled()
  {
    return motor_enabled_;
  }

  /**
   * @brief get_motor_ready
   * @return This gives the status (enabled/disabled) of each motors using the
   * joint ordering convention.
   */
  const std::array<bool, 12>& get_motor_ready()
  {
    return motor_ready_;
  }

  /**
   * @brief get_motor_board_enabled
   * @return This gives the status (enabled/disabled of the onboard control cards).
   */
  const std::array<bool, 6>& get_motor_board_enabled()
  {
    return motor_board_enabled_;
  }

  /**
   * @brief get_motor_board_errors
   * @return This gives the status (enabled/disabled of the onboard control cards).
   */
  const std::array<int, 6>& get_motor_board_errors()
  {
    return motor_board_errors_;
  }

  /**
   * @brief has_error
   * @return Returns true if the robot hardware has an error, false otherwise.
   */
  const bool has_error()
  {
    for (const auto& error_code : motor_board_errors_)
    {
      if (error_code != 0) {
        return true;
      }
    }
    return false;
  }

private:
  /**
   * Maps the adc index to the board and adc port and returns its value.
   */
  double get_adc_by_index_(unsigned int adc_index);

  /**
    * Motor data
    */
  int n_active_motors_;


  /**
   * Joint properties
   */
  Vector12d motor_inertias_; /**< motors inertia. */
  Vector12d motor_torque_constants_; /**< DCM motor torque constants. */
  Vector12d joint_gear_ratios_; /**< joint gear ratios (9). */
  Vector12d motor_max_current_; /**< Max appliable current before the robot shutdown. */
  Vector12d joint_zero_positions_; /**< Offset to the theoretical "0" pose. */
  Eigen::Array<double, 12, 1> max_joint_torques_; /**< Max joint torques (Nm) */
  static const double max_joint_torque_security_margin_; /**<  Security margin on the saturation of the control. */

  /**
   * --------------------------------------------------------------------------
   * Hardware status
   */
  /**
   * @brief This gives the status (enabled/disabled) of each motors using the
   * joint ordering convention.
   */
  std::array<bool, 12> motor_enabled_;

  /**
   * @brief This gives the status (enabled/disabled) of each motors using the
   * joint ordering convention.
   */
  std::array<bool, 12> motor_ready_;

  /**
   * @brief This gives the status (enabled/disabled of the onboard control cards).
   */
  std::array<bool, 6> motor_board_enabled_;

  /**
   * @brief This gives the status (enabled/disabled of the onboard control cards).
   */
  std::array<int, 6> motor_board_errors_;

  /**
    * Joint data
    */

  /**
   * @brief joint_positions_
   */
  Vector12d joint_positions_;
  /**
   * @brief joint_velocities_
   */
  Vector12d joint_velocities_;
  /**
   * @brief joint_torques_
   */
  Vector12d joint_torques_;
  /**
   * @brief joint_target_torques_
   */
  Vector12d joint_target_torques_;
  /**
   * @brief joint_encoder_index_
   */
  Vector12d joint_encoder_index_;

  /**
    * --------------------------------------------------------------------------
    * Additional data
    */

  /**
   * @brief slider_positions_ is the position of the linear potentiometer.
   * Can be used as a joystick input.
   */
  Eigen::Vector4d slider_positions_;

  /**
   * @brief contact_sensors_ is contact sensors at each feet of teh quadruped.
   */
  Eigen::Vector4d contact_sensors_states_;

  /**
   * @brief Maps each joint to a motor index.
   */
  std::array<int, 12> joint_to_motor_index_;


  /**
   * @brief motors_ are the objects allowing us to send motor commands and
   * receive data.
   */
  std::array<MotorInterface_ptr, 12> motors_;

  BlmcJointModules<12> joints_;

  /**
   * @brief Address the rotation direction of the motor.
   */
  std::array<bool, 12> reverse_polarities_;

  /**
    * --------------------------------------------------------------------------
    * Drivers communication objects
    */
  std::shared_ptr<MasterBoardInterface> main_board_ptr;
};

} // namespace blmc_robots
