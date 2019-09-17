/**
 * @file blmc_joint_module.hpp
 * @author Manuel Wuthrich
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-07-11
 */
#pragma once

#include <iostream>
#include <array>
#include <stdexcept>
#include <math.h>
#include <Eigen/Eigen>

#include "blmc_drivers/devices/motor.hpp"
#include "blmc_robots/common_header.hpp"

namespace blmc_robots
{

// TODO what is the best scope for those homing-related types?

/**
 * @brief Possible return values of the homing
 */
enum class HomingReturnCode {
    //! Homing was not initialized and can therefore not be performed.
    NOT_INITIALIZED = 0,
    //! Homing is currently running.
    RUNNING,
    //! Homing is succeeded.
    SUCCEEDED,
    //! Homing failed.
    FAILED
};


/**
 * @brief State variables required for the homing.
 */
struct HomingState {
    //! Id of the joint.  Just use for debug prints.
    int joint_id;
    //! Max. distance to move while searching the encoder index.
    double search_distance_limit_rad;
    //! Offset from home position to zero position.
    double home_offset_rad;
    //! Step size for the position profile.
    double profile_step_size_rad;
    //! Timestamp from when the encoder index was seen the last time.
    long int last_encoder_index_time_index;
    //! Number of profile steps already taken.
    uint32_t step_count;
    //! Current target position of the position profile.
    double target_position_rad;
    //! Current status of the homing procedure.
    HomingReturnCode status;
};



/**
 * @brief The BlmcJointModule class is containing the joint information. It is
 * here to help converting the data from the motor side to the joint side. It
 * also allows the calibration of the joint position during initialization.
 */
class BlmcJointModule
{
public:

    /**
     * @brief Construct a new BlmcJointModule object
     * 
     * @param motor is the C++ object allowing us to send commands and receive
     * sensor data.
     * @param motor_constant (\f$ k \f$) is the torque constant of the motor 
     * \f$ \tau_{motor} = k * i_{motor} \f$
     * @param gear_ratio is the gear ratio between the motor and the joint.
     * @param zero_angle is the angle between the closest positive motor index
     * and the zero configuration.
     * @param reverse_polarity 
     * @param max_current 
     */
    BlmcJointModule(std::shared_ptr<blmc_drivers::MotorInterface> motor,
                    const double& motor_constant,
                    const double& gear_ratio,
                    const double& zero_angle,
                    const bool& reverse_polarity=false,
                    const double& max_current=10.0);

    /**
     * @brief Set the joint torque to be sent.
     * 
     * @param desired_torque 
     */
    void set_torque(const double& desired_torque);

    /**
     * @brief Set the zero_angle. The zero_angle is the angle between the
     * closest positive motor index and the zero configuration.
     * 
     * @param zero_angle 
     */
    void set_zero_angle(const double& zero_angle);

    /**
     * @brief send the joint torque to the motor. The conversion between joint
     * torque and motor current is done automatically.
     */
    void send_torque();

    /**
     * @brief Get the sent joint torque.
     * 
     * @return double 
     */
    double get_sent_torque() const;

    /**
     * @brief Get the measured joint torque.
     * 
     * @return double 
     */
    double get_measured_torque() const;

    /**
     * @brief Get the measured angle of the joint.
     * 
     * @return double 
     */
    double get_measured_angle() const;

    /**
     * @brief Get the measured velocity of the joint. This data is computed on
     * board of the control card.
     * 
     * @return double 
     */
    double get_measured_velocity() const;

    /**
     * @brief Get the measured index angle. There is one index per motor rotation so
     * there are gear_ratio indexes per joint rotation.
     * 
     * @return double 
     */
    double get_measured_index_angle() const;

    /**
     * @brief Get the zero_angle_. These are the angle between the starting pose
     * and the theoretical zero pose.
     * 
     * @return double 
     */
    double get_zero_angle() const;

    /**
     * @brief Set control gains for PD position controller.
     *
     * @param kp P gain.
     * @param kd D gain.
     */
    void set_position_control_gains(double kp, double kd);

    /**
     * @brief Execute one iteration of the position controller.
     *
     * @param target_position_rad  Target position.
     *
     * @return Torque command.
     */
    double execute_position_controller(double target_position_rad) const;

    /**
     * @brief This method calibrate the joint position knowing the angle between
     * the closest (in positive torque) motor index and the theoretical zero
     * pose. Warning, this method should be called in a real time thread!
     * 
     * @param[in][out] angle_zero_to_index this is the angle between the closest (in 
     * positive torque) motor index and the theoretical zero pose. 
     * @param[out] index_angle is the angle where we met the index. This angle
     * is relative to the configuration when the robot booted.
     * @param[in] mechanical_calibration defines if the leg started in the zero
     * configuration or not
     * @return true if success.
     * @return false if problem arose.
     */
    bool calibrate(double& angle_zero_to_index,
                   double& index_angle,
                   bool mechanical_calibration = false);


    /**
     * @brief Initialize the homing procedure.
     *
     * This has to be called before update_homing().
     *
     * @param joint_id ID of the joint.  This is only used for debug prints.
     * @param search_distance_limit_rad  Maximum distance the motor moves while
     *     searching for the encoder index.  Unit: radian.
     * @param home_offset_rad  Offset from home position to zero position.
     *     Unit: radian.
     * @param profile_step_size_rad  Distance by which the target position of the
     *     position profile is changed in each step.  Set to a negative value to
     *     search for the next encoder index in negative direction.  Unit:
     *     radian.
     */
    void init_homing(int joint_id, double search_distance_limit_rad, double
            home_offset_rad, double profile_step_size_rad=0.001);

    /**
     * @brief Perform one step of homing on encoder index.
     *
     * Searches for the next encoder index in positive direction and, when
     * found, sets it as home position.
     *
     * Only performs one step, so this method needs to be called in a loop.
     *
     * The motor is moved with a position profile until either the encoder index
     * is reached or the search distance limit is exceeded.  The position is
     * controlled with a simple PD controller.
     *
     * If the encoder index is found, its position is used as home position.
     * The zero position is offset from the home position by adding the "home
     * offset" to it (i.e. zero = home pos. + home offset).
     * If the search distance limit is reached before the encoder index occurs,
     * the homing fails.
     *
     * @return Status of the homing procedure.
     */
    HomingReturnCode update_homing();

private:
    /**
     * @brief Convert from joint torque to motor current.
     * 
     * @param[in] torque is the input joint
     * @return double the equivalent motor current.
     */
    double joint_torque_to_motor_current(double torque) const;

    /**
     * @brief Convert from motor current to joint torque.
     * 
     * @param current is the motor current.
     * @return double is the equivalent joint torque.
     */
    double motor_current_to_joint_torque(double current) const;

    /**
     * @brief Get motor measurements and check if there are data or not.
     * 
     * @param measurement_id is the id of the measurement you want to get.
     * check: blmc_drivers::MotorInterface::MeasurementIndex
     * @return double the measurement.
     */
    double get_motor_measurement(const mi& measurement_id) const;

    /**
     * @brief Get the last motor measurement index for a specific data. If there
     * was no data yet, return NaN
     * 
     * @param measurement_id is the id of the measurement you want to get.
     * check: blmc_drivers::MotorInterface::MeasurementIndex
     * @return double the measurement.
     */
    long int get_motor_measurement_index(const mi& measurement_id) const;

    /**
     * @brief This is the pointer to the motor interface.
     */
    std::shared_ptr<blmc_drivers::MotorInterface> motor_;

    /**
     * @brief This is the torque constant of the motor:
     * \f$ \tau_{motor} = k * i_{motor} \f$
     */
    double motor_constant_;
    /**
     * @brief This correspond to the reduction (\f$ \beta \f$) between the motor rotation and
     * the joint. \f$ \theta_{joint} = \theta_{motor} / \beta \f$
     */
    double gear_ratio_;
    /**
     * @brief This is the distance between the closest positive index and the
     * zero configuration.
     */
    double zero_angle_;
    /**
     * @brief This change the motor rotation direction.
     */
    double polarity_;
    /**
     * @brief This is the maximum current we can apply during one experiment.
     * The program shut down if this value is achieved.
     */
    double max_current_;

    //! @brief P gain of the position PD controller.
    double position_control_gain_p_;
    //! @brief D gain of the position PD controller.
    double position_control_gain_d_;

    struct HomingState homing_state_;
};

/**
 * @brief BlmcJointModule_ptr shortcut for the shared pointer BlmcJointModule type
 */
typedef std::shared_ptr<BlmcJointModule> BlmcJointModule_ptr;

/**
 * @brief This class defines an interface to a collection of BLMC joints. It
 * creates a BLMCJointModule for every blmc_driver::MotorInterface provided.
 * 
 * @tparam COUNT 
 */
template <int COUNT>
class BlmcJointModules
{
public:
    /**
     * @brief Defines a static Eigen vector type in order to define the
     * interface.
     */
    typedef Eigen::Matrix<double, COUNT, 1> Vector;

    /**
     * @brief Construct a new BlmcJointModules object
     * 
     * @param motors 
     * @param motor_constants 
     * @param gear_ratios 
     * @param zero_angles 
     */
    BlmcJointModules(
        const std::array<std::shared_ptr<blmc_drivers::MotorInterface>, COUNT>&
          motors,
        const Vector& motor_constants,
        const Vector& gear_ratios,
        const Vector& zero_angles,
        const Vector& max_currents)
    {
        set_motor_array(motors, motor_constants, gear_ratios, zero_angles, max_currents);
    }
    /**
     * @brief Construct a new BlmcJointModules object
     */
    BlmcJointModules()
    {
    }
    /**
     * @brief Set the motor array, by creating the corresponding modules.
     * 
     * @param motors 
     * @param motor_constants 
     * @param gear_ratios 
     * @param zero_angles 
     */
    void set_motor_array(
        const std::array<std::shared_ptr<blmc_drivers::MotorInterface>, COUNT>&
          motors,
        const Vector& motor_constants,
        const Vector& gear_ratios,
        const Vector& zero_angles,
        const Vector& max_currents)
    {
        for(size_t i = 0; i < COUNT; i++)
        {
            modules_[i] = std::make_shared<BlmcJointModule>(motors[i],
                                                            motor_constants[i],
                                                            gear_ratios[i],
                                                            zero_angles[i],
                                                            false,
                                                            max_currents[i]);
        }
    }
    /**
     * @brief Send the registered torques to all modules.
     */
    void send_torques()
    {
        for(size_t i = 0; i < COUNT; i++)
        {
            modules_[i]->send_torque();
        }
    }

    /**
     * @brief Register the joint torques to be sent for all modules.
     * 
     * @param desired_torques 
     */
    void set_torques(const Vector& desired_torques)
    {
        for(size_t i = 0; i < COUNT; i++)
        {
            modules_[i]->set_torque(desired_torques(i));
        }
    }

    /**
     * @brief Get the previously sent torques.
     * 
     * @return Vector 
     */
    Vector get_sent_torques() const
    {
        Vector torques;

        for(size_t i = 0; i < COUNT; i++)
        {
            torques(i) = modules_[i]->get_sent_torque();
        }
        return torques;
    }

    /**
     * @brief Get the measured joint torques.
     * 
     * @return Vector 
     */
    Vector get_measured_torques() const
    {
        Vector torques;

        for(size_t i = 0; i < COUNT; i++)
        {
            torques(i) = modules_[i]->get_measured_torque();
        }
        return torques;
    }

    /**
     * @brief Get the measured joint angles.
     * 
     * @return Vector 
     */
    Vector get_measured_angles() const
    {
        Vector positions;

        for(size_t i = 0; i < COUNT; i++)
        {
            positions(i) = modules_[i]->get_measured_angle();
        }
        return positions;
    }

    /**
     * @brief Get the measured joint velocities.
     * 
     * @return Vector 
     */
    Vector get_measured_velocities() const
    {
        Vector velocities;

        for(size_t i = 0; i < COUNT; i++)
        {
            velocities(i) = modules_[i]->get_measured_velocity();
        }
        return velocities;
    }

    /**
     * @brief Set the zero_angles. These are the joint angles between the
     * starting pose and the zero theoretical pose of the urdf.
     * 
     * @param zero_angles 
     */
    void set_zero_angles(const Vector& zero_angles)
    {
        for(size_t i = 0; i < COUNT; i++)
        {
            modules_[i]->set_zero_angle(zero_angles(i));
        }
    }
    /**
     * @brief Get the zero_angles. These are the joint angles between the
     * starting pose and the zero theoretical pose of the urdf.
     * 
     * @return Vector 
     */
    Vector get_zero_angles() const
    {
        Vector positions;

        for(size_t i = 0; i < COUNT; i++)
        {
            positions(i) = modules_[i]->get_zero_angle();
        }
        return positions;
    }
    /**
     * @brief Get the index_angles. There is one index per motor rotation so
     * there are gear_ratio indexes per joint rotation.
     * 
     * @return Vector 
     */
    Vector get_measured_index_angles() const
    {
        Vector index_angles;

        for(size_t i = 0; i < COUNT; i++)
        {
            index_angles(i) = modules_[i]->get_measured_index_angle();
        }
        return index_angles;
    }

    /**
     * @brief Set position control gains for the specified joint.
     *
     * @param joint_id  ID of the joint (in range `[0, COUNT)`).
     * @param kp P gain.
     * @param kd D gain.
     */
    void set_position_control_gains(size_t joint_id, double kp, double kd)
    {
        modules_[joint_id]->set_position_control_gains(kp, kd);
    }

    /**
     * @brief Set same position control gains for all joints.
     *
     * @param kp P gain.
     * @param kd D gain.
     */
    void set_position_control_gains(double kp, double kd)
    {
        for(size_t i = 0; i < COUNT; i++)
        {
           set_position_control_gains(i, kp, kd);
        }
    }

    /**
     * @brief Perform homing for all joints.
     *
     * If one of the joints fails, the complete homing fails.  Otherwise it
     * loops until all joints finished.
     * If a joint is finished while others are still running, it is held at the
     * home position.
     *
     * See BlmcJointModule::update_homing for details on the homing procedure.
     *
     * See BlmcJointModule::init_homing for description of the arguments.
     *
     * @return Final status of the homing procedure (either SUCCESS if all
     *     joints succeeded or the return code of the first joint that failed).
     */
    HomingReturnCode execute_homing(double search_distance_limit_rad,
            Vector home_offset_rad,
            double profile_step_size_rad=0.001)
    {
        // Initialise homing for all joints
        for(size_t i = 0; i < COUNT; i++)
        {
            modules_[i]->init_homing((int) i, search_distance_limit_rad,
                    home_offset_rad[i], profile_step_size_rad);
        }

        // run homing for all joints until all of them are done
        real_time_tools::Spinner spinner;
        spinner.set_period(0.001);  // TODO magic number
        HomingReturnCode homing_status;
        do {
            bool all_succeeded = true;
            homing_status = HomingReturnCode::RUNNING;

            for(size_t i = 0; i < COUNT; i++)
            {
                HomingReturnCode joint_result = modules_[i]->update_homing();

                all_succeeded &= (joint_result == HomingReturnCode::SUCCEEDED);

                if (joint_result == HomingReturnCode::NOT_INITIALIZED ||
                        joint_result == HomingReturnCode::FAILED) {
                    homing_status = joint_result;
                    // abort homing
                    break;
                }
            }

            if (all_succeeded) {
                homing_status = HomingReturnCode::SUCCEEDED;
            }

            spinner.spin();
        } while (homing_status == HomingReturnCode::RUNNING);

        return homing_status;
    }




private:
    /**
     * @brief These are the BLMCJointModule objects corresponding to a robot.
     */
    std::array<std::shared_ptr<BlmcJointModule>, COUNT> modules_;
};





} // namespace blmc_robots
