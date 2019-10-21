#!/usr/bin/env python3
"""Basic demo on how to control the Finger robot.

This script illustrates how to control a robot via the Python interface.
"""
import numpy as np

import robot_interfaces
import blmc_robots


def get_random_position():
    """Generate a random position within a save range."""
    position_min = np.array([1, 0, 0])
    position_max = np.array([3, 2, 5])

    position_range = position_max - position_min

    return position_min + np.random.rand(3) * position_range


def demo_torque_commands(finger):
    """Demo for running a custom controller and sending only torque commands.
    """

    # Control gains
    kp = 5
    kd = 0

    desired_torque = np.zeros(3)
    while True:
        # Run a position controller that randomly changes the desired position
        # every 300 steps.
        # One time step corresponds to roughly 1 ms.

        desired_position = get_random_position()
        for _ in range(300):
            # Appends a torque command ("action") to the action queue.
            # Returns the time step at which the action is going to be
            # executed.
            action = robot_interfaces.finger.Action(torque=desired_torque)
            t = finger.append_desired_action(action)

            # Get observations of the time step t.  Will block and wait if t is
            # in the future.
            current_position = finger.get_observation(t).position
            current_velocity = finger.get_observation(t).velocity

            # Simple PD controller to compute desired torque for next iteration
            position_error = desired_position - current_position
            desired_torque = kp * position_error - kd * current_velocity

        # print current position from time to time
        print("Position: %s" % current_position)


def demo_position_commands(finger):
    """Demo for directly sending position commands."""

    while True:
        # Run a position controller that randomly changes the desired position
        # every 300 steps.
        # One time step corresponds to roughly 1 ms.

        desired_position = get_random_position()
        for _ in range(300):
            # Appends a torque command ("action") to the action queue.
            # Returns the time step at which the action is going to be
            # executed.
            action = robot_interfaces.finger.Action(position=desired_position)
            t = finger.append_desired_action(action)

        # print current position from time to time
        print("Position: %s" % finger.get_observation(t).position)



def main():
    # Storage for all observations, actions, etc.
    finger_data = robot_interfaces.finger.Data()

    # The backend sends actions from the data to the robot and writes
    # observations from the robot to the data.
    real_finger_backend = blmc_robots.create_real_finger_backend("can0",
                                                                 "can1",
                                                                 finger_data)

    # The frontend is used by the user to get observations and send actions
    finger = robot_interfaces.finger.Frontend(finger_data)

    # Initializes the robot (e.g. performs homing).
    real_finger_backend.initialize()


    #demo_torque_commands(finger)
    demo_position_commands(finger)


if __name__ == "__main__":
    main()
