#!/usr/bin/env python3
"""Demo script for the TriFinger robot

Moves the TriFinger robot with a hard-coded choreography for show-casing and
testing.
"""
import time
import numpy as np

import robot_interfaces
import blmc_robots


N_JOINTS = 9

def run_experimental(robot):
    """Move the legs in some hard-coded choreography."""

    def perform_step(position):
        t = robot.frontend.append_desired_action(robot.Action(position=position))
        time.sleep(1)

    deg45 = np.pi / 4
    deg90 = np.pi / 2

    finger0 = [-deg45, -deg45 - np.pi/6, -deg90 - np.pi/6]
    finger120_open = [-deg90, 0, +deg45]
    finger120_clash = [deg45, 0, -deg45/2]
    finger240_open = [-deg45, 0, -deg90]
    finger240_clash = [deg45, 0, 0]

#
#    perform_step([
#        -deg45, -deg45 - np.pi/6, -deg90 - np.pi/6,
#        -deg90, 0, +deg45,
#        -deg45, 0, -deg90,
#    ])
#
#    perform_step([
#        -deg45, -deg45 - np.pi/6, -deg90 - np.pi/6,
#        deg45, 0, -deg45,
#        -deg45, 0, -deg90,
#    ])
#
#    perform_step([
#        -deg45, -deg45 - np.pi/6, -deg90 - np.pi/6,
#        -deg90, 0, +deg45,
#        -deg45, 0, -deg90,
#    ])
#
#    perform_step([
#        -deg45, -deg45 - np.pi/6, -deg90 - np.pi/6,
#        -deg90, 0, +deg45,
#        deg45, 0, -deg45,
#    ])
#
#
#    while True:
#        perform_step([
#            -deg45, -deg45 - np.pi/6, -deg90 - np.pi/6,
#            -deg90, 0, +deg45,
#            -deg45, 0, -deg90,
#        ])
#
#        perform_step([
#            -deg45, -deg45 - np.pi/6, -deg90 - np.pi/6,
#            deg45, 0, -deg45,
#            deg45, 0, -deg45,
#        ])
#

    perform_step(finger0 + finger120_open + finger240_open)
    perform_step(finger0 + finger120_open + finger240_clash)
    perform_step(finger0 + finger120_open + finger240_open)
    perform_step(finger0 + finger120_clash + finger240_open)

    while True:
        perform_step(finger0 + finger120_open + finger240_open)
        perform_step(finger0 + finger120_clash + finger240_clash)


def run_choreography(robot):
    """Move the legs in some hard-coded choreography."""

    def perform_step(position):
        t = robot.frontend.append_desired_action(robot.Action(position=position))
        time.sleep(1)

    deg45 = np.pi / 4

    while True:
        # initial pose
        perform_step([
            0, -deg45, -deg45,
            0, -deg45, -deg45,
            0, -deg45, -deg45,
        ])

        # one finger moving to the centre
        perform_step([
            0, +deg45, +deg45,
            0, -deg45, -deg45,
            0, -deg45, -deg45,
        ])

        perform_step([
            0, -deg45, -deg45,
            0, +deg45, +deg45,
            0, -deg45, -deg45,
        ])

        perform_step([
            0, -deg45, -deg45,
            0, -deg45, -deg45,
            0, +deg45, +deg45,
        ])

        # initial pose
        perform_step([
            0, -deg45, -deg45,
            0, -deg45, -deg45,
            0, -deg45, -deg45,
        ])

        # side-wards movement
        perform_step([
            -deg45, -deg45, -deg45,
            0, -deg45, 0,
            deg45, -deg45, -deg45,
        ])

        perform_step([
            deg45, -deg45, -deg45,
            -deg45, -deg45, -deg45,
            0, -deg45, 0,
        ])

        perform_step([
            0, -deg45, 0,
            deg45, -deg45, -deg45,
            -deg45, -deg45, -deg45,
        ])


def main():
    robot = blmc_robots.Robot(robot_interfaces.trifinger,
                              blmc_robots.create_trifinger_backend,
                              "trifinger.yml")
    robot.initialize()

    # move around
    run_choreography(robot)
    #run_experimental(robot)


if __name__ == "__main__":
    main()

