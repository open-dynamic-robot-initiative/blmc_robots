#!/usr/bin/env python3
import signal
import time
import numpy as np
import copy
import sys
from os import path

import threading
import ipdb
import progressbar

import py_finger
import py_real_finger

from blmc_robots.logger import Logger



def main():

    # Configuration
    # ========================================

    # Offset between encoder index and zero-position (in radian).
    # Set this such that the zero position is in the center between left and
    # right end stop.
    home_offset = 2.241742

    # Limit of the range in which the joint can move (i.e. should be a little
    # bit before hitting the end stop).
    position_limit = 2.7

    # Gains for the position controller
    kp = 5
    kd = 0.04


    # Number of times the motor hits the endstop in each "hit end stop" phase
    NUM_ENDSTOP_HITS = 10

    # Number of back and forth movements during each "fixed velocity" phase
    NUM_FIXED_VELOCITY_MOVEMENT_STEPS = 150


    # Number of times the complete scenario is repeated
    NUM_ITERATIONS = 20


    # ========================================


    finger_data = py_finger.OJData()
    finger_backend = py_real_finger.create_one_joint_backend("can7",
                                                             home_offset,
                                                             finger_data)
    finger = py_finger.OJFrontend(finger_data)

    n_joints = 1

    def zero_torque_ctrl(duration, logger=None, print_position=False):
        desired_torque = np.zeros(n_joints)
        step = 0

        if logger:
            t = finger.get_current_time_index()
            logger.set_time(t)

        while step < duration:
            step += 1
            t = finger.append_desired_action(desired_torque)
            if print_position:
                print("\rPosition: %10.4f" %
                      finger.get_observation(t).angle[0], end="")

            if logger:
                logger.record(finger)

    def go_to(goal_position, steps, hold, logger=None):
        desired_torque = np.zeros(n_joints)

        t = finger.append_desired_action(desired_torque)
        desired_step_position = copy.copy(finger.get_observation(t).angle)

        if logger:
            logger.set_time(t)

        stepsize = (goal_position - desired_step_position) / steps

        for step in range(steps):
            desired_step_position += stepsize
            t = finger.append_desired_action(desired_torque)
            position_error = (desired_step_position -
                              finger.get_observation(t).angle)
            desired_torque = (kp * position_error -
                              kd * finger.get_observation(t).velocity)

            if logger:
                logger.record(finger)

        for step in range(hold):
            t = finger.append_desired_action(desired_torque)
            position_error = goal_position - finger.get_observation(t).angle
            desired_torque = (kp * position_error -
                              kd * finger.get_observation(t).velocity)

            if logger:
                logger.record(finger)

    def go_to_zero(steps, hold, logger=None):
        go_to(np.zeros(n_joints), steps, hold, logger)

    def hit_endstop(desired_torque, hold=0, timeout=5000, logger=None):
        zero_velocity = 0.001
        step = 0
        t = finger.append_desired_action(desired_torque)

        if logger:
            logger.set_time(t)

        while ((np.any(np.abs(finger.get_observation(t).velocity) > zero_velocity) or
               step < 100) and step < timeout):
            t = finger.append_desired_action(desired_torque)

            if logger:
                logger.record(finger)

            step += 1

        for step in range(hold):
            t = finger.append_desired_action(desired_torque)
            finger.get_observation(t)

            if logger:
                logger.record(finger)

    def increase_torque_until_movement(stepsize, max_torque, logger=None):
        """Slowly increase torque until the motor moves."""
        zero_velocity = 0.1
        desired_torque = np.zeros(n_joints)
        step = np.ones(n_joints) * stepsize
        t = finger.append_desired_action(desired_torque)
        if logger:
            logger.set_time(t)

        for i in range(1000):
            t = finger.append_desired_action(desired_torque)
            finger.get_observation(t)
            if logger:
                logger.record(finger)

        while (np.all(np.abs(finger.get_observation(t).velocity) <
                      zero_velocity) and desired_torque <= max_torque):
            t = finger.append_desired_action(desired_torque)
            if logger:
                logger.record(finger)
            desired_torque += step

        if desired_torque > max_torque:
            return False
        else:
            print("motor started movement with torque = %f" % desired_torque)
            return True


    def test_if_moves(desired_torque, timeout, logger=None):
        for i in range(timeout):
            t = finger.append_desired_action(desired_torque)
            if logger:
                logger.record(finger)
            if np.all(finger.get_observation(t).angle > 0):
                return True
        return False


    def determine_start_torque(logger):
        desired_torque = np.zeros(n_joints)
        t = finger.append_desired_action(desired_torque)
        logger.set_time(t)

        max_torque = 0.4
        stepsize = 0.025
        for trq in np.arange(max_torque, step=stepsize):
            print("test %f Nm" % trq)
            go_to(-position_limit, 1000, 100)
            desired_torque = np.ones(n_joints) * trq
            if test_if_moves(desired_torque, 3000, logger):
                break


    def validate_position():
        """Check if measured position is correct.

        Hit the end stop from both sites to check if expected and actual
        position match.
        """
        tolerance = 0.1
        desired_torque = np.ones(n_joints) * 0.15  # 0.3

        angle = [None, None]

        for i, sign in enumerate((+1, -1)):
            hit_endstop(sign * desired_torque)
            t = finger.get_current_time_index()
            angle[i] = finger.get_observation(t).angle
            #expected = sign * position_limit
            #if np.any(np.abs(angle - expected) > tolerance):
            #    raise RuntimeError("Unexpected endstop position."
            #                       "Expected %f, actual is %f" % (expected,
            #                                                      angle))

        center = (angle[0] + angle[1]) / 2

        if np.abs(center) > tolerance:
            raise RuntimeError("Unexpected center position."
                               "Expected 0.0, actual is %f" % center)
        else:
            print("Position is okay.")


    def hard_direction_change(num_repetitions, torque, logger):
        position_limit = 0.6
        direction = +1
        desired_torque = np.ones(n_joints) * torque

        t = finger.append_desired_action(np.zeros(n_joints))
        logger.set_time(t)

        progress = progressbar.ProgressBar()
        for i in progress(range(num_repetitions)):
            step = 0
            while np.all(finger.get_observation(t).angle < position_limit):
                t = finger.append_desired_action(desired_torque)
                logger.record(finger)
                step += 1
                if step > 1000:
                    raise RuntimeError("timeout hard_direction_change")

            step = 0
            while np.all(finger.get_observation(t).angle > -position_limit):
                t = finger.append_desired_action(-desired_torque)
                logger.record(finger)
                step += 1
                if step > 1000:
                    raise RuntimeError("timeout -hard_direction_change")

        # dampen movement to not hit end stop
        go_to(-position_limit, 10, 100, logger=logger)



    if len(sys.argv) >= 2:
        log_directory = sys.argv[1]
    else:
        log_directory = "/tmp"

    def log_path(filename):
        return path.join(log_directory, filename)


    logger = Logger()

    # dump logger data in case the script is killed with SIGINT or SIGQUIT
    def signal_handler(signum, stack):
        if logger.data:
            logger.name += "_aborted"
            logger.dump()
        sys.exit(1)
        #raise RuntimeError("Killed by signal %d" % signum)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGQUIT, signal_handler)



    # rotate without end stop
    #logger.start_new_recording(log_path("move_fixed_velocity_rotate"))
    #goal_position = 60
    ## move to goal position within 2000 ms and wait there for 100 ms
    #go_to(goal_position, 20000, 100, logger)
    #logger.dump()

    #return




    finger_backend.calibrate()
    print("calibration finished")
    go_to_zero(1000, 2000)

    #zero_torque_ctrl(99999999, print_position=True)
    #return

    print("initial position validation")
    validate_position()

    go_to_zero(1000, 2000)


    #[(0, 0.0),
    #  (1, 0.18),
    #  (2, 0.36),
    #  (3, 0.54),
    #  (4, 0.72),
    #  (5, 0.8999999999999999),
    #  (6, 1.08),
    #  (7, 1.26),
    #  (8, 1.44),
    #  (9, 1.6199999999999999),
    #  (10, 1.7999999999999998)]


    # Careful, dangerous!
    #logger.start_new_recording(log_path("hit_endstop"))
    #hit_torque = np.ones(n_joints) * 1.26
    #print("Start to push")
    #hit_endstop(hit_torque, hold=100, logger=logger)
    #hit_endstop(-hit_torque, hold=100, logger=logger)
    #logger.dump()


    for iteration in range (NUM_ITERATIONS):
        print("START TEST ITERATION %d" % iteration)

        print("Determine torque to start movement.")
        logger.start_new_recording(log_path("start_torque_%d" % iteration))
        determine_start_torque(logger)
        logger.dump()

        print("Switch directions with high torque")
        #low_trq = 0.36
        low_trq = 0.2
        currents = range(5, 19)
        for current in currents:
            trq = current * (0.02 * 9)
            print("A = %d (trq = %f)" % (current, trq))
            go_to(-position_limit, 500, 10, logger)
            logger.start_new_recording(log_path("hard_switch_directions_%dA_%d"
                                                % (current, iteration)))
            hard_direction_change(2, trq, logger)

            t = finger.get_current_time_index()
            if np.any(np.abs(finger.get_observation(t).angle) >
                      position_limit):
                print("ERROR: Position limit exceeded!")
                return

            hard_direction_change(10, low_trq, logger)
            logger.dump()

            logger.start_new_recording(log_path("start_torque_after_switch_direction_%dA_%d"
                                                % (current, iteration)))
            determine_start_torque(logger)
            logger.dump()


        print("position validation after switch directions")
        validate_position()

        continue





        print("Hit the end stop...")

        trq = 1.8
        logger.start_new_recording(log_path("hit_endstop_%.3f_%d" % (trq, iteration)))
        hit_torque = np.ones(n_joints) * trq
        progress = progressbar.ProgressBar()
        for i in progress(range(NUM_ENDSTOP_HITS)):
            hit_torque *= -1
            hit_endstop(hit_torque, hold=10, logger=logger)
            #hit_endstop(hit_torque, logger=logger)
            #zero_torque_ctrl(10, logger)
        logger.dump()

        #logger.start_new_recording(log_path("hit_endstop_0.2_%d" % iteration))
        #hit_torque = np.ones(n_joints) * 0.2
        #for i in range(NUM_ENDSTOP_HITS):
        #    hit_torque *= -1
        #    hit_endstop(hit_torque, logger=logger)
        #    zero_torque_ctrl(10, logger)
        #logger.dump()

        #logger.start_new_recording(log_path("hit_endstop_0.4_%d" % iteration))
        #hit_torque = np.ones(n_joints) * 0.4
        #for i in range(NUM_ENDSTOP_HITS):
        #    hit_torque *= -1
        #    hit_endstop(hit_torque, logger=logger)
        #    zero_torque_ctrl(10, logger)
        #logger.dump()

        print("position validation after hitting")
        validate_position()


        print("Move with fixed velocity...")

        goal_position = position_limit

        logger.start_new_recording(log_path("move_fixed_velocity_2000_%d" % iteration))
        progress = progressbar.ProgressBar()
        for i in progress(range(NUM_FIXED_VELOCITY_MOVEMENT_STEPS)):
            goal_position *= -1
            # move to goal position within 2000 ms and wait there for 100 ms
            go_to(goal_position, 2000, 100, logger)
        logger.dump()

        #print("validate position")
        #validate_position()

        #logger.start_new_recording(log_path("move_fixed_velocity_1000_%d" % iteration))
        #for i in range(NUM_FIXED_VELOCITY_MOVEMENT_STEPS):
        #    goal_position *= -1
        #    go_to(goal_position, 1000, 100, logger)
        #logger.dump()

        #print("validate position")
        #validate_position()

        #logger.start_new_recording(log_path("move_fixed_velocity_500_%d" % iteration))
        #for i in range(NUM_FIXED_VELOCITY_MOVEMENT_STEPS):
        #    goal_position *= -1
        #    go_to(goal_position, 500, 100, logger)
        #logger.dump()


        print("final position validation")
        validate_position()

        go_to_zero(1000, 3000)


if __name__ == "__main__":
    main()
