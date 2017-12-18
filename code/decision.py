# from action import stop, coast, accelerate, steer, spin, reverse, idle
from action import *
import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function

def decision_step(rover):
    print("Rover mode is " + rover.mode)
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if rover.nav_angles is not None:

        if rover.mode == 'mapping_spin':
            mapping_spin(rover)

        elif rover.mode == 'forward':
            forward_mode(rover)

        elif rover.mode == 'stop':
            stop_mode(rover)

        elif rover.mode == 'prospect':
            # If in a state where want to pickup a rock send pickup command
            if rover.near_sample and rover.vel == 0 and not rover.picking_up:
                rover.send_pickup = True
            else:
                prospect_mode(rover)
        elif rover.mode == 'reverse':
            print("WE ARE STUCK!!!")
            reverse_mode(rover)
    else:
        accelerate(rover)
        rover.mode == 'forward'

    return rover


def forward_mode(rover):
    # Check the extent of navigable terrain
    if len(rover.rock_angles) >= 10:
        rover.mode = 'prospect'
    elif len(rover.nav_angles) >= rover.stop_forward:
        # If mode is forward, navigable terrain looks good
        # and velocity is below max, then throttle
        set_speed(rover, 1.5)
        steer(rover)

        # After a period with no velocity, flip into reverse mode.
        if rover.vel < 0.2:
            counter_delay(rover, 153, 'reverse')

    # If there's a lack of navigable terrain pixels then go to 'stop' mode
    elif len(rover.nav_angles) < rover.stop_forward:
        # Set mode to "stop" and hit the brakes!
        print("hitting the brake 1")
        stop(rover)
        steer(rover)
        rover.mode = 'stop'

    return rover


def stop_mode(rover):
    # If we're in stop mode but still moving keep braking
    if rover.vel > 0.2:
        print("hitting the brake 2")
        stop(rover)
    # If we're not moving (vel < 0.2) then do something else
    elif rover.vel <= 0.2:
        # check for amount of terrain in view and proximity to forward drive
        if len(rover.nav_angles) < rover.go_forward:
            spin(rover)
            coast(rover)

        # If we're stopped but see sufficient navigable terrain in front then go!
        elif len(rover.nav_angles) >= rover.go_forward:
            set_speed(rover, 1.5)
            rover.mode = 'forward'

        else:
            print("Should not be here.")
            spin(rover)

    return rover


def prospect_mode(rover):
    print("Entered prospecting mode. There is a sample nearby")

    if rover.near_sample:
        print("Stop rover")
        stop(rover)
    else:
        # if we lose the rock signal move along to stop mode

        if len(rover.rock_angles) > 0:
            print("move toward sample until it is within reach - vel is ", rover.vel)
            set_speed(rover, 0.4)
            if rover.vel > 0.4:
                print("Coast Rover")
                steer_high_speed_to_rock(rover)

            elif rover.vel <= 0.4:
                print("Point Rover toward sample")
                steer_low_speed_to_rock(rover)
                counter_delay(rover, 100, 'stop')
        else:
            print("lost the rock. Setting Idle and steering.")
            set_speed(rover, 0.4)
            steer_straight(rover)
            counter_delay(rover, 350, 'reverse')

    return rover


def mapping_spin(rover):
    # use rover.yaw to determine ending rotation
    # spin 60degrees and stop for a count of 10 6 times
    print('rover total spin is ', rover.total_spin)
    if rover.vel < 0.2:
        coast(rover)
        map_spin(rover)
        if rover.first:
            rover.total_spin = 0
            rover.first = False
        else:
            absolute_difference = np.abs(rover.total_spin - rover.yaw)
            rover.total_spin += np.int(absolute_difference)

            if rover.total_spin > 360:
                rover.first = True
                rover.mode = 'forward'
            else:
                sixty = 55 < rover.total_spin < 65
                one_twenty = 115 < rover.total_spin < 125
                one_eighty = 175 < rover.total_spin < 185
                two_forty = 235 < rover.total_spin < 245
                three_hundred = 295 < rover.total_spin < 305

                if sixty or one_twenty or one_eighty or two_forty or three_hundred:
                    print('At a hex point, mapping')
                    stop(rover)
                    counter_delay(rover, 10, 'mapping_spin')
    else:
        stop(rover)


def set_speed(rover, limit):
    if rover.vel < limit:
        accelerate(rover, limit)
    else:
        coast(rover)


def reverse_mode(rover):
    reverse(rover)
    steer_straight(rover)
    counter_delay(rover, 50, 'forward')

    return rover


def counter_delay(rover, total, mode):
    rover.counter += 1
    print("Total and mode = ", total, mode, rover.counter)

    if rover.counter > total:
        rover.counter = 0
        rover.mode = mode

    return rover
