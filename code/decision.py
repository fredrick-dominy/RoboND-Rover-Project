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

        elif rover.mode == 'stop_and_pause':
            stop_and_pause_mode(rover)

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
        print('Mode fell through to default, forward. (Nav_angles was NaN)')
        rover.mode = 'forward'

    return rover


def forward_mode(rover):
    total_angles = len(rover.nav_angles)
    total_rock_angles = len(rover.rock_angles)

    # Check the extent of navigable terrain
    if total_rock_angles >= 10:
        rover.mode = 'prospect'
    elif total_angles >= rover.stop_forward:
        # If mode is forward, navigable terrain looks good
        # and velocity is below max, then throttle
        set_speed(rover, 1.5)
        master_steer(rover, 'nav', -12, 12, -6)

        print(total_angles)

        # After a period with no velocity, flip into reverse mode.
        if rover.vel < 0.2:
            counter_delay(rover, 150, 'reverse')

    # If there's a lack of navigable terrain pixels then go to 'stop' mode
    elif total_angles < rover.stop_forward:
        # Set mode to "stop" and hit the brakes!
        print("hitting the brake 1")

        stop(rover)
        master_steer(rover, 'nav', -5, 8, -3)
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
            if rover.vel > 0.5:
                print("Coast Rover")
                master_steer(rover, 'rock', -8, 8, 0)

            elif rover.vel <= 0.5:
                print("Point Rover toward sample")
                master_steer(rover, 'rock', -15, 15, 0)
                counter_delay(rover, 100, 'stop')
        else:
            print("lost the rock. Setting Idle and steering.")
            set_speed(rover, 0.4)
            steer_straight(rover)
            counter_delay(rover, 100, 'forward')

    return rover


def mapping_spin(rover):
    # use rover.yaw to determine ending rotation
    # spin 60degrees and stop for a count of 10 6 times
    if rover.vel < 0.2:
        # set_speed(rover, 0.15)
        map_spin(rover)

        print('Rover.yaw is...', rover.yaw)
        print('Rover.total_spin is ', rover.total_spin)
        print('Rover.yaw_differential is ', rover.yaw_differential)
        print('Rover.last_delta_rotation is ', rover.last_delta_rotation)

        if rover.first:
            # First time this is called, set some initial variables.
            rover.total_spin = 0
            rover.yaw_differential = rover.yaw
            rover.first = False
            rover.last_delta_rotation = 0
        elif rover.total_spin >= 350:
            # When counter is
            rover.first = True
            rover.mode = 'forward'
        else:
            set_total_spin(rover)
            rover.last_delta_rotation = rover.total_spin

            # Setting points to pause and map
            first = 42 < rover.total_spin < 48
            second = 132 < rover.total_spin < 138
            third = 212 < rover.total_spin < 218
            fourth = 302 < rover.total_spin < 308

            if rover.spin_map_performed:
                coast(rover)
                map_spin(rover)
                rover.spin_map_performed = False
            else:
                if first or second or third or fourth:
                    rover.spin_map_performed = True
                    rover.mode = 'stop_and_pause'
                else:
                    coast(rover)
                    map_spin(rover)
    else:
        stop(rover)


def set_total_spin(rover):
    # if differential is greater than the yaw then we've spun past 0
    if rover.yaw < rover.yaw_differential:
        print("spun past zero!")
        absolute_spin_calc = np.abs(360 + rover.yaw - rover.yaw_differential)
    else:
        absolute_spin_calc = np.abs(rover.yaw - rover.yaw_differential)

    # difference in yaw of last frame and current frame.
    spin_increment = (np.int(absolute_spin_calc) - rover.last_delta_rotation)
    rover.total_spin += spin_increment


def set_speed(rover, limit):
    if rover.vel < limit:
        accelerate(rover, limit)
    else:
        coast(rover)


def stop_and_pause_mode(rover):
    set_speed(rover, 0.1)
    # stop(rover)
    counter_delay(rover, 20, 'mapping_spin')


def reverse_mode(rover):
    reverse(rover)
    steer_straight(rover)
    counter_delay(rover, 100, 'forward')

    return rover


def counter_delay(rover, total, mode):
    rover.counter += 1
    print("Total and mode = ", total, mode, rover.counter)

    if rover.counter > total:
        rover.counter = 0
        rover.mode = mode

    return rover
