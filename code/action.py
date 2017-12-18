import numpy as np


def stop(rover):
    # apply brake and set throttle to 0
    rover.throttle = 0
    rover.brake = rover.brake_set


def coast(rover):
    # release brake and throttle
    rover.throttle = 0
    rover.brake = 0


def accelerate(rover, limit):
    # set throttle to full and release brake
    rover.brake = 0
    if rover.vel >= limit:
        rover.throttle = 0
    else:
        rover.throttle = 0.4


def reverse(rover):
    # reverse rover
    rover.brake = 0
    rover.throttle = -0.4


def steer_low_speed_to_rock(rover):
    rover.steer = np.clip(np.mean(rover.rock_angles * 180 / np.pi), -15, 15)


def steer_high_speed_to_rock(rover):
    rover.steer = np.clip(np.mean(rover.rock_angles * 180 / np.pi), -6, 6)


def steer(rover):
    # set steering angle params to the mean of the available
    print('Steering by nav angles')

    steer_calculation = np.mean(rover.nav_angles * 180 / np.pi)
    if np.isnan(steer_calculation):
        print('steer calc is NAN')
        rover.steer = 0

    else:
        if rover.vel < 1.1:
            rover.steer = np.clip(steer_calculation, -15, 15)
            print('Steering angle set to ', rover.steer)
        else:
            rover.steer = np.clip(steer_calculation, -5, 8) - 3
            print('Steering angle set to ', rover.steer)


def steer_straight(rover):
    print('Steering angle is ZERO')
    rover.steer = 0


def spin(rover):
    # when stopped, apply full rotation
    print('Steering angle is SPIN (25)')
    rover.steer = 25  # Could be more clever here about which way to turn

def map_spin(rover):
    # when stopped, apply full rotation
    print('Steering angle is SPIN (25)')
    rover.steer = 25  # Could be more clever here about which way to turn
