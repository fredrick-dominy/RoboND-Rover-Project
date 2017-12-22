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
        rover.throttle = 0.24


def reverse(rover):
    # reverse rover
    rover.brake = 0
    rover.throttle = -0.4


def master_steer(rover, type, neg_val, pos_val, adjust):

    nav_type = rover.nav_angles
    adjustment = adjust or 0

    if type is 'rock':
        nav_type = rover.rock_angles

    steer_calculation = np.mean(nav_type * 180 / np.pi)
    if np.isnan(steer_calculation):
        print('steer calc is NAN')
        rover.steer = 0

    else:
        if rover.vel < 1.0:
            rover.steer = np.clip(steer_calculation, -15, 15)
        else:
            rover.steer = np.clip(steer_calculation, neg_val, pos_val) + adjustment


def steer_straight(rover):
    rover.steer = 0


def spin(rover):
    # when stopped, apply full rotation
    rover.steer = 25  # Could be more clever here about which way to turn

def map_spin(rover):
    # when stopped, apply full rotation
    rover.steer = 25  # Could be more clever here about which way to turn
