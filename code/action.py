import numpy as np


def stop(rover):
    # apply brake and set throttle to 0
    rover.throttle = 0
    rover.brake = rover.brake_set


def coast(rover):
    # release brake and throttle
    rover.throttle = 0
    rover.brake = 0


def accelerate(rover):
    # set throttle to full and release brake
    rover.brake = 0
    rover.throttle = rover.throttle_set


def steer(rover):
    # set steering angle params to the mean of the available
    if rover.mode is 'prospect':

        if rover.rock_angles.any:
            print("Prospecting - Steer rock angles are ", np.mean(rover.rock_angles * 180 / np.pi))
            print("Clipped = ", np.clip(np.mean(rover.rock_angles * 180 / np.pi), -15, 15))
            rover.steer = np.clip(np.mean(rover.rock_angles * 180 / np.pi), -15, 15)
        else:
            rover.mode = 'forward'

    else:
        rover.steer = np.clip(np.mean(rover.nav_angles * 180 / np.pi), -15, 15)


def spin(rover):
    # when stopped, apply full rotation
    rover.steer = -15  # Could be more clever here about which way to turn


def reverse(rover):
    # reverse rover
    rover.throttle = -10