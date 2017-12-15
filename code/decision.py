import numpy as np

from action import stop, coast, accelerate, steer, spin


# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function

def decision_step(rover):
    print("Rover is " + rover.mode)
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if rover.nav_angles is not None:

        if rover.mode == 'forward':
            forward_mode(rover)

        elif rover.mode == 'stop':
            stop_mode(rover)

        elif rover.mode == 'prospect':
            # If in a state where want to pickup a rock send pickup command
            if rover.near_sample and rover.vel == 0 and not rover.picking_up:
                rover.send_pickup = True
            else:
                prospect_mode(rover)
    else:
        accelerate(rover)

    return rover


def forward_mode(rover):
    # Check the extent of navigable terrain
    if len(rover.rock_angles) >= 10:
        rover.mode = 'prospect'
    elif len(rover.nav_angles) >= rover.stop_forward:
        # If mode is forward, navigable terrain looks good
        # and velocity is below max, then throttle
        if rover.vel < rover.max_vel:
            accelerate(rover)
            steer(rover)
        else:  # Else coast
            coast(rover)
            steer(rover)
    # If there's a lack of navigable terrain pixels then go to 'stop' mode
    elif len(rover.nav_angles) < rover.stop_forward:
        # Set mode to "stop" and hit the brakes!
        stop(rover)
        steer(rover)
        rover.mode = 'stop'


def stop_mode(rover):
    # If we're in stop mode but still moving keep braking
    if rover.vel > 0.2:
        stop(rover)
    # If we're not moving (vel < 0.2) then do something else
    elif rover.vel <= 0.2:
        # Now we're stopped and we have vision data to see if there's a path forward
        if len(rover.nav_angles) < rover.go_forward + 500:
            coast(rover)
            spin(rover)
        # If we're stopped but see sufficient navigable terrain in front then go!
        if len(rover.nav_angles) >= rover.go_forward:
            accelerate(rover)
            rover.mode = 'forward'


def prospect_mode(rover):
    print("Entered prospecting mode. There is a sample nearby")

    if rover.near_sample:
        print("Stop rover")
        stop(rover)
    else:
        # if we lose the rock signal move along to stop mode
        if len(rover.rock_angles) >= 0:
            print("move toward sample until it is within reach")
            if rover.vel > 0.4:
                print("Coast Rover")
                coast(rover)
                steer(rover)
            elif rover.vel <= 0.2:
                print("Point Rover toward sample")
                steer(rover)
                accelerate(rover)
        else:
            print("once picked up, move along to stop mode.")
            rover.mode = 'stop'
