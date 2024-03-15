    #!/usr/bin/env python3

"""
Module to play sounds when the touch sensor is pressed.
This file must be run on the robot.
"""

from utils.brick import TouchSensor, wait_ready_sensors, reset_brick

TOUCH_SENSOR = TouchSensor(2)

wait_ready_sensors()  # Note: Touch sensors actually have no initialization time

def killSwitch():
    "In an infinite loop, play a single note when the touch sensor is pressed."
    try:
        if TOUCH_SENSOR.is_pressed():
            reset_brick() # Turn off everything on the brick's hardware, and reset it
            exit()

    except BaseException:  # capture all exceptions including KeyboardInterrupt (Ctrl-C)
        exit()


if __name__ == '__main__':
    killSwitch()
