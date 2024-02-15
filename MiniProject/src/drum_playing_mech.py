#!/usr/bin/env python3

"""
Module to play sounds when the touch sensor is pressed.
This file must be run on the robot.
"""

from utils.brick import Motor
import time

MOTOR = Motor(1)

def drumMotor():
    "Move the motor from 0 to 90 degrees and back continuously."
    while True:
        MOTOR.run_angle(speed=100, rotation_angle=90)  # Move to 90 degrees
        time.sleep(1)  # Wait for stability (adjust as needed)
        MOTOR.run_angle(speed=100, rotation_angle=0)  # Move back to 0 degrees
        time.sleep(1)  # Wait for stability (adjust as needed)

if __name__ == '__main__':
    drumMotor()
