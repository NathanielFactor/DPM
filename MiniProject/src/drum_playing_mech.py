#!/usr/bin/env python3

"""
Module to play sounds when the touch sensor is pressed.
This file must be run on the robot.
"""

from utils.brick import EV3UltrasonicSensor, Touchsensor, Motor, wait_ready_sensors, reset_brick
import brickpi3
import time

START_BUTTON = Touchsensor(2)
EMERGENCY_STOP = Touchsensor(1)
BP = brickpi3.BrickPi3()
DRUM_ARM = BP.PORT_A
POWER = 70

wait_ready_sensors()
print("Done initalizing code")

def killSwitch():
    try:
        if EMERGENCY_STOP.is_pressed():
            reset_brick()
            exit()
    except BaseException:
        exit()
    
def drum():
    print("waiting for start")   
    while True:
        if START_BUTTON.is_pressed():
            time.sleep(1)
            killSwitch = False
            while not START_BUTTON.is_pressed():
                while not killSwitch:
                    BP.set_motor_limits(DRUM_ARM, POWER, 500)
                    BP.set_motor_position_relative(DRUM_ARM, 90)
                    time.sleep(1)
                    BP.set_motor_limits(DRUM_ARM, POWER, 500)
                    BP.set_motor_position_relative(DRUM_ARM, -90)
                    time.sleep(1)
                    
                    if EMERGENCY_STOP.is_pressed():
                        killSwitch = True
                break
        
if __name__ == '__main__':
    drum()


def ienfain():
    function = [drums,kisswitch]
    for function in process:
        something(lambda: drum(), killSwitch())