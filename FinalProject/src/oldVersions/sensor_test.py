#!/usr/bin/env python3

# Imports 
from utils import brick
from utils.brick import wait_ready_sensors, TouchSensor, reset_brick, Motor, EV3ColorSensor, EV3UltrasonicSensor
from time import sleep
from utils import sound
import brickpi3
from threading import Thread
from types import FunctionType

BP = brickpi3.BrickPi3() # Initialize the brickPi

# Touch Sensor port connections 
#LOADING_PHASE_BTN = TouchSensor(1)
#EMERGENCY_STOP = TouchSensor(1)

# Colour and US Sensor port connections
SIDE_US = EV3UltrasonicSensor(1) # port S3

# MOTOR_NAVIGATION = BP.PORT_C

print("Done initializing code")
wait_ready_sensors(True)

def sideUSensor():
    while True:
        us_data = SIDE_US.get_value()
        if us_data is not None:
            print(us_data)
        sleep(1)

if __name__ == "__main__":
    reset_brick()
    print("starting")
    sleep(2)
    print("Robot initializing...")
    #collisionFront()
    #launch()
    #begin_threading_instances()
    #intakeSystem()
    sideUSensor()
    #navigationMotor()


