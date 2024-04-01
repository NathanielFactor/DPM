#!/usr/bin/env python3

# Imports 
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
COLOR_SENSOR = EV3ColorSensor(2) # port S2
FRONT_US = EV3UltrasonicSensor(1) # port S4
SIDE_US = EV3UltrasonicSensor(4) # port S3

MOTOR_NAVIGATION = BP.PORT_C

print("Done initializing code")
wait_ready_sensors()


