#!/usr/bin/env python3

# Imports 
from utils.brick import wait_ready_sensors, TouchSensor, reset_brick, Motor, EV3ColorSensor, EV3UltrasonicSensor
from time import sleep
from utils import sound
import brickpi3
from threading import Thread
from types import FunctionType

BP = brickpi3.BrickPi3() # Initialize the brickPi

# Colour port connection
COLOR_SENSOR = EV3ColorSensor() 

# Motor Connections and initialization
LAUNCH_MOTOR = BP.PORT_C
MOTOR_INTAKE = BP.PORT_D

# Global Variables
RED_DETECTED = 0

# Setup before the robot begins
print("Done initializing code")
wait_ready_sensors()
        
# Feeding door mechanism to load launcher with one ball at a time

def intakeSystem():
    
    for i in range(5):
        print("intaking ball")
        BP.set_motor_dps(MOTOR_INTAKE, 220)
        sleep(0.3)
        
        BP.set_motor_dps(MOTOR_INTAKE, -220)
        sleep(0.6)
        
        BP.set_motor_dps(MOTOR_INTAKE, 220)
        sleep(0.3)
        
        print("launching")
        BP.set_motor_dps(MOTOR_INTAKE, 0)
        launch()
        sleep(2)


#Launch Mechanism
def launch():
    while True: 
        for i in range(10): #change 10 to however many balls are loaded
            BP.set_motor_limits(LAUNCH_MOTOR, 100, 100)
            BP.set_motor_position_relative(LAUNCH_MOTOR, 20)
            BP.set_motor_position_relative(LAUNCH_MOTOR, -20)
            print("launched ball")
            sleep(5)
            i += 1


# Function that checks the color sensor value to see if its red
def is_red():

        #COLOR_SENSOR
        color_lvl = COLOR_SENSOR.get_rgb()

        if color_lvl[0] > 30: # Red is the first value in the color array
            print("red detected")
            return True
        else:
            return False 


def main ():
    while True:
        sleep(0.2)
        if RED_DETECTED == 0:
            if (is_red()):
                RED_DETECTED += 1
                sleep(30) # wait 30 seconds for the TA to load the robot, and for the robot to leave red region

        elif RED_DETECTED == 1:
            if is_red():
                sleep(1) #underterminate amount based on robot calibration 
                launch()

            



if __name__ == "__main__":

    sleep(2)
    print("Robot initializing...")
    # colourDetection()

launch ()