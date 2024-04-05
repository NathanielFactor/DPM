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
#LOADING_PHASE_BTN = TouchSensor(2)
#EMERGENCY_STOP = TouchSensor(1)

# Colour and US Sensor port connections
COLOR_SENSOR = EV3ColorSensor(2) # port S2
FRONT_US = EV3UltrasonicSensor(1) # port S1
SIDE_US = EV3UltrasonicSensor(4) # port S4

# Motor Connections and initialization
MOTOR_LEFT = BP.PORT_A
MOTOR_RIGHT = BP.PORT_B
MOTOR_NAVIGATION = BP.PORT_C
#MOTOR_LAUNCH = BP.PORT_C
MOTOR_INTAKE = BP.PORT_D

# Setup before the robot begins
print("Done initializing code")
wait_ready_sensors()

# Initialize global variables
LOADING_PHASE = False
ROTATE_NAVIGATION = False
POWER_LIMIT = 70

speed = 300

# Pathing global variables
frontCollisionCounter = 0  # Number of front collisions detected

distanceToLoading = 10
RED_DETECTION = False
RW = 0.0212
RB = 0.1905
DISTTODEG = 180/(3.1416*RW)
ORIENTTODEG = RB/RW
tunnel_counter = 0
SLEEP_CONSTANT = 8.72 # Seconds/meter -- Determined experimentally 
SLEEP_CONSTANT_CHANGED = DISTTODEG / speed 

# CHANGED: Moved outside of the moveDistForward to initializing area at the top 
# so the adjustments for sideUS can update the values and use moveDistForward to move forward

BP.set_motor_limits(MOTOR_LEFT, 100, speed)  # Adjust as needed
BP.set_motor_limits(MOTOR_RIGHT, 100, speed)  # Adjust as needed

def play_sound(NOTE):
    
    """Play a single note."""
    NOTE.play()
    NOTE.wait_done()

def moveDistForward(dist):
    try:
        BP.set_motor_position_relative(MOTOR_LEFT, int(dist*DISTTODEG))
        BP.set_motor_position_relative(MOTOR_RIGHT, int(dist*DISTTODEG))
        return dist
        # sleep(dist * SLEEP_CONSTANT)
        # print("Finished moving: " + str(dist))
        
        
    except IOError as error:
        print(error)


def rotateDegreesRight(angle):
    angle = angle/2 # Divide by 2 since each motor will move at half of the angle desired
    try:
        speed = 150
        BP.set_motor_limits(MOTOR_LEFT, 80, speed)  # Adjust as needed
        BP.set_motor_limits(MOTOR_RIGHT, 80, speed)  # Adjust as needed
        BP.set_motor_position_relative(MOTOR_LEFT, int(angle*ORIENTTODEG))
        BP.set_motor_position_relative(MOTOR_RIGHT, int(-angle*ORIENTTODEG))
    except IOError as error:
        print(error)
        
def rotateDegreesLeft(angle):
    angle = angle/2 # Divide by 2 since each motor will move at half of the angle desired
    try:
        speed = 150
        BP.set_motor_limits(MOTOR_LEFT, 80, speed)  # Adjust as needed
        BP.set_motor_limits(MOTOR_RIGHT, 80, speed)  # Adjust as needed
        BP.set_motor_position_relative(MOTOR_LEFT, int(-angle*ORIENTTODEG))
        BP.set_motor_position_relative(MOTOR_RIGHT, int(angle*ORIENTTODEG))
    except IOError as error:
        print(error)


def initPath():
    #align the robot with the red tunnel close to the wall
    rotateDegreesRight(90)
    sleep(3)
    moveDistForward(.15)
    sleep(3)
    rotateDegreesLeft(90)
    print("finish test")
    

def otherTunnel():
    #rotate the robot a certain amount of cm to the left and select the other tunnel then resume drive
    print("Enter other tunnel")
    rotateDegreesLeft(90)
    sleep(3)
    moveDistForward(.2)
    sleep(3)
    rotateDegreesRight(90)

def tight_turn_left():
    # rotateDegreesLeft(25)
    # sleep(1)
    moveDistForward(0.3)
    rotateDegreesLeft(25)
    sleep(1)

def rotateNavigationMotor(degrees):
    BP.set_motor_limits(MOTOR_NAVIGATION, 100, degrees)
    
    BP.set_motor_dps(MOTOR_NAVIGATION, degrees)
    sleep(1)
    BP.set_motor_dps(MOTOR_NAVIGATION, 0)

def tunnelDetection():
    
    rotateNavigationMotor(-120)
    left_tunnel = FRONT_US.get_cm()
    rotateNavigationMotor(30)
    right_tunnel = FRONT_US.get_cm()

    if left_tunnel > right_tunnel:
        return 0
    else:
        return 1

def moving_forward():
    startingDist = FRONT_US.get_cm()
    print("Starting dist: " + str(startingDist))
    frontUSensor(0)
    sideUSensor(0.2)
    frontUSensor(0)
    sideUSensor(0.2)
    endDist = FRONT_US.get_cm()
    print("Finishing dist: " + str(endDist))
    
    print("DIFFERENCE: " + str(endDist - startingDist))
    sleep(1)

'''
MOVE WITH THE SIDE SNESOR AND CONTANTLY CHECK THE FEONT COLLISION INSIDE THAT FUNCTION

'''
def sideUSensor(wallDistance):
    # CHANGED: A refresh / distance checker that can be called. Making adjustments is the same as before
    # Call moveDistForward(0.1) at the end to move with adjustment (can be changed for more/less distance)
    " CALL UPDATES TO THE SIDE SENSOR DISTANCE "
    sample_int = 0.2 #sample every 200 ms
    wall_dist = wallDistance #20 cm from wall
    deadband = 0.025 #2.5 cm tolerance from wall_dist
    speed = 400 #default speed
    delta_speed = 100 #default change in speed
    us_outlier = 200 #anything outside of 200 cm is ignored

    try:
        print("Refreshing side US")
        dist = SIDE_US.get_value()

        if dist >= us_outlier:
            dist = wall_dist

        dist = dist/100.0
        error = wall_dist - dist
        #print('dist: {:0.2f}'.format(dist))
        #print('error: {:0.2f}'.format(error))

        #case1: error is within deadband tolerance: no change
        if abs(error) <= deadband:
            BP.set_motor_limits(MOTOR_LEFT, 100, speed)
            BP.set_motor_limits(MOTOR_RIGHT, 100, speed)

        #case2: negative error, move closer to wall
        elif error < 0:
            print("Adjust closer")
            BP.set_motor_limits(MOTOR_RIGHT, 100, speed)
            BP.set_motor_limits(MOTOR_LEFT, 100, speed + delta_speed)
            moveDistForward(0.1)

        #case3: positive error, move further from wall
        else:
            print("Adjust further")
            BP.set_motor_limits(MOTOR_RIGHT, 100, speed + delta_speed)
            BP.set_motor_limits(MOTOR_LEFT, 100, speed)
            moveDistForward(0.1)

    
    except IOError as error:
        print(error)

def frontUSensor(counter_fr):
    """ Checks the front US sensor for collisions and moves incrementally forward 
        Parameter: counter_fr to track the number of front collisions detected
        Returns: updated counter_fr if collisions were detected
    """

    print("COUNTER: " + str(counter_fr))  
    distance = FRONT_US.get_cm()

    #front collision counter passing through tunnel first time
    if (distance < 20): # collection detected
        print("Collision detected at: " + str(distance))
        if (counter_fr == 0): # before the first tunnel
            # go to other tunnel
            tunnel = tunnelDetection()
            if tunnel == 0:
                #hardcode the path into the left tunnel
                rotateDegreesLeft(90)
                moveDistForward(0.15)
                rotateDegreesRight(90)
            else:
                #hardcode the path into the right tunnel
                rotateDegreesRight(90)
                moveDistForward(0.15)
                rotateDegreesLeft(90)
            counter_fr = 1
        
        elif (counter_fr == 1): # after first tunnel
            # navigate the corner
            counter_fr = 2
        
        elif (counter_fr == 2): # after corner
            # reached the end
            counter_fr = 3
    
    #moveDistForward(0.1)
    return counter_fr



def pathingPhase():
    frontCollisionCounter = 0
    # frontCollision Counter code: 
    # 0: before tunnel, 1: past tunnel, 2: past 1st corner, 3: STOP

    # Distance values to turn know when to turn off the sideUS
    sideErrorTunnel = 0.05 # error to scan for distance from wall inside tunnel
    
    while frontCollisionCounter < 3:
        frontCollisionCounter = frontUSensor(frontCollisionCounter)
        sideUSensor(0.2) # Check the side sensor
        moveDistForward(0.1)
        sleep(0.2)
        


#Stops Driving Motors
def stopDriving():
    BP.set_motor_limits(MOTOR_LEFT, 0)
    BP.set_motor_limits(MOTOR_RIGHT, 0)


#Detects Red Colour
def colourDetection():
    while True:
        #COLOR_SENSOR
        red_lvl = COLOR_SENSOR.get_rgb()[0]
        if red_lvl > 30:
            print("red detected")
            RED_DETECTION = True



if __name__ == "__main__":
    sleep(2)
    print("Robot initializing...")
    wait_ready_sensors()
    pathingPhase()
    #moving_forward()