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
totalDistTravelled = 10 # Total distance travelled tracker
frontCollisionCounter = 0  # Number of front collisions detected

distanceToLoading = 10
RED_DETECTION = False
RW = 0.0212
RB = 0.1905
DISTTODEG = 180/(3.1416*RW)
ORIENTTODEG = RB/RW
TUNNEL_COUNTER = 0
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
        sleep(dist * SLEEP_CONSTANT)
        print("Finished moving: " + str(dist))
        
        
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


def moving_forward():
    moveDistForward(0.30)
    sleep(1)


def sideUSensor():
    # CHANGED: A refresh / distance checker that can be called. Making adjustments is the same as before
    # Call moveDistForward(0.1) at the end to move with adjustment (can be changed for more/less distance)
    " CALL UPDATES TO THE SIDE SENSOR DISTANCE "
    sample_int = 0.2 #sample every 200 ms
    wall_dist = 0.2 #20 cm from wall
    deadband = 0.05 #5 cm tolerance from wall_dist
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
        print('dist: {:0.2f}'.format(dist))
        print('error: {:0.2f}'.format(error))

        #case1: error is within deadband tolerance: no change
        if abs(error) <= deadband:
            BP.set_motor_limits(MOTOR_RIGHT, 100, speed)
            BP.set_motor_limits(MOTOR_LEFT, 100, speed)

        #case2: negative error, move closer to wall
        elif error < 0:
            print("Adjust closer")
            BP.set_motor_limits(MOTOR_RIGHT, 100, speed)
            BP.set_motor_limits(MOTOR_LEFT, 100, speed + delta_speed)

        #case3: positive error, move further from wall
        else:
            print("Adjust further")
            BP.set_motor_limits(MOTOR_RIGHT, 100, speed + delta_speed)
            BP.set_motor_limits(MOTOR_LEFT, 100, speed)

        moveDistForward(0.1)
        BP.set_motor_limits(MOTOR_RIGHT, 100, speed)
        BP.set_motor_limits(MOTOR_LEFT, 100, speed)
    
    except IOError as error:
        print(error)


def frontUSensor(counter_fr):
    """ Checks the front US sensor for collisions and moves incrementally forward 
        Parameter: counter_fr to track the number of front collisions detected
        Returns: updated counter_fr if collisions were detected
    """

    print("refreshing front US")  
    distance = FRONT_US.get_cm()
    
    if (distance < 20) and ((counter_fr == 0) or (counter_fr == 3)):
        otherTunnel()
        return 1 # updates front collision counter
    elif (distance < 20) and (counter_fr == 1):
        return 2 # updates front collision counter
    elif (distance < 20) and (counter_fr == 2):
        # counter_fr
        return 3 
    else:
        # No collision detected, continue moving
        moveDistForward(0.05)
        return counter_fr
    '''   
    # First encounter with wall turn left
    elif (distance < 20) and (TUNNEL_COUNTER == 1):
        #rotate the robot 90 degrees with the wheels
        rotateDegreesLeft(45)
        
        # Time taken to rotate 90 degrees (adjust this based on experimentation)
        rotation_time = 3  # seconds
        # Wait for the rotation to complete
        sleep(rotation_time)
        # Stop motors
        stopDriving()
        TUNNEL_COUNTER = 2
    
    # Second encounter with wall, turn right
    elif (distance < 20) and (TUNNEL_COUNTER == 2):
        #rotate the robot 90 degrees with the wheels
        speed = 150
        BP.set_motor_limits(MOTOR_LEFT, 100, speed)  # Adjust as needed
        BP.set_motor_limits(MOTOR_RIGHT, 100, speed)  # Adjust as needed

        #rotate 90 degrees
        rotateDegreesLeft(45)
        
        # Time taken to rotate 90 degrees (adjust this based on experimentation)
        rotation_time = 3  # seconds
        # Wait for the rotation to complete
        sleep(rotation_time)
        # Stop motors
        stopDriving()
        TUNNEL_COUNTER = 3
        '''
    
# CHANGED: Added this loop to call upon refresh
# distanceToLoading is fed into the call at the start (or can be hard-coded/set in the function)
# and each call to sideUSensor and frontUSensor will moveDistForward(0.1), and this will update the distanceToLoading
# While loop runs until distanceToLoading = 0 (at Loading Zone)
def pathingPhase(totalDistTravelled):
    frontCollisionCounter = 0
    # frontCollision Counter code: 
    # 0: before tunnel, 1: past tunnel, 2: past 1st corner, 3: STOP

    # Distance values to turn know when to turn off the sideUS
    distToTunnel = 10 # distance travelled by robot before tunnel (UNDERESTIMATE)
    distPastTunnel = 20 # distance travelled by robot after clearing tunnel (OVERESTIMATE)
    sideErrorTunnel = 0.05 # error to scan for distance from wall inside tunnel
    
    while frontCollisionCounter < 3:
        if (frontCollisionCounter == 0) and (totalDistTravelled > distToTunnel):
            # Robot travelled to first tunnel area without encountering, continue forward without sideUS
            while totalDistTravelled < distPastTunnel:
                # move with only side US active
                sideUSensor()
                totalDistTravelled += 0.1
            frontCollisionCounter = 1 # cleared tunnel
        sleep(0.2)
        sideUSensor() # Check the side sensor
        TUNNEL_COUNTER = frontUSensor(TUNNEL_COUNTER) # Check front sensor
        totalDistTravelled += 0.2
    
#Enter Loading Phase Initiation           
def loadingPhase():
    """
    This is to be run in its own thread. Enter the loading phase if the global variable of 420 cm is met.
    Once met, it will stop on the tile and enter the loading phase until a button is pressed to leave the
    loading phase and continue pathing back to the launching phase.
    """

    LOADING_PHASE = False
    
    while True:
        #note this detection for distance will be in the drive() function not in here!
        if distanceToLoading == 420:
            LOADING_PHASE = True 
            
        while LOADING_PHASE: # If button pressed, motor will continuously drum
            print("entering loading phase...")
            play_sound("A")
            # kill all other threads
            
            if LOADING_PHASE_BTN.is_pressed():
                print("leaving loading phase")
                # rotate the navingation system
                BP.set_motor_limits(MOTOR_NAVIGATION,POWER_LIMIT,500)
                BP.set_motor_position_relative(MOTOR_NAVIGATION, 180)
                LOADING_PHASE = False


#Stops Driving Motors
def stopDriving():
    BP.set_motor_limits(MOTOR_LEFT, 0)
    BP.set_motor_limits(MOTOR_RIGHT, 0)


#Launch Mechanism
def launch():
    while True:
        if RED_DETECTION:
            #kill driving
            for i in range(10): #change 10 to however many balls are loaded
                BP.set_motor_limits(MOTOR_LAUNCH, 100, 100)
                BP.set_motor_position_relative(MOTOR_LAUNCH, 20)
                BP.set_motor_position_relative(MOTOR_LAUNCH, -20)
                print("launched ball")
                sleep(5)
                i += 1


#Detects Red Colour
def colourDetection():
    while True:
        #COLOR_SENSOR
        red_lvl = COLOR_SENSOR.get_rgb()[0]
        if red_lvl > 30:
            print("red detected")
            RED_DETECTION = True
            
'''
#Kill Switch Implementation
def monitor_kill_switch():
    """
    Verify the status of the kill switch touch sensor. If the kill switch is activated, terminate the whole program.
    This function will be continuously ran, and is only terminated if the kill switch is pressed.
    """
    try:
        while True:
            sleep(0.01)
            if EMERGENCY_STOP.is_pressed():
                print("KILL SWItCH HIt")    
                exit()
                
    except Exception as e:  # Calpute all exceptions and record it
        print(e) #if an exception occurs, print it
        pass
    
    finally: # When touch sensor 2 is touched, initiate closing sequence
        reset_brick() # Turn off everything on the brick's hardware, and reset it
        exit()


def begin_threading_instances():
    """
    Begin different threading instances for the different parts of the code that should be running and retrieving 
    data at the same time. 
    """

    #fix this last
    run_in_backgroud(lambda: encoder_to_sound(), lambda: drum()) # Both functions that we want running at the same time


def run_in_backgroud(action: FunctionType, action2: FunctionType):

    """
    Run the functions specified in the argument in different threads.
    """

    # Start the thread for playing notes based on motor encoder 
    thread1= Thread(target=action)
    thread1.start()

    # Start the thread for playing the drum on touch sensor actication
    thread2= Thread(target=action2)
    thread2.start()
    return 
'''

if __name__ == "__main__":

    sleep(2)
    print("Robot initializing...")
    #initPath()
    #frontUSensor()
    #pathingPhase(distanceToLoading)
    #reset_brick()
    #sideUSensor()
    tight_turn_left()
    # moving_forward()