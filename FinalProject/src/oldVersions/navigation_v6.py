#!/usr/bin/env python3

# Imports 
from utils.brick import wait_ready_sensors, TouchSensor, reset_brick, Motor, EV3ColorSensor, EV3UltrasonicSensor
from time import sleep
from utils import sound
import brickpi3
from threading import Thread
from types import FunctionType

BP = brickpi3.BrickPi3() # Initialize the brickPi

# Colour and US Sensor port connections
COLOR_SENSOR = EV3ColorSensor(2) # port S4
FRONT_US = EV3UltrasonicSensor(1) # port S1
SIDE_US = EV3UltrasonicSensor(3) # port S3

# Motor Connections and initialization
MOTOR_LEFT = BP.PORT_A
MOTOR_RIGHT = BP.PORT_B
MOTOR_NAVIGATION = BP.PORT_C
MOTOR_INTAKE = BP.PORT_D

# Setup before the robot begins
print("Done initializing code")

# Pathing global variables
speed = 400

# Hardware Constants
RW = 0.0212 #e Estimate wheel radius
RB = 0.1905 # Estimate wheel base
DISTTODEG = 180/(3.1416*RW)
ORIENTTODEG = RB/RW
SLEEP_CONSTANT = 8.72 # Seconds/meter -- Determined experimentally 
SLEEP_CONSTANT_CHANGED = DISTTODEG / speed 

BP.set_motor_limits(MOTOR_LEFT, 100, speed)  # Adjust as needed
BP.set_motor_limits(MOTOR_RIGHT, 100, speed)  # Adjust as needed


def moveDistForward(dist): # moves wheels forward at the PRESET speed by set_motor_limits
    try:
        # set position relative to the wheel radius and radians conversion
        BP.set_motor_position_relative(MOTOR_LEFT, int(dist*DISTTODEG))
        BP.set_motor_position_relative(MOTOR_RIGHT, int(dist*DISTTODEG))
        return dist
        
    #error handling if error
    except IOError as error:
        print(error)
        

def rotateDegreesRight(angle): # rotate the robot to right in certain degrees 
    angle = angle/2 # Divide by 2 since each motor will move at half of the angle desired
    try:
        #rotate motor with repect to RB and RW
        speed = 300
        BP.set_motor_limits(MOTOR_LEFT, 100, speed)  # Adjust as needed
        BP.set_motor_limits(MOTOR_RIGHT, 100, speed)  # Adjust as needed
        BP.set_motor_position_relative(MOTOR_LEFT, int(angle*ORIENTTODEG))
        BP.set_motor_position_relative(MOTOR_RIGHT, int(-angle*ORIENTTODEG))

    except IOError as error:
        print(error)


def rotateDegreesLeft(angle): # rotate the robot to left in certain degrees
    angle = angle/2 # Divide by 2 since each motor will move at half of the angle desired
    try:
        #rotate motor with repect to RB and RW
        speed = 300
        BP.set_motor_limits(MOTOR_LEFT, 100, speed)  # Adjust as needed
        BP.set_motor_limits(MOTOR_RIGHT, 100, speed)  # Adjust as needed
        BP.set_motor_position_relative(MOTOR_LEFT, int(-angle*ORIENTTODEG))
        BP.set_motor_position_relative(MOTOR_RIGHT, int(angle*ORIENTTODEG))
    except IOError as error:
        print(error)


def initPath():
    #align the robot with the red tunnel close to the wall
    rotateDegreesRight(75)
    sleep(3)
    moveDistForward(.15)
    sleep(3)
    rotateDegreesLeft(75)
    print("finish test")
    

def rotateNavigationMotor(degrees, sleepTime=0.5):
    # the function to rotate the navigation motor
    BP.set_motor_limits(MOTOR_NAVIGATION, 150, degrees)
    degrees = degrees * 2
    
    print("rotating")
    BP.set_motor_dps(MOTOR_NAVIGATION, degrees)
    sleep(sleepTime)
    
    print("finished rotating")
    BP.set_motor_dps(MOTOR_NAVIGATION, 0)


def tunnelDetection():
    """
    ROTATES NAVIGATION MOTOR AND COMPARES DISTANCE ON BOTH SIDES
    Returns:    0 for left tunnel
                1 for right tunnel
    rotateNavigationMotor(dps, sleepTime=0.5)
    """
    #DONT CHANGE
    rotateNavigationMotor(-115)
    sleep(2)
    left_tunnel = SIDE_US.get_value()
    print("Left Tunnel Dist: " + str(left_tunnel))
    
    #DONT CHANGE
    sleep(2)
    rotateNavigationMotor(70)
    right_tunnel = SIDE_US.get_value()
    print("Right Tunnel Dist: " + str(right_tunnel))
    
    #DONT CHANGE
    sleep(1)
    rotateNavigationMotor(50)
    # rotates slightly more to return to proper position

    
    if left_tunnel > right_tunnel:
        return 0
    else:
        return 1
    
def is_sane(dist_prev, dist_cur):
    error = 0.1
    if (dist_prev - dist_cur > 0.1):
        return 0
    else:
        return 1
    

def sideUSensorRight(wall_dist=0.3, speed = 400, delta_speed = 150):
    """
    CALL UPDATES TO THE SIDE SENSOR DISTANCE WHEN FACING RIGHT 
    Parameters:
        wall_dist: desired distance from wall (default, 30cm)
        speed: speed of wheel rotation (default, global speed = 400)
            slow: 150
            fast: 500
        delta_speed: adjustments to speed to adjust for wall following (default = 150)
            should be approximately 30-40% of speed
    """
    deadband = 0.01 #1 cm tolerance from wall_dist
    us_outlier = 200 #anything outside of 200 cm is ignored

    try:
        print("\nRefreshing side US")
        dist = SIDE_US.get_cm()

        if dist >= us_outlier:
            dist = wall_dist

        dist = dist/100.0
        error = wall_dist - dist
        print('dist: {:0.2f}'.format(dist))
        print('error: {:0.2f}'.format(error))

        #case1: error is within deadband tolerance: no change
        if abs(error) <= deadband:
            BP.set_motor_limits(MOTOR_LEFT, 100, speed)
            BP.set_motor_limits(MOTOR_RIGHT, 100, speed)
            moveDistForward(0.1)

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


def sideUSensorLeft(wallDistance=0.3, speed = 400, delta_speed = 150):
    """
    CALL UPDATES TO THE SIDE SENSOR DISTANCE WHEN FACING LEFT
    Parameters:
        wall_dist: desired distance from wall (default, 30cm)
        speed: speed of wheel rotation (400 to be faster on way back)
            slow: 150
            fast: 500
        delta_speed: adjustments to speed to adjust for wall following (default = 150)
            should be approximately 30-40% of speed
    """
    wall_dist = wallDistance
    deadband = 0.01 #2 cm tolerance from wall_dist
    us_outlier = 200 #anything outside of 200 cm is ignored

    try:
        print("\nRefreshing side US")
        sleep(0.2)
        dist = SIDE_US.get_cm()

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
            moveDistForward(0.1)

        #case2: negative error, move closer to wall
        elif error < 0:
            print("Adjust closer")
            BP.set_motor_limits(MOTOR_LEFT, 100, speed)
            BP.set_motor_limits(MOTOR_RIGHT, 100, speed + delta_speed)
            moveDistForward(0.1)

        #case3: positive error, move further from wall
        else:
            print("Adjust further")
            BP.set_motor_limits(MOTOR_LEFT, 100, speed + delta_speed)
            BP.set_motor_limits(MOTOR_RIGHT, 100, speed)
            moveDistForward(0.1)

    except IOError as error:
        print(error)


def pathingPhaseOne():
    """
    PATHING FROM START TO LOADING ZONE
    frontCollisionCounter to track region along path
        0: before tunnel
        1: through and after tunnel
        2: after corner
    Returns: tunnel (0 for LEFT, 1 for RIGHT)
    Calls:
        sideUSensorRight(wall_dist=0.3, speed=400)
    """
    # set the initial value
    frontCollisionCounter = 0
    throughtunnel = True
    tunnel = 0

    # the robot will keep moving until it arrived to the loading zone
    while frontCollisionCounter < 3:
        distance = FRONT_US.get_cm()
        print("\n\n", distance, "\n\n")
        sleep(0.02)
        print("Front Collision distance: " + str(distance))

        if frontCollisionCounter == 0: # BEFORE TUNNEL
            #moveDistForward(0.1)
            sideUSensorRight(0.3)
            sleep(0.2) #I ADDED THIS 
            
            print(distance)
            if (distance < 40): # COLLISION/TUNNEL DETECTED 
                stopDriving()
                tunnel = tunnelDetection() # RETURNS 0 FOR LEFT tunnel, 1 FOR RIGHT tunnel
                if tunnel == 0:
                    # hardcode the path into the left tunnel
                    rotateDegreesLeft(60)
                    sleep(2)
                    moveDistForward(0.14)
                    sleep(2)
                    rotateDegreesRight(60)
                    sleep(2)
                    print('turning')
                else:
                    #hardcode the path into the right tunnel
                    rotateDegreesRight(70)
                    sleep(2)
                    moveDistForward(0.17)
                    sleep(2.5)
                    rotateDegreesLeft(55)
                    sleep(2)
                frontCollisionCounter = 1

        elif frontCollisionCounter == 1: # NAVIGATED TO TUNNEL
            if throughtunnel: # TRAVEL THROUGH TUNNEL FIRST
                moveDistForward(1)
                sleep(9)
                throughtunnel = False

            # MOVE FORWARD UNTIL CORNER    
            moveDistForward(0.1)
            sideUSensorRight(0.4)
            sleep(0.3)
            
            if (distance < 30): # COLLISION / CORNER WALL DETECTED
                stopDriving()
                sleep(2)
                rotateDegreesLeft(85)
                frontCollisionCounter = 2
                sleep(5)
    
        elif frontCollisionCounter == 2: # NAVIGATED PAST CORNER
            #moveDistForward(0.1)
            sideUSensorRight(0.3)
            sleep(0.3)
            distance = FRONT_US.get_cm()
            if (distance < 15): # FINAL WALL DETECTED
                stopDriving()
                sleep(2)
                # TURN AROUND TO GO BACK
                rotateDegreesLeft(165)
                sleep(5)
                frontCollisionCounter = 3

    # PHYSICAL CUE TO START LOADING (PREPARE LOADING)           
    rotateNavigationMotor(-160)
    return tunnel


def loadingPhase(): 
    """ 
    HOLD IN LOADING UNTIL PHYSICAL CUE IS DETECTED
    """
    loadingPhase = False
    while not loadingPhase: # WAIT FOR PHYSICAL CUE
        if FRONT_US.get_cm() < 5: #wave hand close to less than 5 cm infront of the Front US
            loadingPhase = True
        sleep(0.3)

    # PHYSICAL CUE TO INDICATE LOADING IS COMPLETED
    rotateNavigationMotor(170)
    rotateNavigationMotor(-170)
    sleep(1)
    
 
def pathingPhaseTwo(tunnelVal):
    """
    PATHING FROM LOADING BACK TO START
    Parameter: tunnelVal: 0 for RIGHT, 1 for LEFT (reversed from pathOne) 
    Calls:
        sideUSensorLeft(wall_dist=0.3, speed=500)
    """
    
    
    while True:
        distance = FRONT_US.get_cm()
        print(distance)
        #travel until collission with wall FRONT US
        if (distance < 15):
            
            sleep(1)
            #if originally was the far tunnel then travel backwards 30cm
            if tunnelVal == 0:
                BP.set_motor_limits(MOTOR_RIGHT, 100, 400)
                BP.set_motor_limits(MOTOR_LEFT, 100, 400)
                moveDistForward(-0.325)
                sleep(1)
            break
        
        #if no collission use side sensor to travel forward 30cm away from side wall
        sideUSensorLeft(0.3)
        sleep(0.2)
              
    # exits loop and inits final pathing phase for red detection
    sleep(2)
    rotateDegreesRight(80)
    sleep(2)
    
    #no sensor usage through the tunnel
    moveDistForward(1)
    sleep(9)
                
            
def finalPathing():
    """
    MOVE FORWARD SLOWLY FOR COLOUR SENSOR TO DETECT
    Calls: sideUSensorLeft(wall_dist=0.3, speed=500, delta_speed=150)
    """
    #move faster out of the tunnel to save time
    for i in range(5):
        sideUSensorLeft(0.4)
        sleep(0.4) # int i increments every 0.4
    
    while True:     
        moveDistForward(0.05)
        sideUSensorLeft(0.4, 150, 50) # working for 0.4, 150, 50
        sleep(0.05) # working for 0.05

    
def stopDriving(): # STOPS MOTORS FROM DRIVING
    #set driving speed limit to 0 to stop motor
    BP.set_motor_limits(MOTOR_LEFT, 70, 0)
    BP.set_motor_limits(MOTOR_RIGHT, 70, 0)
    

def colourDetection(): # RED COLOUR DETECTION
    """
    CONTINUOUS SAMPLING FOR RED COLOUR
    resets brick when detected
    """
    while True:
        #get the colour sensor rgb
        colour = COLOR_SENSOR.get_rgb()
        red_lvl, green_lvl, blue_lvl = colour[0], colour[1], colour[2]
        print(colour)
        
        #if <R,G,B> values fit estimated red colour vector then exit infinite loop
        if (50 < red_lvl < 300) and (0 < green_lvl < 70) and (0 < blue_lvl < 60):
            print("red detected")
            break
        
        sleep(0.2) # working for sampling rate of 0.2
        
    print("stop driving")
    sleep(2.2) # working for 2.2 sec sleep
    # CONTINUE DRIVING ENOUGH TO MATCH LAUNCHING ZONE
    reset_brick()
    exit()

def begin_threading_instances():
    """
    Begin different threading instances for the different parts of the code that should be running and retrieving 
    data at the same time. 
    """
    run_in_background(lambda: finalPathing()) # Both functions that we want running at the same time


def run_in_background(action: FunctionType):
    """
    Run the functions specified in the argument in different threads.
    """
    
    # Start the thread for sideUS based on motor encoder 
    thread1 = Thread(target=action)
    thread1.start()
    return 

def startThreading(action: FunctionType):
    """
    Run the functions specified in the argument in different threads.
    """
    thread1 = Thread(target=action)
    thread1.start()
    return

if __name__ == "__main__":
    print("Robot initializing...")
    wait_ready_sensors(True)
    #reset_brick() #used for reseting all encoders during testing
    
    tunnelCode = pathingPhaseOne()
    loadingPhase()
    pathingPhaseTwo(tunnelCode)
    startThreading(finalPathing)
    colourDetection() 
    while True:
        distance = FRONT_US.get_cm()
           

