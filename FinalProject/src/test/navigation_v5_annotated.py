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
COLOR_SENSOR = EV3ColorSensor(4) # port S2
FRONT_US = EV3UltrasonicSensor(1) # port S1
SIDE_US = EV3UltrasonicSensor(3) # port S3

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

speed = 400

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

def moveDistForward(dist): # moves wheels forward at the PRESET speed by set_motor_limits
    try:
        BP.set_motor_position_relative(MOTOR_LEFT, int(dist*DISTTODEG))
        BP.set_motor_position_relative(MOTOR_RIGHT, int(dist*DISTTODEG))
        return dist
        
    except IOError as error:
        print(error)


def rotateDegreesRight(angle):
    angle = angle/2 # Divide by 2 since each motor will move at half of the angle desired
    try:
        speed = 300
        BP.set_motor_limits(MOTOR_LEFT, 100, speed)  # Adjust as needed
        BP.set_motor_limits(MOTOR_RIGHT, 100, speed)  # Adjust as needed
        BP.set_motor_position_relative(MOTOR_LEFT, int(angle*ORIENTTODEG))
        BP.set_motor_position_relative(MOTOR_RIGHT, int(-angle*ORIENTTODEG))

    except IOError as error:
        print(error)


def rotateDegreesLeft(angle):
    angle = angle/2 # Divide by 2 since each motor will move at half of the angle desired
    try:
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
    rotateNavigationMotor(-145)
    left_tunnel = SIDE_US.get_value()
    print("Left Tunnel Dist: " + str(left_tunnel))
    
    sleep(1)
    rotateNavigationMotor(100)
    right_tunnel = SIDE_US.get_value()
    print("Right Tunnel Dist: " + str(right_tunnel))
    
    sleep(1)
    rotateNavigationMotor(45,0.65)
    # rotates slightly more to return to proper position

    if left_tunnel > right_tunnel:
        return 0
    else:
        return 1
    

def tunnelDetection2():
    
    rotateNavigationMotor(+135)
    left_tunnel = SIDE_US.get_value()
    print("Left Tunnel Dist: " + str(left_tunnel))
    
    sleep(1)
    rotateNavigationMotor(-90)
    right_tunnel = SIDE_US.get_value()
    print("Right Tunnel Dist: " + str(right_tunnel))
    
    sleep(1)
    rotateNavigationMotor(-35,0.65)

    if left_tunnel > right_tunnel:
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


def sideUSensorLeft(wallDistance=0.3, speed = 500, delta_speed = 150):
    """
    CALL UPDATES TO THE SIDE SENSOR DISTANCE WHEN FACING LEFT
    Parameters:
        wall_dist: desired distance from wall (default, 30cm)
        speed: speed of wheel rotation (500 to be faster on way back)
            slow: 150
            global: 400
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


def pathingPhaseOne(frontCollisionCounter):
    """
    PATHING FROM START TO LOADING ZONE
    Parameter: frontCollisionCounter to track region along path
        0: before tunnel
        1: through and after tunnel
        2: after corner
    Calls:
        sideUSensorRight(wall_dist=0.3, speed=400)
    """
    throughtunnel = True
    while frontCollisionCounter < 3:
        distance = FRONT_US.get_cm()
        print("Front Collision distance: " + str(distance))

        if frontCollisionCounter == 0: # BEFORE TUNNEL
            moveDistForward(0.1)
            sideUSensorRight(0.3, 350)
            sleep(0.2)

            if distance < 30: # COLLISION/TUNNEL DETECTED 
                stopDriving()
                tunnel = tunnelDetection() # RETURNS 0 FOR LEFT, 1 FOR RIGHT
                if tunnel == 0:
                    # hardcode the path into the left tunnel
                    rotateDegreesLeft(75)
                    sleep(4)
                    moveDistForward(0.15)
                    sleep(4)
                    rotateDegreesRight(73)
                    sleep(4)
                    print('turning')
                else:
                    #hardcode the path into the right tunnel
                    rotateDegreesRight(75)
                    sleep(4)
                    moveDistForward(0.15)
                    sleep(4)
                    rotateDegreesLeft(75)
                    sleep(3)
                frontCollisionCounter = 1
                
        elif frontCollisionCounter == 1: # NAVIGATED TO TUNNEL
            if throughtunnel: # TRAVEL THROUGH TUNNEL FIRST
                moveDistForward(1)
                sleep(7)
                throughtunnel = False

            # MOVE FORWARD UNTIL CORNER    
            moveDistForward(0.1)
            sleep(0.2)
            # print("distance before turn", distance)
            
            if distance < 30: # COLLISION / CORNER WALL DETECTED
                stopDriving()
                sleep(2)
                rotateDegreesLeft(85)
                frontCollisionCounter = 2
                sleep(5)
    
        elif frontCollisionCounter == 2: # NAVIGATED PAST CORNER
            while True:
                # print(" The distance after turning is", distance)
                moveDistForward(0.1)
                sideUSensorRight(0.3)
                sleep(0.2)
                distance = FRONT_US.get_cm()
                if distance < 15: # FINAL WALL DETECTED
                    # print("Distance: " , distance)
                    stopDriving()
                    sleep(2)
                    # TURN AROUND TO GO BACK
                    rotateDegreesLeft(165)
                    sleep(5)
                    frontCollisionCounter = 3
                    break

    # PHYSICAL CUE TO START LOADING (PREPARE LOADING)           
    rotateNavigationMotor(-175)   


def loadingPhase(): 
    """ 
    HOLD IN LOADING UNTIL PHYSICAL CUE IS DETECTED
    """
    loadingPhase = False
    while not loadingPhase: # WAIT FOR PHYSICAL CUE
        if FRONT_US.get_cm() < 5:
            loadingPhase = True
        sleep(0.5)

    # PHYSICAL CUE TO INDICATE LOADING IS COMPLETED
    rotateNavigationMotor(175)
    rotateNavigationMotor(-175)
    sleep(1)
    
    
def pathingPhaseTwo(frontCollisionCounter):
    """
    PATHING FROM LOADING BACK TO START
    Parameter: frontCollisionCounter to track region along path
        0: before corner
        1: tunnel detection
        2: tunnel and to the end
    Calls:
        sideUSensorLeft(wall_dist=0.3, speed=500)
    """
    throughtunnel = False
    while frontCollisionCounter < 2:
        distance = FRONT_US.get_cm()
        
        if frontCollisionCounter == 0: # FROM LOADING TO CORNER
            while True:
                print(" The distance after turning is", distance)
                moveDistForward(0.1)
                sideUSensorLeft(0.3)
                sleep(0.2)
                distance = FRONT_US.get_cm()
                if distance < 20: # COLLISION DETECTED (CORNER)
                    print("Distance: " , distance)
                    stopDriving()
                    sleep(2)
                    rotateDegreesRight(85)
                    sleep(5)
                    frontCollisionCounter = 1
                    break 
        
        elif frontCollisionCounter == 1: # TUNNEL DETECTION
            tunnel = tunnelDetection2() # RETURNS 1 FOR LEFT, 0 FOR RIGHT
            if tunnel == 1:
                # hardcode the path into the left tunnel
                rotateDegreesLeft(75)
                sleep(4)
                moveDistForward(0.15)
                sleep(4)
                rotateDegreesRight(73)
                sleep(4)
                print('turning')
                throughtunnel = True
            else:
                # hardcode the path into the right tunnel
                rotateDegreesRight(75)
                sleep(4)
                moveDistForward(0.15)
                sleep(4)
                rotateDegreesLeft(75)
                sleep(3)
                throughtunnel = True
            frontCollisionCounter = 2
            
    # exits loop and inits final pathing phase for red detection
    if throughtunnel:
                moveDistForward(1)
                sleep(7)
                throughtunnel = False
                
            
def finalPathing():
    """
    MOVE FORWARD SLOWLY FOR COLOUR SENSOR TO DETECT
    Calls: sideUSensorLeft(wall_dist=0.3, speed=500, delta_speed=150)
    """
    while True:     
        moveDistForward(0.05)
        sideUSensorLeft(0.4, 150, 50) # working for 0.4, 150, 50
        sleep(0.05) # working for 0.05

    
def stopDriving(): # STOPS MOTORS FROM DRIVING
    BP.set_motor_limits(MOTOR_LEFT, 70, 0)
    BP.set_motor_limits(MOTOR_RIGHT, 70, 0)
    

def colourDetection(): # RED COLOUR DETECTION
    """
    CONTINUOUS SAMPLING FOR RED COLOUR
    resets brick when detected
    """
    while True:
        colour = COLOR_SENSOR.get_rgb()
        red_lvl, green_lvl, blue_lvl = colour[0], colour[1], colour[2]
        print(colour)
        
        if (50 < red_lvl < 300) and (0 < green_lvl < 70) and (0 < blue_lvl < 60):
            print("red detected")
            break
        
        sleep(0.2) # working for sampling rate of 0.2
        
    print("stop driving")
    sleep(2) # working for 2 sec sleep
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


if __name__ == "__main__":
    sleep(2)
    print("Robot initializing...")
    #reset_brick()
    wait_ready_sensors(True)
    #moveDistForward(0.1)
    pathingPhaseOne(frontCollisionCounter)
    loadingPhase()
    pathingPhaseTwo(0)
    begin_threading_instances()
    colourDetection()
    #reset_brick()


