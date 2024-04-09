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

def rotateNavigationMotor(degrees, sleepTime=0.5):
    BP.set_motor_limits(MOTOR_NAVIGATION, 150, degrees)
    degrees = degrees * 2
    
    print("rotating")
    BP.set_motor_dps(MOTOR_NAVIGATION, degrees)
    sleep(sleepTime)
    
    print("finished rotating")
    BP.set_motor_dps(MOTOR_NAVIGATION, 0)

def tunnelDetection():
    # sleep(2)
    rotateNavigationMotor(-145)
    left_tunnel = SIDE_US.get_value()
    print("Left Tunnel Dist: " + str(left_tunnel))
    
    sleep(1)
    rotateNavigationMotor(100)
    right_tunnel = SIDE_US.get_value()
    print("Right Tunnel Dist: " + str(right_tunnel))
    
    sleep(1)
    rotateNavigationMotor(45,0.65)

    if left_tunnel > right_tunnel:
        return 0
    else:
        return 1
def tunnelDetection2():
    # sleep(2)
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


def sideUSensorRight(wallDistance=0.3):
    # CHANGED: A refresh / distance checker that can be called. Making adjustments is the same as before
    # Call moveDistForward(0.1) at the end to move with adjustment (can be changed for more/less distance)
    " CALL UPDATES TO THE SIDE SENSOR DISTANCE "
    wall_dist = wallDistance
    deadband = 0.01 #2 cm tolerance from wall_dist
    speed = 400 #default speed
    delta_speed = 100 #default change in speed
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
        
def sideUSensorLeft(wallDistance=0.3):
    # CHANGED: A refresh / distance checker that can be called. Making adjustments is the same as before
    # Call moveDistForward(0.1) at the end to move with adjustment (can be changed for more/less distance)
    " CALL UPDATES TO THE SIDE SENSOR DISTANCE "
    wall_dist = wallDistance
    deadband = 0.01 #2 cm tolerance from wall_dist
    speed = 400 #default speed
    delta_speed = 100 #default change in speed
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
        
def action(frontCollisionCounter):
    throughtunnel = True
    passing = True
    while frontCollisionCounter < 3:
        distance = FRONT_US.get_cm()

        
        
        #detect tunnel
        if frontCollisionCounter == 0:
            moveDistForward(0.1)
            sideUSensorRight(0.3)
            print(distance)
            sleep(0.2)
            if distance < 25:
                stopDriving()
                tunnel = tunnelDetection()
                if tunnel == 0:
                    #hardcode the path into the left tunnel
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
                
        #turn left after tunnel
        elif frontCollisionCounter == 1:
            if throughtunnel:
                moveDistForward(1)
                sleep(7)
                throughtunnel = False
                
            moveDistForward(0.1)
            print("distance before turn", distance)
            sleep(0.2)
            
            if distance < 30:
                stopDriving()
                sleep(2)
                rotateDegreesLeft(85)
                frontCollisionCounter = 2
                sleep(5)
    
        elif frontCollisionCounter == 2:
            while True:
                print(" The distance after turning is", distance)
                moveDistForward(0.1)
                sideUSensorRight(0.3)
                sleep(0.2)
                distance = FRONT_US.get_cm()
                if distance < 15:
                    print("Distance: " , distance)
                    stopDriving()
                    sleep(2)
                    rotateDegreesLeft(165)
                    sleep(5)
                    frontCollisionCounter = 3
                    break
                
    loadingPhase()
        
        
def loadingPhase():
    loadingPhase = False
    while not loadingPhase:
        if FRONT_US.get_cm() < 5:
            loadingPhase = True
        sleep(0.5)
    rotateNavigationMotor(-155)
    action2(0)
    
    
def action2(frontCollisionCounter):
    throughtunnel = True
    passing = True
    while frontCollisionCounter < 3:
        distance = FRONT_US.get_cm()
        colour = colourDetection()
        
        if frontCollisionCounter == 0:
            while True:
                print(" The distance after turning is", distance)
                moveDistForward(0.1)
                sideUSensorLeft(0.3)
                sleep(0.2)
                distance = FRONT_US.get_cm()
                if distance < 25:
                    print("Distance: " , distance)
                    stopDriving()
                    sleep(2)
                    rotateDegreesRight(85)
                    sleep(5)
                    frontCollisionCounter = 1
                    break 
        
        #detect tunnel
        elif frontCollisionCounter == 1:
            tunnel = tunnelDetection2()
            if tunnel == 1:
                #hardcode the path into the left tunnel
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
            frontCollisionCounter = 2
            
        #turn left after tunnel
        elif frontCollisionCounter == 2:
            if throughtunnel:
                moveDistForward(1)
                sleep(7)
                throughtunnel = False
                
            moveDistForward(0.05)
            sideUSensorLeft(0.4)
            sleep(0.05)
            colour = colourDetection()
            
            if colour == 1:
                stopDriving()
                frontCollisionCounter = 3

        

        

def frontUSensor(counter_fr):
    """ Checks the front US sensor for collisions and moves incrementally forward 
        Parameter: counter_fr to track the number of front collisions detected
        Returns: updated counter_fr if collisions were detected
    """
    distance = FRONT_US.get_cm()
        
    #front collision counter passing through tunnel first time
    if (distance < 18): 
        # collection detected
        stopDriving()
        if (counter_fr == 0): # before the first tunnel
            # navigate the tunnel
            stopDriving()
            tunnel = tunnelDetection()
            if tunnel == 0:
                #hardcode the path into the left tunnel
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
            
#             moveDistForward(0.5) # inch into the tunnel
#             sleep(4)
            counter_fr = 1
            
            if counter_fr ==1:
                moveDistForward(0.5)
                print("out of tunnel")
                sleep(3)
                rotateDegreesLeft(85)
                counter_fr = 2
                
                
                
        
#        elif (counter_fr == 1): # after first tunnel
            # navigate the corner
#             rotateDegreesLeft(75)
#             sleep(4)
#             counter_fr = 2
        
#         elif (counter_fr == 2): # after corner
#             counter_fr = 3
            # reached the end
           
    # This next if statement is for exitting the tunnel 
#         if ((distance < 52) and counter_fr == 1): # if the robot is about to leave the tunnel
#             print("HITTTTT")
#                 
#                 # This logic works for both tunnels, but turning at the end is tighter on the way out of the
#                 # right tunnel. Basically, after the while, we just need to check which tunnel we are dealing
#                 # with, and then adjust accordingly.
# 
#             while (distance > 30): # Essentially, go straight until close to wall
#                     
#                 print("\n\nDistance at front is", distance)
#                 moveDistForward(0.15) # inch a little closer to the wall
#                 sleep(2)
#                 distance = FRONT_US.get_cm() # get distance again
#                 sleep(0.2)
#                     
#             rotateDegreesLeft(85)
#             print("I am rotating")
#             sleep(3)
#             counter_fr = 2
#             print("The distance after the first turn is", distance)
# 
#             if ((distance < 10) and counter_fr == 2): # The robot is approaching the last wall
#                 print("\n ---->>>>> hit wall <<<<<----\n")
#                 counter_fr = 3

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
        moveDistForward(0.1)
        sideUSensor(0.3)
        sleep(0.5)
            
    reset_brick()
    

#Stops Driving Motors
def stopDriving():
    BP.set_motor_limits(MOTOR_LEFT, 70, 0)
    BP.set_motor_limits(MOTOR_RIGHT, 70, 0)
    

#Detects Red Colour
def colourDetection():
    colour = COLOR_SENSOR.get_rgb()
    red_lvl, green_lvl, blue_lvl = colour[0], colour[1], colour[2]
    print(colour)
    if (90 < red_lvl < 200) and (0 < green_lvl < 60) and (0 < blue_lvl < 60):
        print("red detected")
        return 1
    else:
        return 0



if __name__ == "__main__":
    sleep(2)
    print("Robot initializing...")
    #reset_brick()
    wait_ready_sensors(True)
    #moveDistForward(0.1)
    action(frontCollisionCounter)
    #reset_brick()

