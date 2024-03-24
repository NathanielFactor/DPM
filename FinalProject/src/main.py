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
COLOR_SENSOR = EV3ColorSensor(3) # port S2
FRONT_US = EV3UltrasonicSensor(1) # port S4
#SIDE_US = EV3UltrasonicSensor(3) # port S3

# Motor Connections and initialization
MOTOR_LEFT = BP.PORT_A
MOTOR_RIGHT = BP.PORT_B
#MOTOR_NAVIGATION = BP.PORT_C
MOTOR_LAUNCH = BP.PORT_C
MOTOR_INTAKE = BP.PORT_D

# Initialize global variables
LOADING_PHASE = False
ROTATE_NAVIGATION = False
POWER_LIMIT = 70

# Pathing global variables
FRONT_COLLISSION = False
distanceToLoading = 0
tunnelCounter = 0
RED_DETECTION = False
RW = 0.025
RB = 0.1905
DISTTODEG = 180/(3.1416*RW)
ORIENTTODEG = RB/RW

# Setup before the robot begins
print("Done initializing code")
wait_ready_sensors()

def play_sound(NOTE):
    
    """Play a single note."""
    NOTE.play()
    NOTE.wait_done()

def moveDistForward(dist):
    try:
        print("moving forward!")
        speed = 150
        BP.set_motor_limits(MOTOR_LEFT, 100, speed)  # Adjust as needed
        BP.set_motor_limits(MOTOR_RIGHT, 100, speed)  # Adjust as needed
        BP.set_motor_position_relative(MOTOR_LEFT, int(dist*DISTTODEG))
        BP.set_motor_position_relative(MOTOR_RIGHT, int(dist*DISTTODEG))
        print("Finished moving: " + str(dist))
    except IOError as error:
        print(error)


def rotateDegreesRight(angle):
    try:
        print("rotating right")
        speed = 150
        BP.set_motor_limits(MOTOR_LEFT, 100, speed)  # Adjust as needed
        BP.set_motor_limits(MOTOR_RIGHT, 100, speed)  # Adjust as needed
        BP.set_motor_position_relative(MOTOR_LEFT, int(angle*ORIENTTODEG))
        BP.set_motor_position_relative(MOTOR_RIGHT, int(-angle*ORIENTTODEG))
    except IOError as error:
        print(error)
        
def rotateDegreesLeft(angle):
    try:
        print("rotating left")
        speed = 150
        BP.set_motor_limits(MOTOR_LEFT, 100, speed)  # Adjust as needed
        BP.set_motor_limits(MOTOR_RIGHT, 100, speed)  # Adjust as needed
        BP.set_motor_position_relative(MOTOR_LEFT, int(-angle*ORIENTTODEG))
        BP.set_motor_position_relative(MOTOR_RIGHT, int(angle*ORIENTTODEG))
    except IOError as error:
        print(error)

def collisionFront():
    frontUSensor()

def initPath():
    #align the robot with the red tunnel close to the wall
    print("starting path")
    moveDistForward(1)
    
    #rotateDegreesRight(45)
    #sleep(3)
    #moveDistForward(.15)
    #sleep(3)
    #rotateDegreesLeft(45)
    print("finish test")

    
def otherTunnel():
    #rotate the robot a certain amount of cm to the left and select the other tunnel then resume drive
    rotateDegreesLeft(45)
    sleep(3)
    moveDistForward(.2)
    sleep(3)
    rotateDegreesRight(45)


def sideUSensor():
    sample_int = 0.2 #sample every 200 ms
    wall_dist = 0.2 #20 cm from wall
    deadband = 0.05 #5 cm tolerance from wall_dist
    speed = 150 #default speed
    delta_speed = 100 #default change in speed
    us_outlier = 200 #anything outside of 200 cm is ignored

    try:
        wait_ready_sensors()
        #BP.set_motor_limits(MOTOR_LEFT, 100, 150)
        #BP.set_motor_limits(MOTOR_RIGHT, 100, 150)
        #MOTOR_LEFT.reset_encoder()
        #MOTOR_RIGHT.reset_encoder()
        

        while True:
            dist = SIDE_US.get_cm()

            if dist >= us_outlier:
                dist = wall_dist

            dist = dist/100.0
            error = wall_dist - dist
            print('dist: {:0.2f}'.format(dist))
            print('error: {:0.2f}'.format(error))

            #case1: error is within deadband tolerance: no change
            if abs(error) <= deadband:
                MOTOR_LEFT.set_dps(speed)
                MOTOR_RIGHT.set_dps(speed)

            #case2: negative error, move closer to wall
            elif error < 0:
                MOTOR_LEFT.set_dps(speed)
                MOTOR_RIGHT.set_dps(speed + delta_speed)

            #case3: positive error, move further from wall
            else:
                MOTOR_LEFT.set_dps(speed + delta_speed)
                MOTOR_RIGHT.set_dps(speed)

            sleep(sample_int)

    except (KeyboardInterrupt, OSError): #program will exit when Ctrl + C
        BP.reset_all()


def frontUSensor():
    print("entering frontUSensor")
    sleep(1)
    FRONT_COLLISSION = False
    TUNNEL_COUNTER = 0
    
    while not FRONT_COLLISSION:
        distance = FRONT_US.get_cm()
        moveDistForward(1)
        print(str(distance))
        
        ##update total distance travelled
        #distance_difference = distance - distanceToLoading
        #distanceToLoading += distance_difference#need to init global variable like this to update value
        
        #tunnel detected
        if (distance < 20) and ((TUNNEL_COUNTER == 0) or (TUNNEL_COUNTER == 3)):
            FRONT_COLLISSION = True
            TUNNEL_COUNTER = 1
            otherTunnel()
            
        # First encounter with wall turn left
        elif (distance < 20) and (TUNNEL_COUNTER == 1):
            FRONT_COLLISSION = True #stop driving thread
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
            FRONT_COLLISSION = True #stop driving thread
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
    sleep(2)
    RED_DETECTION = True
    BP.set_motor_limits(MOTOR_LAUNCH, 100, 700)
    print("forwards")
    BP.set_motor_position_relative(MOTOR_LAUNCH, 90)
    sleep(2)
    print("backwards")
    BP.set_motor_position_relative(MOTOR_LAUNCH, -90)
    sleep(2)
    print("launched ball")


#Detects Red Colour
def colourDetection():
    while True:
        #COLOR_SENSOR
        red_lvl = COLOR_SENSOR.get_rgb()[0]
        if red_lvl > 30:
            print("red detected")
            RED_DETECTION = True
            
def intakeSystem():
    
    for i in range(9):
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
        
    print("intaking ball")
    BP.set_motor_dps(MOTOR_INTAKE, 240)
    sleep(0.4)
    
    BP.set_motor_dps(MOTOR_INTAKE, -245)
    sleep(0.6)
    
    BP.set_motor_dps(MOTOR_INTAKE, 240)
    sleep(0.4)
    
    print("launching")
    BP.set_motor_dps(MOTOR_INTAKE, 0)
    launch()
    sleep(2)
    
    reset_brick()


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
    run_in_backgroud(lambda: sideUSensor(), lambda: initPath()) # Both functions that we want running at the same time


def run_in_backgroud(action: FunctionType, action2: FunctionType):

    """
    Run the functions specified in the argument in different threads.
    """

    # Start the thread for sideUS based on motor encoder 
    thread1 = Thread(target=action)
    thread1.start()

    # Start the thread for Pathing on touch sensor actication
    thread2 = Thread(target=action2)
    thread2.start()
    return 
'''


if __name__ == "__main__":
    reset_brick()
    print("starting")
    sleep(2)
    print("Robot initializing...")
    #collisionFront()
    #launch()
    #begin_threading_instances()
    intakeSystem()
    
    
    #monitor_kill_switch()
              