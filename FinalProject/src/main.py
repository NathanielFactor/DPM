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
LOADING_PHASE_BTN = TouchSensor(2)
EMERGENCY_STOP = TouchSensor(1)

# Colour and US Sensor port connections
COLOR_SENSOR = EV3ColorSensor(3) # port S2
FRONT_US = EV3UltrasonicSensor(4) # port S3
SIDE_US = EV3UltrasonicSensor(5) # port S3

# Motor Connections and initialization
MOTOR_LEFT = BP.PORT_A
MOTOR_RIGHT = BP.PORT_B
MOTOR_NAVIGATION = BP.PORT_C

# Setup before the robot begins
print("Done initializing code")
wait_ready_sensors()

# Initialize global variables
LOADING_PHASE = False
ROTATE_NAVIGATION = False
POWER_LIMIT = 70

# Pathing global variables
FRONT_COLLISSION = False
DISTANCE_TO_LOADING = 0
TUNNEL_COUNTER = 0


def play_sound(NOTE):
    
    """Play a single note."""
    NOTE.play()
    NOTE.wait_done()


def drive():
    speed = 50  # Adjust speed as needed
    BP.set_motor_limits(MOTOR_LEFT, 50)  # Adjust as needed
    BP.set_motor_limits(MOTOR_RIGHT, 50)  # Adjust as needed
    while True:
        BP.set_motor_power(MOTOR_LEFT, speed)
        BP.set_motor_power(MOTOR_RIGHT, speed)
   
    
def otherTunnel():
    #rotate the robot a certain amount of cm to the left and select the other tunnel then resume drive
    TUNNEL_COUNTER = 0
    return


def sideUSensor():
    return


def frontUSensor():
    
    while not FRONT_COLLISSION:
        distance = FRONT_US.get_cm()
        
        #update total distance travelled
        distance_difference = distance - DISTANCE_TO_LOADING
        DISTANCE_TO_LOADING += distance_difference
        
        #tunnel detected
        if (distance < 20) and (TUNNEL_COUNTER == 0):
            FRONT_COLLISSION = True
            TUNNEL_COUNTER = 1
            otherTunnel()
            
        elif (distance < 20) and (TUNNEL_COUNTER == 1):
            #rotate the robot 90 degrees with the wheels
            
            
def loadingPhase():
    """
    This is to be run in its own thread. Enter the loading phase if the global variable of 420 cm is met.
    Once met, it will stop on the tile and enter the loading phase until a button is pressed to leave the
    loading phase and continue pathing back to the launching phase.
    """

    LOADING_PHASE = False
    
    while True:
        #note this detection for distance will be in the drive() function not in here!
        if DISTANCE_TO_LOADING == 420:
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
    

def monitor_kill_switch():

    """
    Verify the status of the kill switch touch sensor. If the kill switch is activated, terminate the whole program.
    This function will be continuously ran, and is only terminated if the kill switch is pressed.
    """

    try:
        while True:
            sleep(0.01)
            print("in main")
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


if __name__ == "__main__":
    

    sleep(2)
    print("Robot initializing...")
    begin_threading_instances()
    monitor_kill_switch()