# Motor Encoder 
#!/usr/bin/env python3

# Add your imports here, if any
from utils.brick import EV3ColorSensor, wait_ready_sensors, TouchSensor, reset_brick, Motor 
from time import sleep
from utils import sound
import brickpi3

from collections import deque
from statistics import mean
from threading import Thread
from types import FunctionType

BP = brickpi3.BrickPi3()

# Touch Sensor port connections 
DRUM_START_BUTTON = TouchSensor(2)
EMERGENCY_STOP = TouchSensor(1)
PLAY_NOTE_BUTTON = TouchSensor(3)

# Motor Connections and initialization
DRUM_ARM = BP.PORT_A
NOTE_SELECT_MOTOR = Motor("D")
POWER_LIMIT = 70


print("Done initializing code")
wait_ready_sensors()
killswitch = False
IS_DRUMMING = False


def check_KS_status():
    while True:
         if EMERGENCY_STOP.is_pressed():
             print("KILL SWItCH HIt")    
             exit()

def drum():
    IS_DRUMMING = False
    
    while True:
            
       

        if DRUM_START_BUTTON.is_pressed():
            IS_DRUMMING = True 
            
        while IS_DRUMMING:

            print("Start Button pressed. Drumming activated.")
            BP.set_motor_limits(DRUM_ARM,POWER_LIMIT,500)
            print("test2")
            BP.set_motor_position_relative(DRUM_ARM, 60)
            sleep(0.5)
            BP.set_motor_limits(DRUM_ARM,POWER_LIMIT,500)
            print("test3")
            BP.set_motor_position_relative(DRUM_ARM, -60)
            sleep(0.5)
            print("finished")
    

def collect_motor_data():

    "Collect motor encoder data."

    try:

        while True:

            # if DRUM_START_BUTTON.is_pressed():
            #     drum_on = True

            # if drum_on:
            #     drum()
            sleep(1)
            print("in main")




    except Exception as e:  # capture all exceptions including KeyboardInterrupt (Ctrl-C)
        print(e)
        pass

    finally: # When touch sensor 2 is touched, initiate closing sequence

        reset_brick() # Turn off everything on the brick's hardware, and reset it
        exit()



def determine_max_sensor_sample_rate_multithreaded():
    """
    Determine the maximum sample rate of both sensors in Hz. Do this by sampling the sensors,
    each in their own thread, at a known rate of 1 Hz (1 sample per second) and then continuously
    increasing the sample rate, by halving the sleep time, until the number of reads no longer increases.
    """
    # for sensor in (KILL_TOUCH_SENSOR, DRUM_TOUCH ):
        # the lambda here means that the entire function invocation is first passed to run_in_background() and then run
    
    run_in_background(lambda: check_KS_status())
    run_in_background(lambda: drum())


def run_in_background(action: FunctionType):
    "Use to run an action (a function) in the background."
    return Thread(target=action).start()


if __name__ == "__main__":
    # print("Determining max sensor sample rates using a single thread")
    # for sensor in (KILL_TOUCH_SENSOR, DRUM_TOUCH ):
    #     determine_max_sensor_sample_rate(sensor)
    sleep(2)
    print("Determining max sensor sample rates using multiple threads")
    determine_max_sensor_sample_rate_multithreaded()
    collect_motor_data()







if __name__ == "__main__":
    collect_motor_data()
