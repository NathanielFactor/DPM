#!/usr/bin/env python3

"""
Module to play sounds when the touch sensor is pressed.
This file must be run on the robot.
"""

from utils import sound
from utils.brick import TouchSensor, wait_ready_sensors, Motor
import math

SOUND1 = sound.Sound(duration=0.7, pitch="A4", volume=100)
SOUND2 = sound.Sound(duration=0.7, pitch="B4", volume=100)
SOUND3 = sound.Sound(duration=0.7, pitch="C4", volume=100)
SOUND4 = sound.Sound(duration=0.7, pitch="D4", volume=100)
TOUCH_SENSOR = TouchSensor(1)
MOTOR = Motor(1)
DEGREES = 0

wait_ready_sensors()  # Note: Touch sensors actually have no initialization time


def play_sound(note):
    "Play a single note."
    if note == 'A':
        SOUND1.play()
        SOUND1.wait_done()
    elif note == 'B':
        SOUND2.play()
        SOUND2.wait_done()
    elif note == 'C':
        SOUND3.play()
        SOUND3.wait_done()
    elif note == 'D':
        SOUND4.play()
        SOUND4.wait_done()


def play_sound_on_button_press():
    "In an infinite loop, play a single note when the touch sensor is pressed."
    global DEGREES  # accessing global variable DEGREES
    try:
        while True:
            # Get degrees from motor encoder
            DEGREES = MOTOR.get_degrees()

            # Normalize degrees to range 0-359
            DEGREES %= 360

            if TOUCH_SENSOR.is_pressed():
                if DEGREES < 90:
                    play_sound('A')
                elif DEGREES < 180:
                    play_sound('B')
                elif DEGREES < 270:
                    play_sound('C')
                else:
                    play_sound('D')

    except BaseException:  # capture all exceptions including KeyboardInterrupt (Ctrl-C)
        exit()


if __name__ == '__main__':
    play_sound_on_button_press()
