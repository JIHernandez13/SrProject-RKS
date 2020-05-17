#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  rksolver.py
#
#  Copyright 2020  Jesus Hernandez
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#
#

from __future__ import print_function
import pixy
# from ctypes import *
from pixy import *
from time import sleep, localtime
import RPi.GPIO as GPIO
import asyncio

# rotations per instruction
""" ' indicates prime which is ccw as per std cube notation """
RKS_NOTATION = {"F": 90,
                "R": 90,
                "U": 90,
                "L": 90,
                "B": 90,
                "D": 90,
                "F'": -90,
                "R'": -90,
                "U'": -90,
                "L'": -90,
                "B'": -90,
                "D'": -90,
                "F2": 180,
                "R2": 180,
                "U2": 180,
                "L2": 180,
                "B2": 180,
                "D2": 180
                }

# temp hardcoded frame locations. Should be fairly consistent once camera is mounted.
# TODO: update these numbers with the camera mounted
FRAME_LOCATIONS = [[0, 0],
                   [0, 1],
                   [0, 2],
                   [1, 0],
                   [1, 1],
                   [1, 2],
                   [2, 0],
                   [2, 1],
                   [2, 2]]

# TODO: populate these with actual pin config
DIR_PINS = []
STEP_PINS = []


def gpio_init():
    # reference pins by physical pin number
    # avoids compatibility issues between pi board revisions
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)  # remove annoying debugger issues


class motor:
    '''motor object to describe setup and functions'''
    # static class vars
    directions = {'cw': 1, 'ccw': 0}
    step_delay = .1  # delay between steps

    # default constructor
    def __init__(self, step_pin=40, dir_pin=5):
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.steps = 0
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.setup(self.dir_pin, GPIO.OUT)

    # TODO: map steps to degrees of rotation
    # NOTE: GPIO.output([list of pins], [list of state]) works to turn on
    # multiple ports with different states
    def rotate(self, steps=10, choice='cw'):
        ''' steps is number of times the motor toggles the step pin
            choices are 'cw' or 'ccw' as string
        '''
        if choice in self.directions:
            GPIO.output(self.dir_pin, self.directions[choice])
        else:
            raise Exception('invalid direction input')

        # routine to toggle step pin
        while steps > 0:
            GPIO.output(self.step_pin, True)
            sleep(self.step_delay)
            GPIO.output(self.step_pin, False)
            sleep(self.step_delay)
            steps -= 1

# TODO: create outline structure of rks module
# what should be in the module
# what functions the module should have
# getters and setters for private vars
class rks(motor):
    def __init__(self, **kwargs):
        self.m_front = kwargs['front']
        self.m_back = kwargs['back']
        self.m_left = kwargs['left']
        self.m_right = kwargs['right']
        self.m_xAxis = kwargs['xAxis']
        self.m_yAxis = kwargs['yAxis']

    # TODO: create rotations for each item in the list returned from
    def RKS_Rotation(self, solution):
        for value in solution:
            pass
        pass

    def move_axis(self, open_close=None):
        if open_close == 'in':
            self.MotorA.rotate()
            self.MotorB.rotate()
        elif open_close == 'out':
            self.MotorA.rotate()
            self.MotorB.rotate()
        else:
            raise ValueError(
                "innappropriate value\n in or out needs to be specified")


def main():
    # init pixy
    pixy.init()
    pixy.set_lamp(False, False)  # upper lamp off, lower lamp on

    # init GPIO
    gpio_init()

    # init stepper motors
    motors = {'front': motor(STEP_PINS[0], DIR_PINS[0]),
              'back': motor(STEP_PINS[1], DIR_PINS[1]),
              'left': motor(STEP_PINS[2], DIR_PINS[2]),
              'right': motor(STEP_PINS[3], DIR_PINS[3]),
              'xAxis': motor(STEP_PINS[4], DIR_PINS[4]),
              'yAxis': motor(STEP_PINS[5], DIR_PINS[5])}

    # init rks module
    rks(**motors)
    """ Grab RGB from video """
    get_rgb()
    """ Subroutine for grabbing each side """
    sleep(5)


def get_rgb():
    cube_state = []

    for i in range(6):
        for x, y in FRAME_LOCATIONS:
            cube_state.append([pixy.video_get_RGB(x, y), x, y])

    for val in cube_state:
        print(val)


# TODO: write wrapper function to map rotations to rks notation.
"""     while 1:
        # TODO: implement state machine
        sleep(.1) """


if __name__ == '__main__':
    import sys
    sys.exit((main()))
