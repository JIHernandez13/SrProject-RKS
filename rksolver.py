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
    """ NEMA17 motor object to describe setup and functions """
    # static class vars
    directions = {'cw': 1, 'ccw': 0}
    step_delay = .1  # delay between steps
    steps_per_revolution = 200  # degrees per step 1.8 deg

    # default constructor
    def __init__(self, step_pin=40, dir_pin=5):
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.steps = 0
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.setup(self.dir_pin, GPIO.OUT)

    # NOTE: GPIO.output([list of pins], [list of state]) works to turn on
    # multiple ports with different states
    # TODO: refactor to take negative degrees
    def rotate(self, degrees=90):
        ''' steps is number of times the motor toggles the step pin
            choices are 'cw' or 'ccw' as string
        '''
        steps = degrees/200
        assert choice in self.directions

        if degrees > 0:
            GPIO.output(self.dir_pin, self.directions['cw'])
        elif degrees < 0:
            GPIO.output(self.dir_pin, self.directions['ccw'])

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
    """ Rubik's cube solver object
    Local vars: 6 motor objects
     """

    def __init__(self, **kwargs):
        self.m_front = kwargs['front']
        self.m_back = kwargs['back']
        self.m_left = kwargs['left']
        self.m_right = kwargs['right']
        self.m_xAxis = kwargs['xAxis']
        self.m_yAxis = kwargs['yAxis']
        # rotations per instruction
        """ indicates prime which is ccw as per std cube notation """
        RKS_NOTATION = {"F": [self.m_front, 90],
                        "R": [self.m_right, 90],
                        "U": ["up", 90],
                        "L": [self.m_left, 90],
                        "B": [self.m_back, 90],
                        "D": ["down", 90],
                        "F'": [self.m_front, -90],
                        "R'": [self.m_right, -90],
                        "U'": ["up", -90],
                        "L'": [self.m_left, -90],
                        "B'": [self.m_back, -90],
                        "D'": ["down", -90],
                        "F2": [self.m_front, 180],
                        "R2": [self.m_right, 180],
                        "U2": ["up", 180],
                        "L2": [self.m_left, 180],
                        "B2": [self.m_back, 180],
                        "D2": ["down", 180]}

    # TODO: create rotations for each item in the list returned from
    # TODO: create solution for up and down row rotations
    def RKS_Rotation(self, solution):
        """ takes in list of rotations required to solve the cube """
        for rotation in solution:
            motor, degrees = RKS_NOTATION[rotation]  # unpack solution
            if motor is not "up" or "down":
                motor.rotate(degrees)
            elif motor is "up":
                # open up side motors to flip cube
                self.m_xAxis.rotate(45)
                sleep(0.1)

                # flip cube
                # TODO: make these tasks concurrent
                self.m_left.rotate(-90)
                self.m_right.rotate(90)

                # make the rotation
                self.m_front.rotate(degrees)

                # put cube back in proper orientation
                self.m_left.rotate(90)
                self.m_right.rotate(-90)

                # close side motors
                self.m_xAxis.rotate(-45)

            elif motor is "down":
                # open up side motors to flip cube
                self.m_yAxis.rotate(45)
                sleep(0.1)

                # flip cube
                # TODO: make these tasks concurrent
                self.m_front.rotate(-90)
                self.m_back.rotate(90)

                # make the rotation
                self.m_right.rotate(degrees)

                # put cube back in proper orientation
                self.m_front.rotate(90)
                self.m_back.rotate(-90)

                # close side motors
                self.m_yAxis.rotate(-45)

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
    """ Grabs 1 side of cube. Returns a list with RGB val in each position """
    cube_state = []

    for i in range(6):
        for x, y in FRAME_LOCATIONS:
            cube_state.append([pixy.video_get_RGB(x, y)])

    for val in cube_state:
        print(val)
    return cube_state


# TODO: write wrapper function to map rotations to rks notation.
"""     while 1:
        # TODO: implement state machine
        sleep(.1) """


if __name__ == '__main__':
    import sys
    sys.exit((main()))
