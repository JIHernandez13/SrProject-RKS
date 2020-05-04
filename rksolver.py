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
import threading

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
DIR_PINS = [11, 15, 21, 31, 35, 39]
STEP_PINS = [13, 19, 23, 29, 33, 37]


def gpio_init():
    # reference pins by physical pin number
    # avoids compatibility issues between pi board revisions
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)  # remove annoying debugger issues


class motor:
    """ NEMA17 motor object to describe setup and functions """
    # static class vars
    directions = {'cw': 1, 'ccw': 0}
    step_delay = .001  # delay between steps
    steps_per_revolution = 200  # degrees per step 1.8 deg

    # default constructor
    def __init__(self, step_pin=40, dir_pin=5):
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.position_degrees = 0  # degrees
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.setup(self.dir_pin, GPIO.OUT)

    # NOTE: GPIO.output([list of pins], [list of state]) works to turn on
    # multiple ports with different states
    def rotate(self, degrees=90):
        ''' steps is number of times the motor toggles the step pin
            choices are 'cw' or 'ccw' as string
        '''
        steps = int((degrees*self.steps_per_revolution)/360)
        steps = abs(steps)

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
            self.position_degrees += 1.8

# TODO: create outline structure of rks module
# what should be in the module
# what functions the module should have
# getters and setters for private vars


class rks(motor):
    """ Rubik's cube solver object
    Local vars: 6 motor objects
     """
    colors = {'white': (255, 255, 255),  # FFFFFF
              'red': (255, 0, 0),  # FF0000
              'blue': (0, 0, 255),  # 0000FF
              'orange': (255, 165, 0),  # FFA500
              'green': (0, 128, 0),  # 008000
              'yellow': (255, 255, 0)  # FFFF00
              }

    def __init__(self, **kwargs):
        self.m_front = kwargs['front']
        self.m_back = kwargs['back']
        self.m_left = kwargs['left']
        self.m_right = kwargs['right']
        self.m_xAxis = kwargs['xAxis']
        self.m_yAxis = kwargs['yAxis']
        #  dict to map standard rks notation to motor and routine
        self.RKS_NOTATION = {"F": [self.m_front, 90],
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
    # TODO: Create wrapper function for concurrent motor driving
    def RKS_Solve(self, solution):
        """ takes in list of rotations required to solve the cube """
        for rotation in solution:
            motor, degrees = RKS_NOTATION[rotation]  # unpack solution
            if motor is not "up" or "down":
                motor.rotate(degrees)
            elif motor is "up":
                # open up side motors to flip cube
                self.move_axis('x', 'open')
                sleep(0.1)

                # flip cube
                # Threading first command will pass onto second intruction and
                # make these 2 lines more or less concurrent
                jobs = []
                t1 = threading.Thread(self.m_left.rotate(90))
                t2 = threading.Thread(self.m_right.rotate(-90))
                jobs.append(t1)
                jobs.append(t2)

                # flip cube
                # start turns concurrently
                for j in jobs:
                    j.start()

                # make sure tasks finishes
                for j in jobs:
                    j.join()

                sleep(.5)

                # make the rotation
                self.m_front.rotate(degrees)

                # put cube back in proper orientation
                jobs = []
                t1 = threading.Thread(self.m_left.rotate(-90))
                t2 = threading.Thread(self.m_right.rotate(90))
                jobs.append(t1)
                jobs.append(t2)

                # flip cube
                # start turns concurrently
                for j in jobs:
                    j.start()

                # make sure tasks finishes
                for j in jobs:
                    j.join()

                # close side motors
                self.move_axis('x', 'close')

            elif motor is "down":
                # open up side motors to flip cube
                self.move_axis('y', 'open')

                sleep(0.1)

                jobs = []
                t1 = threading.Thread(self.m_left.rotate(-90))
                t2 = threading.Thread(self.m_right.rotate(90))
                jobs.append(t1)
                jobs.append(t2)

                # flip cube
                # start turns concurrently
                for j in jobs:
                    j.start()

                # make sure tasks finishes
                for j in jobs:
                    j.join()

                # make the rotation
                self.m_right.rotate(degrees)

                jobs = []
                t1 = threading.Thread(self.m_left.rotate(90))
                t2 = threading.Thread(self.m_right.rotate(-90))
                jobs.append(t1)
                jobs.append(t2)

                # flip cube
                # start turns concurrently
                for j in jobs:
                    j.start()

                # make sure tasks finishes
                for j in jobs:
                    j.join()

                # close side motors
                self.move_axis('y', 'close')

    @staticmethod
    def move_axis(self, axis=None, direction=None):
        """ moves axis in or out\n
        @param1: axis to be moved\n
        @param2: open or close axis"""
        assert (direction == 'open' or direction == 'close')
        assert (axis == 'x' or axis == 'y')

        if axis == 'x':
            if direction == 'open':
                self.m_xAxis.rotate(45)
            elif direction == 'close':
                self.m_xAxis.rotate(-45)

        elif axis == 'y':
            if direction == 'open':
                self.m_yAxis.rotate(45)
            elif direction == 'close':
                self.m_yAxis.rotate(-45)
        else:
            raise ValueError("innappropriate value")

    def get_cube_state():
        """ Grabs 1 side of cube. Returns a list with RGB val in each position """
        cube_state = []
        tolerance = [10, 10, 10]
        sides = 6
        # need to compare color (w/ tolerances) to output of camera

        # for every side
        for face in range(sides):
            # for each face
            for x, y in FRAME_LOCATIONS:
                temp = pixy.video_get_RGB(x, y)
                # find color within tolerance
                for color in self.colors:
                    # red difference
                    r_diff = abs(self.colors[color][0] - temp[0])
                    g_diff = abs(self.colors[color][1] - temp[1])  # green diff
                    b_diff = abs(self.colors[color][2] - temp[2])  # blue diff

                    # if any of these fall out of tolerance with the given color pallete
                    if r_diff > tolerance:
                        break
                    elif g_diff > tolerance:
                        break
                    elif b_diff > tolerance:
                        break
                    else:
                        cube_state.append(color)
                        break

        if len(cube_state) != 54:
            print("error finding colors")
            print(*cube_state, sep='\n')

            # TODO: rotate cube to get sides
        return cube_state

    def cube_state_to_rks(cube_state):
        return cube_state

    def is_solved(cube_state):
        valid_cube = {'white': 0,  # FFFFFF
                      'red': 0,  # FF0000
                      'blue': 0,  # 0000FF
                      'orange': 0,  # FFA500
                      'green': 0,  # 008000
                      'yellow': 0  # FFFF00
                      }
        for i in cube_state:
            valid_cube[i] += 1

        for i in cube_state:
            if valid_cube[i] != 9:
                return False
        return True


def main():
    #     try:
    #         # init pixy
    #         pixy.init()
    #         pixy.set_lamp(False, False)  # upper lamp off, lower lamp on
    #     except Exception as error:
    #         print(f"{error}\npixy not connected")
    #         pass

    # init GPIO
    gpio_init()

    # init stepper motors
    motors = {'front': motor(STEP_PINS[0], DIR_PINS[0]),
              'back': motor(STEP_PINS[1], DIR_PINS[1]),
              'left': motor(STEP_PINS[2], DIR_PINS[2]),
              'right': motor(STEP_PINS[3], DIR_PINS[3]),
              'xAxis': motor(STEP_PINS[4], DIR_PINS[4]),
              'yAxis': motor(STEP_PINS[5], DIR_PINS[5])}
    d1 = motor(13, 11)
    d2 = motor(19, 15)
    d3 = motor(23, 21)
    d4 = motor(29, 31)
    while 1:
        d1.rotate(-360)
        sleep(1)
        d2.rotate(-360)
        sleep(1)
        d3.rotate(-360)
        sleep(1)
        d4.rotate(-360)
        sleep(1)

    # init rks module
    rks(**motors)
    """ Grab RGB from video """
    get_cube_state()
    """ Subroutine for grabbing each side """
    sleep(5)


# TODO: write wrapper function to map rotations to rks notation.
"""     while 1:
        # TODO: implement state machine
        sleep(.1) """


if __name__ == '__main__':
    import sys
    sys.exit((main()))
