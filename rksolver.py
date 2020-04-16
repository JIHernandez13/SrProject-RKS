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

RKS_NOTATION = {'F': 0,
                'R': 0,
                'U': 0,
                'L': 0,
                'B': 0,
                'D': 0,
                'F\'': 0,
                'R\'': 0,
                'U\'': 0,
                'L\'': 0,
                'B\'': 0,
                'D\'': 0,
                'F2': 0,
                'R2': 0,
                'U2': 0,
                'L2': 0,
                'B2': 0,
                'D2': 0
                }


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
    # BUG: GPIO.output needs to support multiple axis and subsequently multiple IO in different directions
    # NOTE: GPIO.output([list of pins], [list of state]) works to turn on multiple ports with different states
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


class axis(motor):
    ''' 2 motor per axis '''

    def __init__(self, motor_a=None, motor_b=None):
        self.MotorA = motor_a
        self.MotorB = motor_b

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


def init_modules():
    '''Initialize all variables and classes involved'''

    # init pixy
    pixy.init()
    pixy.set_lamp(False, False)  # upper lamp off, lower lamp on

    # init GPIO
    gpio_init()

    # init variables


def main(args):
    init_modules()

    # init stepper motors
    m_front = motor()
    m_back = motor()
    m_left = motor()
    m_right = motor()
    m_xAxis = motor()
    m_yAxis = motor()

    while 1:
        sleep(.1)


if __name__ == '__main__':
    import sys
    sys.exit((main(sys.argv)))
