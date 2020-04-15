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
from ctypes import *
from pixy import *
from time import sleep
import RPi.GPIO as GPIO

STEPPER_PINS = []


class motor:
    '''motor object to describe setup and functions'''
    # static class vars
    directions = {'cw': 1, 'ccw': 0}

    # default constructor
    def __init__(self, step_pin=None):
        self.step_pin = 0
        self.steps = 0
        # GPIO.

    def rotate(self, choice):
        if choice in self.directions:
            GPIO.output(self.step_pin, self.directions[choice])
        else:
            raise Exception('invalid direction input')


def gpio_init():
    # reference pins by physical pin number
    # avoids compatibility issues between pi board revisions
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)  # remove annoying debugger issues

    # TODO: populate this list with the pin configuration
    for pin in STEPPER_PINS:
        GPIO.setChannel()
        STEPPER_PINS = []  # list to hold dedicated stepper pins


def init_modules():
    '''Initialize all variables and classes involved'''

    # init pixy
    pixy.init()
    pixy.set_lamp(False, False)  # upper lamp off, lower lamp on

    # init GPIO
    gpio_init()

    # init stepper motors
    a = motor()
    b = motor(1)

    # init variables


def main(args):
    init_modules()
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main(sys.argv))
