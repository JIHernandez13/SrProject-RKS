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
    def rotate(self, steps, choice):
        print(localtime())
        if choice in self.directions:
            GPIO.output(self.dir_pin, self.directions[choice])
            while steps > 0:
                print(steps)
                GPIO.output([self.dir_pin, self.step_pin], [True, False])
                sleep(self.step_delay)
                GPIO.output([self.dir_pin, self.step_pin], [False, True])
                sleep(self.step_delay)
                steps -= 1

        else:
            raise Exception('invalid direction input')
        print(localtime())


def gpio_init():
    # reference pins by physical pin number
    # avoids compatibility issues between pi board revisions
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)  # remove annoying debugger issues

    # TODO: populate this list with the pin configuration
    stepper_pins = []  # list to hold dedicated stepper pins


def init_modules():
    '''Initialize all variables and classes involved'''

    # init pixy
    pixy.init()
    pixy.set_lamp(False, False)  # upper lamp off, lower lamp on

    # init GPIO
    gpio_init()

    # init stepper motors
    a = motor()
    a.rotate(50, 'cw')
    a.rotate(100, 'ccw')
    # init variables
    print("async done")


def main(args):
    init_modules()
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main(sys.argv))
