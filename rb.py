# -*- coding: utf-8 -*-

from __future__ import print_function
# Import all necessary libraries
import time
# import RPi.GPIO as GPIO, sys, threading, time, os, smbus
# import pca9685_p3
# from sgh_PCF8591P_p3 import sgh_PCF8591P
from motor import Motor


class Robot:
    def __init__(self):

        # self.lcd = Lcd7segment()
        # self.ir = InfraRed()
        self._speed = 50
        self.motor = Motor(self._speed)
        # self.button = Button(self)
        # self.leds = Leds()
        print('Robot is ready !')

    # def cleanup(self):
    #     """Reset everything (leds and stuff)"""
    #     cleanup()

    def sleep(self, seconds):
        """ shortcut to time.sleep"""
        time.sleep(seconds)
