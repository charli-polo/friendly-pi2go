# -*- coding: utf-8 -*-

from __future__ import print_function
# Import all necessary libraries
import time, RPi.GPIO as GPIO

# Robot modules
from motor import Motor
from board import Board
from led import Leds
from button import Button
from lcd7segment import Lcd7segment
from infraRed import InfraRed
from sonar import Sonar

import os
def errase_pic(module_name):
    os.system('rm ' + module_name + '.pyc')
    
def reload_modules():
    for module_name in ['rb', 'motor', 'board', 'led', 'button', 'lcd7segment']:
        errase_pic(module_name)

class Robot:
    def __init__(self):
        self._board = Board()
        self._speed = 50
        self.motor = Motor(self._speed)
        self.button = Button(self)
        self.leds = Leds()
        self.lcd = Lcd7segment()
        self.ir = InfraRed(self._board)
        self.sonar = Sonar()

        print('Robot is ready !')

    # def cleanup(self):
    #     """Reset everything (leds and stuff)"""
    #     cleanup()

    def sleep(self, seconds):
        """ shortcut to time.sleep"""
        time.sleep(seconds)
        
    def say(self, words):
        """Scroll some text on the LCD"""
        self.lcd.scroll(words, 1)

    def getDistance(self):
        """Return distance with sonar"""
        GPIO.setup(sonar, GPIO.OUT)
        # Send 10us pulse to trigger
        GPIO.output(sonar, True)
        time.sleep(0.00001)
        GPIO.output(sonar, False)
        start = time.time()
        count=time.time()
        GPIO.setup(sonar,GPIO.IN)
        while GPIO.input(sonar)==0 and time.time()-count<0.1:
            start = time.time()
        count=time.time()
        stop=count
        while GPIO.input(sonar)==1 and time.time()-count<0.1:
            stop = time.time()
        # Calculate pulse length
        elapsed = stop-start
        # Distance pulse travelled in that time is time
        # multiplied by the speed of sound 34000(cm/s) divided by 2
        distance = elapsed * 17000
        return distance

        
        
