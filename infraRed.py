# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO


class InfraRed(object):
    def __init__(self, board):
        if board.PGType == board.PGFull:
            self._frontChannel = 13     # Middle obstacle sensor - not available on Lite version
            self._leftChannel = 11      # Front Left obstacle sensor
            self._rightChannel = 7      # Front Right obstacle sensor
            self._lineRightChannel = 15 # Right Line sensor
            self._lineLeftChannel = 12  # Left Line sensor
 
        elif board.PGType == board.PGLite:
            self._leftChannel = 7       # Front Left obstacle sensor
            self._rightChannel = 11      # Front Right obstacle sensor
            self._lineRightChannel = 13 # Right Line sensor
            self._lineLeftChannel = 12  # Left Line sensor
        
        else:
            raise Exception('Unknown Board')

        #set up digital line detectors as inputs
        GPIO.setup(self._lineRightChannel, GPIO.IN) # Right line sensor
        GPIO.setup(self._lineLeftChannel, GPIO.IN) # Left line sensor

        #Set up IR obstacle sensors as inputs
        GPIO.setup(self._leftChannel, GPIO.IN) # Left obstacle sensor
        GPIO.setup(self._rightChannel, GPIO.IN) # Right obstacle sensor
        if board.PGType == board.PGFull:
            GPIO.setup(self._frontChannel, GPIO.IN) # Centre Front obstacle sensor

            

    @staticmethod
    def _ir_check(channel):
        if GPIO.input(channel)==0:
            return True
        else:
            return False

    @property
    def front(self):
        return self._ir_check(self._frontChannel)

    @property
    def left(self):
        return self._ir_check(self._leftChannel)

    @property
    def right(self):
        return self._ir_check(self._rightChannel)

    @property
    def lineRight(self):
        return self._ir_check(self._lineRightChannel)

    @property
    def lineLeft(self):
        return self._ir_check(self._lineLeftChannel)

    @property
    def line(self):
        return (self.lineLeft, self.lineRight)

    @property
    def irAll(self):
        return self.left or self.front or self.right
