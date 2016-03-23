# -*- coding: utf-8 -*-
from __future__ import print_function

import RPi.GPIO as GPIO
import sys, threading, time, os
#from Adafruit_PWM_Servo_Driver import PWM
import pca9685_p3 as pca9685
from sgh_PCF8591P_p3 import sgh_PCF8591P

class Board:
    # Define Type of Pi2Go
    PGNone = 0
    PGFull = 1
    PGLite = 2
    PGType = PGNone # Set to None until we find out which during init()

    # Pins 24, 26 Left Motor
    # Pins 19, 21 Right Motor
    L1 = 26
    L2 = 24
    R1 = 19
    R2 = 21

    # Button switch
    SWITCH = 16
    LSWITCH = 23

    # Define Colour IDs for the RGB LEDs (Pi2Go full only)
    # Blue = 0
    # Green = 1
    # Red = 2
    # pwmMax = 4095 # maximum PWM value

    # Define GPIO pins for Front/rear LEDs on Pi2Go-Lite
    frontLED = 15
    rearLED = 16
    
    def __init__(self):
        # global p, q, a, b, pwm, pcfADC, PGType
        # global irFL, irFR, irMID, lineLeft, lineRight
        GPIO.setwarnings(False)

        self.PGType = self.PGFull
        # Initialise the PCA9685 PWM device using the default address
        try:
            pca9685.init()
        except:
            self.PGType = self.PGLite # No PCA9685 so set to Pi2Go-Lite

        # Initalise the ADC
        self.pcfADC = None # ADC object
        try:
            self.pcfADC = sgh_PCF8591P(1) #i2c, 0x48)
        except:
            self.PGType = self.PGLite

        #use physical pin numbering
        GPIO.setmode(GPIO.BOARD)
        

        # # Define IR sensor pins - varies with model
        # if PGType == PGLite:
        #     lineRight = lineRight_2
        #     lineLeft = lineLeft_2
        #     irFL = irFL_2
        #     irFR = irFR_2
        #     irMID = 0
        # else:
        #     lineRight = lineRight_1
        #     lineLeft = lineLeft_1
        #     irFL = irFL_1
        #     irFR = irFR_1
        #     irMID = irMID_1
        #
        # #set up digital line detectors as inputs
        # GPIO.setup(lineRight, GPIO.IN) # Right line sensor
        # GPIO.setup(lineLeft, GPIO.IN) # Left line sensor
        #
        # #Set up IR obstacle sensors as inputs
        # GPIO.setup(irFL, GPIO.IN) # Left obstacle sensor
        # GPIO.setup(irFR, GPIO.IN) # Right obstacle sensor
        # if PGType == PGFull:
        #     GPIO.setup(irMID, GPIO.IN) # Centre Front obstacle sensor


        # # initialise servos (Pi2Go-Lite only)
        # if PGType == PGLite:
        #     startServos()
        #
        # #set up Pi2Go-Lite White LEDs as outputs
        #     GPIO.setup(frontLED, GPIO.OUT)
        #     GPIO.output(frontLED, 1)    # switch front LEDs off as they come on by default
        #     GPIO.setup(rearLED, GPIO.OUT)
        #     GPIO.output(rearLED, 1)    # switch rear LEDs off as they come on by default
        #
        # #set switch as input with pullup
        # if PGType == PGLite:
        #     GPIO.setup(Lswitch, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        # else:
        #     GPIO.setup(switch, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        #
        # # initialise wheel counters if Pi2Go-Lite
        # if PGType == PGLite:
        #     threadC = threading.Thread(target = wheelCount)
        #     threadC.start()
        #     running = True
