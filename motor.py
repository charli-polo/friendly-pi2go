# -*- coding: utf-8 -*-

from __future__ import print_function

# Import all necessary libraries
import RPi.GPIO as GPIO, sys, threading, time, os
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


class Motor:
    def __init__(self, speed):
        self._board = Board()

        #use pwm on inputs so motors don't go too fast
        GPIO.setup(Board.L1, GPIO.OUT)
        self._p = GPIO.PWM(Board.L1, 20)
        self._p.start(0)

        GPIO.setup(Board.L2, GPIO.OUT)
        self._q = GPIO.PWM(Board.L2, 20)
        self._q.start(0)

        GPIO.setup(Board.R1, GPIO.OUT)
        self._a = GPIO.PWM(Board.R1, 20)
        self._a.start(0)

        GPIO.setup(Board.R2, GPIO.OUT)
        self._b = GPIO.PWM(Board.R2, 20)
        self._b.start(0)

        self.speed = speed

        # Motor correction
        # If your robot goes to the right, put a positiv value.
        # If your robot goes to the left, put a negativ value.
        self.tuning = 0.1

        # references for turtle like movement
        self._time_ref_distance = 6.0 # Time for a unit of distance
        self._time_ref_spin = 2.475 # Time for a complete spin

    def stop(self):
        """stop(): Stop the motor"""
        self._p.ChangeDutyCycle(0)
        self._q.ChangeDutyCycle(0)
        self._a.ChangeDutyCycle(0)
        self._b.ChangeDutyCycle(0)

    def move(self, distance, speed=None):
        """move(distance, speed): move a certain number of steps (can be negativ)"""
        speed = speed if speed else self.speed
        if distance >= 0:
            self.goBoth(speed)
        else:
            self.goBoth(-speed)
        time.sleep(self._time_ref_distance * abs(distance))
        self.stop()

    def rotate(self, angle, speed=None):
        """rotate(angle, speed): rotate for angle degrees (can be negativ)"""
        speed = speed if speed else self.speed
        if angle >= 0:
            self.spinRight(speed)
        else:
            self.spinLeft(speed)
        time.sleep(abs(angle)/360.0 * self._time_ref_spin)
        self.stop()

    def turnRight(self, speed=None):
        speed = speed if speed else self.speed
        self.rotate(90, speed)

    def turnLeft(self, speed=None):
        speed = speed if speed else self.speed
        self.rotate(-90, speed)

    def turnAround(self, speed=None):
        speed = speed if speed else self.speed
        self.rotate(180, speed)

    def forward(self, speed = None):
        """forward(speed): Sets both motors to move forward at speed. 0 <= speed <= 100"""
        speed = speed if speed else self.speed
        self.go(speed,speed)

    def reverse(self, speed = None):
        """reverse(speed): Sets both motors to reverse at speed. 0 <= speed <= 100"""
        speed = speed if speed else self.speed
        self.go(-speed, -speed)

    def spinLeft(self, speed = None):
        """spinLeft(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100"""
        speed = speed if speed else self.speed
        self.go(-speed , speed)

    def spinRight(self, speed = None):
        """spinRight(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100"""
        speed = speed if speed else self.speed
        self.go(speed, -speed)

    def turnForward(self, leftSpeed, rightSpeed):
        """turnForward(leftSpeed, rightSpeed): Moves forwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100"""
        self.go(leftSpeed, rightSpeed)

    def turnReverse(self, leftSpeed, rightSpeed):
        """turnReverse(leftSpeed, rightSpeed): Moves backwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100"""
        self.go(-leftSpeed, -rightSpeed)

    def goBoth(self, speed):
        """go(speed): controls motors in both directions together with positive/negative speed parameter. -100<= speed <= 100"""
        if speed<0:
            self.reverse(abs(speed))
        else:
            self.forward(speed)

    def go(self, leftSpeed, rightSpeed):
        """go(leftSpeed, rightSpeed): controls motors in both directions independently using different positive/negative speeds. -100<= leftSpeed,rightSpeed <= 100"""
        if self.tuning > 0:
            leftSpeed = int(leftSpeed * (1 - self.tuning))
        else:
            rightSpeed = int(rightSpeed * (1 + self.tuning))

        if leftSpeed<0:
            self._p.ChangeDutyCycle(0)
            self._q.ChangeDutyCycle(abs(leftSpeed))
            self._q.ChangeFrequency(abs(leftSpeed) + 5)
        else:
            self._q.ChangeDutyCycle(0)
            self._p.ChangeDutyCycle(leftSpeed)
            self._p.ChangeFrequency(leftSpeed + 5)
        if rightSpeed<0:
            self._a.ChangeDutyCycle(0)
            self._b.ChangeDutyCycle(abs(rightSpeed))
            self._p.ChangeFrequency(abs(rightSpeed) + 5)
        else:
            self._b.ChangeDutyCycle(0)
            self._a.ChangeDutyCycle(rightSpeed)
            self._p.ChangeFrequency(rightSpeed + 5)
