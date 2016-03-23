# -*- coding: utf-8 -*-

from __future__ import print_function

# Import all necessary libraries
import time
import RPi.GPIO as GPIO
from board import Board as Board




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
