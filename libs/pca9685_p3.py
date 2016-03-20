#!/usr/bin/python
#
# PCA9685 Library
# Purely used for Pi2Go, so not as flexible as other libraries
#
# Created by Gareth Davies, Feb 2016
# Copyright 4tronix
# Modified by Charlie Idrac to be python3 compatible
#
#
# This code is in the public domain and may be freely copied and used
# No warranty is provided or implied
#
#======================================================================

import smbus, time
bus = smbus.SMBus(1)

PCA = 0x40    # Fixed I2C Address of PC9685
SERVOS = 6+48
RED = 14
GREEN = 10
BLUE = 6

def init():
    bus.write_byte_data(PCA, 1, 0x04)  # set Mode2 outputs to push-pull
    mode1 = bus.read_byte_data(PCA, 0) # get current Mode1 register
    mode1 &= 0x7f # ignore the reset bit
    mode1 |= 0x10 # set Sleep bit
    bus.write_byte_data(PCA, 0, mode1) # sleep
    bus.write_byte_data(PCA, 254, 101)    # set prescaler
    mode1 &= 0xef # clear Sleep bit
    bus.write_byte_data(PCA, 0, mode1 | 0x80) # wake up
    time.sleep(0.005)

def setServo(servo, degrees):
    # note servo 0 is output 12 (ie. offset 48) on the PCA9685
    # change degrees to a value between 175 (-90) to 575 (+90)
    if (servo>=0) and (servo<=3) and (degrees>= -90) and (degrees<=90):
        start = 0
        stop = 175 + ((degrees + 90) * 400) / 180
        print("Setting:", 6 + 48 + servo*4, start, stop)
        bus.write_byte_data(PCA, SERVOS + servo*4 + 0, start & 0xff)
        bus.write_byte_data(PCA, SERVOS + servo*4 + 1, start >> 8)
        bus.write_byte_data(PCA, SERVOS + servo*4 + 2, stop & 0xff)
        bus.write_byte_data(PCA, SERVOS + servo*4 + 3, stop >> 8)

def stopServo(servo):
    if (servo>=0) and (servo<=3):
        bus.write_byte_data(PCA, SERVOS + servo*4 + 0, 0)
        bus.write_byte_data(PCA, SERVOS + servo*4 + 1, 0)
        bus.write_byte_data(PCA, SERVOS + servo*4 + 2, 0)
        bus.write_byte_data(PCA, SERVOS + servo*4 + 3, 0)

def setRGBLED(led, red, green, blue):
        bus.write_byte_data(PCA, RED + led*12 + 0, 0)
        bus.write_byte_data(PCA, RED + led*12 + 1, 0)
        bus.write_byte_data(PCA, RED + led*12 + 2, red & 0xff)
        bus.write_byte_data(PCA, RED + led*12 + 3, red >> 8)
        bus.write_byte_data(PCA, GREEN + led*12 + 0, 0)
        bus.write_byte_data(PCA, GREEN + led*12 + 1, 0)
        bus.write_byte_data(PCA, GREEN + led*12 + 2, green & 0xff)
        bus.write_byte_data(PCA, GREEN + led*12 + 3, green >> 8)
        bus.write_byte_data(PCA, BLUE + led*12 + 0, 0)
        bus.write_byte_data(PCA, BLUE + led*12 + 1, 0)
        bus.write_byte_data(PCA, BLUE + led*12 + 2, blue & 0xff)
        bus.write_byte_data(PCA, BLUE + led*12 + 3, blue >> 8)
