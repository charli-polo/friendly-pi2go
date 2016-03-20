# -*- coding: utf-8 -*-
#! python3

from __future__ import print_function
# Import all necessary libraries
import RPi.GPIO as GPIO, sys, threading, time, os, smbus
# from Adafruit_PWM_Servo_Driver import PWM
import pca9685_p3
from sgh_PCF8591P_p3 import sgh_PCF8591P

# for Push repr only...
# from datetime import datetime

class Robot:

    def __init__(self):
        init()
        self.PGType = PGType
        if PGType == 1:
            self.version = "Full Pi2Go"
        elif PGType == 2:
            self.version = "Pi2Go-Lite"

        self.lcd = Lcd7segment()
        self.ir = InfraRed()
        self._speed = 50
        self.motor = Motor(self._speed)
        self.button = Button(self)
        self.leds = Leds()
        print('Robot is ready !')

    def cleanup(self):
        """Reset everything (leds and stuff)"""
        cleanup()

    def sleep(self, seconds):
        """ shortcut to time.sleep"""
        time.sleep(seconds)

    def say(self, words):
        """Scroll some text on the LCD"""
        self.lcd.scroll(words, 1)

    def write(self, words):
        """Put some fixed text on the lcd"""
        self.lcd7segment.sDisplay(words)

    def forward(self, speed = None):
        """Move forward at speed"""
        speed = speed if speed else self.motor.speed
        forward(speed)

    def reverse(self, speed = None):
        """Move backward at speed"""
        speed = speed if speed else self.motor.speed
        reverse(speed)

    def stop(self):
        """Stop the motors"""
        stop()

    def getDistance(self):
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



class Motor:
    def __init__(self, speed):
        self.speed = speed
        self.tuning = 0

    def spinLeft(self, speed = None):
        """spinLeft(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
        """
        speed = speed if speed else self.speed
        spinLeft(speed)

    def spinRight(self, speed = None):
        """spinRight(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
        """
        speed = speed if speed else self.speed
        spinRight(speed)

    def turnForward(self, leftSpeed, rightSpeed):
        """turnForward(leftSpeed, rightSpeed): Moves forwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100"""
        turnForward(leftSpeed, rightSpeed)

    def turnReverse(self, leftSpeed, rightSpeed):
        """# turnReverse(leftSpeed, rightSpeed): Moves backwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100"""
        turnReverse(leftSpeed, rightSpeed)

    def go(self, leftSpeed, rightSpeed):
        """# go(leftSpeed, rightSpeed): controls motors in both directions independently using different positive/negative speeds. -100<= leftSpeed,rightSpeed <= 100"""
        # go(leftSpeed + self.tuning, rightSpeed - self.tuning)
        go(leftSpeed , rightSpeed)


    def goBoth(self, speed = None):
        """go(speed): controls motors in both directions together with positive/negative speed parameter. -100<= speed <= 100"""
        speed = speed if speed else self.speed
        go(speed, speed)
        # self.go(speed, speed)

class Push:
    TYPES = { 'short': 1,
              'long': 2,
              'double':3}

    def __init__(self):
        self.start = time.time()
        self._duration = None
        self.end = None
        self.type = self.TYPES['short']

    def stop(self):
        self.end = time.time()
        self._duration = self.end - self.start

    @property
    def duration(self):
        if self._duration is None:
            return time.time() - self.start
        else:
            return self._duration

    def _strtime(self, t):
        a = time.strftime("%H:%M:%S", time.localtime(t))
        b = int((t - int(t))*1000)
        return "{}.{:03}".format( a , b )

    def __repr__(self):
        return "type:{} {} -> {} ({:.2f}) ".format(self.type, self._strtime(self.start), self._strtime(self.end), self.duration)

def slingshot(delay, channel, callback, *args):
    """ Trigers the callback after the delay unless the channel is changed.
    Typical use is to check for a doubleclick.
    Callback is called with *args.
    """
    t0 = time.time()
    state = GPIO.input(channel)
    while state == GPIO.input(channel) :
        if (time.time() - t0 > delay) :
            return callback(*args)
        # time.sleep(sleep)

def windowshot(delay, channel, callback, *args):
    """ Trigers the callback only if the channel is changed before the delay ends
    Callback is called with *args.
    return True if the callback is calles
    """
    t0 = time.time()
    state = GPIO.input(channel)
    while time.time() - t0 < delay:
        if state != GPIO.input(channel) :
            callback(*args)
            return True
        # time.sleep(sleep)
    return False



class Button:
    CLICK_LONG = 1.0
    PAUSE_DOUBLE_CLICK = 0.5
    # Define pins for switch (different on each version)
    #EV_CLIC = ('short')
    #EV_CLIC_LONG = ('long')
    #EV_DOUBLE_CLIC = ('short', 'short_pause', 'short')
    #EVENTS = (EV_CLIC, EV_CLIC_LONG, EV_DOUBLE_CLIC)

    def __init__(self, robot):
        # TODO add a singleton method to avoid errors ??
        # Make a meta event list to avoid messing around with the raw events list
        # make some method to create callbacks or multiple call backs
        # have them executed in a separate thread ?
        # self.pg = rpigpio
        self.debug = False
        self.robot = robot

        # Setup
        if self.robot.PGType == 1:
            self.channel = Board.SWITCH
        elif self.robot.PGType == 2:
            self.channel = Board.LSWITCH
            #set switch as input with pullup
        GPIO.setup(self.channel, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.events = []
        self.double_click = False
        GPIO.remove_event_detect(self.channel)
        GPIO.add_event_detect(self.channel, GPIO.BOTH, callback=self._callback)

        self.click_callback = None
        self.double_click_callback = None
        self.long_click_callback = None
        self.while_long_click_callback = None

    def _debug(self, *args):
        if self.debug:
            print(*args)

    def is_pushed(self):
        return not GPIO.input(self.channel)

    def _callback(self, channel=None):
        channel = channel if channel else self.channel
        if self.is_pushed():
            # PUSH START
            # interception du deuxième clic d'un double click
            if self.double_click :
                self.double_click = False
                return False
            new_push = Push()
            self.events.append(new_push)
            slingshot(self.CLICK_LONG, self.channel, self._start_long_click, new_push)
        else:
            # PUSH END
            if len(self.events) == 0 :
                raise Exception("No events yet so push end is inconcistent")

            last_push = self.events[-1]
            last_push.stop()
            if last_push.type == last_push.TYPES['short']:
                self._end_short_click(last_push)
            elif last_push.type == last_push.TYPES['long']:
                self._end_long_click(last_push)
            elif last_push.type == last_push.TYPES['double']:
                self._end_double_click(last_push)

    def _start_short_click(self, push):
        """ Not used currently """
        pass

    def _end_short_click(self, push):
        self._debug('short click', push)
        if not windowshot(self.PAUSE_DOUBLE_CLICK, self.channel, self._start_double_click, push):
            self.execute_short_click(push)


    def _start_double_click(self, push):
        self._debug('start double click', push)
        push.type = push.TYPES['double']
        self.double_click = True
        # we need to put the following event also as double click

    def _end_double_click(self, push):
        self._debug('end double click', push)
        self.execute_double_click(push)

    def _start_long_click(self, push):
        self._debug('doing click long')
        push.type = push.TYPES['long']
        #while self.is_pushed():
            #time.sleep(0.5)
            #self._debug('still doing click long', time.time() - push.start)
            # print perc, intensity
            # pg.setLED(0,intensity,int(intensity/2),int(intensity/3))
        self.execute_while_long_click(push)

    def _end_long_click(self, push):
        self._debug('long click', push)
        self.execute_long_click(push)

    def execute_while_long_click(self, push):
        self._debug('while long click', push)
        if self.while_long_click_callback is not None:
            self.while_long_click_callback(push)
        else :
            n = 0
            while self.is_pushed():
                self.robot.lcd.sDisplay("{:04}".format(int(push.duration*100)), 2)
                n += 1

    def execute_double_click(self, push):
        if self.double_click_callback is not None:
            self.double_click_callback(push)
        else :
            self.robot.say('double click')

    def execute_short_click(self, push):
        if self.click_callback is not None:
            self.click_callback(push)
        else :
            self.robot.say('short click')

    def execute_long_click(self, push):
        if self.long_click_callback is not None:
            self.long_click_callback(push)
        else :
            self.robot.say('long click')

class Leds:
    def __init__(self):
        self.front = Led(3)
        self.right = Led(2)
        self.rear = Led(1)
        self.left = Led(0)
        self.all = [self.front, self.right, self.rear, self.left]
        self.off()

    def _set(self, red, green, blue):
        """ Sets all LEDs to required RGB. 0 <= R,G,B <= 4095"""
        for led in self.all:
            led.set(red, green, blue)

    def off(self):
        for led in self.all:
            led.off()


class Led:
    MAX_LED_VALUE = 4096

    def __init__(self, LED):
        # TODO: _red doit-il contenir le pourcentage ou la valeur entière ?
        self._LED = LED
        self._red = 0
        self._green = 0
        self._blue = 0

    @classmethod
    def _integer_to_percentage(cls, n_value):
        """ transform an integer into percentage """
        n = n_value % cls.MAX_LED_VALUE
        perc = n/(cls.MAX_LED_VALUE-1)
        return round(perc ,2)

    @classmethod
    def _percentage_to_integer(cls, percentage):
        """ transform a percentage into an integer < 4096 """
        n = int(percentage * (cls.MAX_LED_VALUE-1))
        return n


    def _set(self, red, green, blue):
        """ Sets the LED specified to required RGB value. 0 <= R,G,B <= 4095"""
        pca9685_p3.setRGBLED(self._LED, red, green, blue)

    @property
    def red(self):
        perc = self._integer_to_percentage(self._red)
        return perc

    @red.setter
    def red(self, perc):
        self._red = self._percentage_to_integer(perc)
        self._set(self._red, self._green, self._blue)

    @property
    def green(self):
        perc = self._integer_to_percentage(self._green)
        return perc

    @green.setter
    def green(self, perc):
        self._green = self._percentage_to_integer(perc)
        self._set(self._red, self._green, self._blue)

    @property
    def blue(self):
        perc = self._integer_to_percentage(self._blue)
        return perc

    @blue.setter
    def blue(self, perc):
        self._blue = self._percentage_to_integer(perc)
        self._set(self._red, self._green, self._blue)

    def __repr__(self):
        return "<Robot.Leds.Led {}: red {}, green {}, blue {}>".format(self._LED, self.red, self.green, self.blue)

    def off(self):
        self.blue = 0.0
        self.green = 0.0
        self.red = 0.0

    def rgb(self, red, green, blue):
        self.blue = blue
        self.green = green
        self.red = red




class InfraRed:
    def __init__(self):
        self._frontChannel = 13 # Middle obstacle sensor - not available on Lite version
        self._leftChannel = 11 # Front Left obstacle sensor
        self._rightChannel = 7 # Front Right obstacle sensor
        self._lineRightChannel = 15 # Right Line sensor
        self._lineLeftChannel = 12  # Left Line sensor

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


class Lcd7segment:
    # TODO: le mot SPEED est trompeur....
    ADDR = 0x20 # I2C address of MCP23017
    SPEED = 0.005
    LETTERS = {
        'a' : 0b01110111,
        'b' : 0b01111100,
        'c' : 0b01011000,
        'd' : 0b01011110,
        'e' : 0b01111001,
        'f' : 0b01110001,
        'g' : 0b01101111,
        'h' : 0b01110100,
        'i' : 0b00110000,
        'j' : 0b00011110,
        'k' : 0b00111001,
        'l' : 0b00111000,
        'm' : 0b00110111,
        'n' : 0b01010100,
        'o' : 0b01011100,
        'p' : 0b01110011,
        'q' : 0b01100111,
        'r' : 0b01010000,
        's' : 0b01101101,
        't' : 0b01111000,
        'u' : 0b00011100,
        'v' : 0b00011100,
        'w' : 0b00011100,
        'x' : 0b01010010,
        'y' : 0b01101110,
        'z' : 0b01011011,

        '0' : 0b00111111,
        '1' : 0b00000110,
        '2' : 0b01011011,
        '3' : 0b01001111,
        '4' : 0b01100110,
        '5' : 0b01101101,
        '6' : 0b01111101,
        '7' : 0b00000111,
        '8' : 0b01111111,
        '9' : 0b01101111,

        ' ' : 0b00000000,
        '-' : 0b01000000,
        '.' : 0b10000000,
        ',' : 0b10000000,
        '!' : 0b10000110,
        "'" : 0b00100000,
        '"' : 0b00100010
    }
    # TODO: permettre un affichage permanent pour ces deux fonctions
    # Ajouter des fonctions qui permettent de cibler un segment en particulier notamment pour permettre des animations de type chenillard

    def __init__(self):
        if GPIO.RPI_REVISION > 1:
            self.bus = smbus.SMBus(1) # For revision 1 Raspberry Pi, change to bus = smbus.SMBus(1) for revision 2.
        else:
            self.bus = smbus.SMBus(0) # For revision 1 Raspberry Pi, change to bus = smbus.SMBus(1) for revision 2.

        self.bus.write_byte_data(self.ADDR, 0x00, 0x00) # Set all of bank 0 to outputs
        self.bus.write_byte_data(self.ADDR, 0x01, 0x00) # Set all of bank 1 to outputs
        self.bus.write_byte_data(self.ADDR, 0x13, 0xff) # Set all of bank 1 to High (Off)

    def scroll(self, str1, count=1):
        """Display a string by scrolling count times.
        """
        # TODO: permettre un affichage permanent pour ces deux fonctions
        string = '   ' + str1 + '   '
        for j in range(count):
            for i in range(len(string)-3):
                str2 = string[i:(i+4)]
                self.sDisplay(str2)


    def sDisplay(self, safeStr, count = 10):
        """Display the 4 first letters of a string for 10 (count) cycles (0,2 in total).
        Includes multiplexing, each cycle has SPEED x 4 duration (0,005 x 4 = 0,02)
        """
        # TODO: changer count en secondes
        safeStr = safeStr.lower()
        d1 = safeStr[3]
        d2 = safeStr[2]
        d3 = safeStr[1]
        d4 = safeStr[0]
        count2 = 0
        while count2 < count:
            self._sendDigit(self.LETTERS[d1], 0) # '1'
            time.sleep(self.SPEED)
            self._sendDigit(0, 0)
            self._sendDigit(self.LETTERS[d2], 1) # '2'
            time.sleep(self.SPEED)
            self._sendDigit(0, 1)
            self._sendDigit(self.LETTERS[d3], 2) # '3'
            time.sleep(self.SPEED)
            self._sendDigit(0, 2)
            self._sendDigit(self.LETTERS[d4], 3) # '0'
            time.sleep(self.SPEED)
            self._sendDigit(0, 3)
            count2 += 1

    def _sendDigit(self, digit, pos):
        """Send a digit (bit) to the pos position of the lcd."""
        t = (1<<pos) ^ 255
        self.bus.write_byte_data(self.ADDR, 0x13, t) # Set bank 1 Pos to Low
        self.bus.write_byte_data(self.ADDR, 0x12, digit) # Set bank 0 to digit

    def setLetter(self, digit, pos):
        """ Set a letter (or number) to one of the 4 digits of the LCD """
        self._sendDigit(self.LETTERS[str(digit)[0]], pos)




def blink(r, g, b, sec=0.25, count=5 ):
    for i in range(count):
        pg.setAllLEDs(r,g,b)
        time.sleep(sec)
        pg.setAllLEDs(0,0,0)
        time.sleep(sec)




#
# Python Module to externalise all Pi2Go specific hardware
#
# Created by Gareth Davies and Zachary Igielman, May 2014
# Updated June 2014 to include Pi2Go-Lite within same framework
# Copyright 4tronix
#
# This code is in the public domain and may be freely copied and used
# No warranty is provided or implied
#
#======================================================================


#======================================================================
# General Functions
# (Both versions)
#
# init(). Initialises GPIO pins, switches motors and LEDs Off, etc
# cleanup(). Sets all motors and LEDs off and sets GPIO to standard values
# version(). Returns 1 for Full Pi2Go, and 2 for Pi2Go-Lite. Invalid until after init() has been called
#======================================================================


#======================================================================
# Motor Functions
# (Both Versions)
#
# stop(): Stops both motors
# forward(speed): Sets both motors to move forward at speed. 0 <= speed <= 100
# reverse(speed): Sets both motors to reverse at speed. 0 <= speed <= 100
# spinLeft(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
# spinRight(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
# turnForward(leftSpeed, rightSpeed): Moves forwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100
# turnreverse(leftSpeed, rightSpeed): Moves backwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100
# go(leftSpeed, rightSpeed): controls motors in both directions independently using different positive/negative speeds. -100<= leftSpeed,rightSpeed <= 100
# go(speed): controls motors in both directions together with positive/negative speed parameter. -100<= speed <= 100
#======================================================================


#======================================================================
# Wheel Sensor Functions
# (Pi2Go-Lite Only)
# stepForward(speed, steps): moves the unit forwards specified number of steps, then stops
# stepReverse(speed, steps): Moves backward specified number of counts, then stops
# stepSpinL(speed, steps): Spins left specified number of counts, then stops
# stepSpinR(speed, steps): Spins right specified number of counts, then stops
#======================================================================


#======================================================================
# RGB LED Functions
# (Full Pi2Go only)
#
# setLED(LED, Red, Green, Blue): Sets the LED specified to required RGB value. 0 >= LED <= 4; 0 <= R,G,B <= 4095
# setAllLEDs(Red, Green, Blue): Sets all LEDs to required RGB. 0 <= R,G,B <= 4095
#======================================================================


#======================================================================
# WHITE LED Functions
# (Pi2Go-Lite only)
#
# LsetLED(LED, value): Sets the LED specified to OFF == 0 or ON >= 1
# LsetAllLEDs(value): Sets both LEDs to OFF == 0 or ON >= 1
#======================================================================


#======================================================================
# IR Sensor Functions
# (Both Versions)
#
# irLeft(): Returns state of Left IR Obstacle sensor
# irRight(): Returns state of Right IR Obstacle sensor
# irCentre(): Returns state of Centre IR Obstacle sensor (Full Pi2Go Only)
# irAll(): Returns true if any of the Obstacle sensors are triggered
# irLeftLine(): Returns state of Left IR Line sensor
# irRightLine(): Returns state of Right IR Line sensor
#======================================================================


#======================================================================
# UltraSonic Functions
# (Both Versions)
#
# getDistance(). Returns the distance in cm to the nearest reflecting object. 0 == no object
#======================================================================


#======================================================================
# Light Sensor Functions
# (Full Pi2Go only)
#
# getLight(Sensor). Returns the value 0..1023 for the selected sensor, 0 <= Sensor <= 3
# getLightFL(). Returns the value 0..1023 for Front-Left light sensor
# getLightFR(). Returns the value 0..1023 for Front-Right light sensor
# getLightBL(). Returns the value 0..1023 for Back-Left light sensor
# getLightBR(). Returns the value 0..1023 for Back-Right light sensor
#======================================================================


#======================================================================
# Servo Functions
#
# startServos(). Initialises the servo background process
# stop Servos(). terminates the servo background process
# setServo(Servo, Degrees). Sets the servo to position in degrees -90 to +90
#======================================================================


#======================================================================
# Switch Functions
#
# getSwitch(). Returns the value of the tact switch: True==pressed
#======================================================================

class Board:
    """ A class to contain and initialize the board...
    """
    SWITCH = 16
    LSWITCH = 23


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

#=======
# Define obstacle sensors and line sensors
#Pi2Go
irFL_1 = 11      # Front Left obstacle sensor
irFR_1 = 7       # Front Right obstacle sensor
irMID_1 = 13     # Middle obstacle sensor - not available on Lite version
lineRight_1 = 15 # Right Line sensor
lineLeft_1 = 12  # Left Line sensor
#---
#Pi2Go-Lite
irFL_2 = 7       # Front Left obstacle sensor
irFR_2 = 11      # Front Right obstacle sensor
lineRight_2 = 13 # Right Line sensor
lineLeft_2 = 12  # Left Line sensor
#========
irFL = 0
irFR = 0
irMID = 0
lineRight = 0
lineLeft = 0

# Define Colour IDs for the RGB LEDs (Pi2Go full only)
Blue = 0
Green = 1
Red = 2
pwmMax = 4095 # maximum PWM value

# Define GPIO pins for Front/rear LEDs on Pi2Go-Lite
frontLED = 15
rearLED = 16

# Define Sonar Pin (same pin for both Ping and Echo
sonar = 8


# Define if servo background process is active
ServosActive = False

# Global variables for wheel sensor counting
running = True
countL = 0
countR = 0


#======================================================================
# General Functions
#
# init(). Initialises GPIO pins, switches motors and LEDs Off, etc
def init():
    global p, q, a, b, pwm, pcfADC, PGType
    global irFL, irFR, irMID, lineLeft, lineRight
    GPIO.setwarnings(False)

    PGType = PGFull
    # Initialise the PCA9685 PWM device using the default address
    try:
        #pwm = PWM(0x40, debug = False)
        #pwm.setPWMFreq(60)  # Set frequency to 60 Hz
        pca9685_p3.init()
    except:
        PGType = PGLite # No PCA9685 so set to Pi2Go-Lite

    # Initalise the ADC
    pcfADC = None # ADC object
    try:
        pcfADC = sgh_PCF8591P(1) #i2c, 0x48)
    except:
        PGType = PGLite

    #use physical pin numbering
    GPIO.setmode(GPIO.BOARD)

    # Define IR sensor pins - varies with model
    if PGType == PGLite:
        lineRight = lineRight_2
        lineLeft = lineLeft_2
        irFL = irFL_2
        irFR = irFR_2
        irMID = 0
    else:
        lineRight = lineRight_1
        lineLeft = lineLeft_1
        irFL = irFL_1
        irFR = irFR_1
        irMID = irMID_1

    #set up digital line detectors as inputs
    GPIO.setup(lineRight, GPIO.IN) # Right line sensor
    GPIO.setup(lineLeft, GPIO.IN) # Left line sensor

    #Set up IR obstacle sensors as inputs
    GPIO.setup(irFL, GPIO.IN) # Left obstacle sensor
    GPIO.setup(irFR, GPIO.IN) # Right obstacle sensor
    if PGType == PGFull:
        GPIO.setup(irMID, GPIO.IN) # Centre Front obstacle sensor

    #use pwm on inputs so motors don't go too fast
    GPIO.setup(L1, GPIO.OUT)
    p = GPIO.PWM(L1, 20)
    p.start(0)

    GPIO.setup(L2, GPIO.OUT)
    q = GPIO.PWM(L2, 20)
    q.start(0)

    GPIO.setup(R1, GPIO.OUT)
    a = GPIO.PWM(R1, 20)
    a.start(0)

    GPIO.setup(R2, GPIO.OUT)
    b = GPIO.PWM(R2, 20)
    b.start(0)

    # initialise servos (Pi2Go-Lite only)
    if PGType == PGLite:
        startServos()

    #set up Pi2Go-Lite White LEDs as outputs
        GPIO.setup(frontLED, GPIO.OUT)
        GPIO.output(frontLED, 1)    # switch front LEDs off as they come on by default
        GPIO.setup(rearLED, GPIO.OUT)
        GPIO.output(rearLED, 1)    # switch rear LEDs off as they come on by default


    # initialise wheel counters if Pi2Go-Lite
#    if PGType == PGLite:
#        threadC = threading.Thread(target = wheelCount)
#        threadC.start()
#        running = True


# cleanup(). Sets all motors and LEDs off and sets GPIO to standard values
def cleanup():
    global running
    running = False
    stop()
    setAllLEDs(0, 0, 0)
    stopServos()
    time.sleep(1)
    GPIO.cleanup()


# version(). Returns 1 for Full Pi2Go, and 2 for Pi2Go-Lite. Invalid until after init() has been called
def version():
    return PGType

# End of General Functions
#======================================================================


#======================================================================
# Motor Functions
# (both versions)
#
# stop(): Stops both motors
def stop():
    p.ChangeDutyCycle(0)
    q.ChangeDutyCycle(0)
    a.ChangeDutyCycle(0)
    b.ChangeDutyCycle(0)

# forward(speed): Sets both motors to move forward at speed. 0 <= speed <= 100
def forward(speed):
    p.ChangeDutyCycle(speed)
    q.ChangeDutyCycle(0)
    a.ChangeDutyCycle(speed)
    b.ChangeDutyCycle(0)
    p.ChangeFrequency(speed + 5)
    a.ChangeFrequency(speed + 5)

# reverse(speed): Sets both motors to reverse at speed. 0 <= speed <= 100
def reverse(speed):
    p.ChangeDutyCycle(0)
    q.ChangeDutyCycle(speed)
    a.ChangeDutyCycle(0)
    b.ChangeDutyCycle(speed)
    q.ChangeFrequency(speed + 5)
    b.ChangeFrequency(speed + 5)

# spinLeft(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
def spinLeft(speed):
    p.ChangeDutyCycle(0)
    q.ChangeDutyCycle(speed)
    a.ChangeDutyCycle(speed)
    b.ChangeDutyCycle(0)
    q.ChangeFrequency(speed + 5)
    a.ChangeFrequency(speed + 5)

# spinRight(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
def spinRight(speed):
    p.ChangeDutyCycle(speed)
    q.ChangeDutyCycle(0)
    a.ChangeDutyCycle(0)
    b.ChangeDutyCycle(speed)
    p.ChangeFrequency(speed + 5)
    b.ChangeFrequency(speed + 5)

# turnForward(leftSpeed, rightSpeed): Moves forwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100
def turnForward(leftSpeed, rightSpeed):
    p.ChangeDutyCycle(leftSpeed)
    q.ChangeDutyCycle(0)
    a.ChangeDutyCycle(rightSpeed)
    b.ChangeDutyCycle(0)
    p.ChangeFrequency(leftSpeed + 5)
    a.ChangeFrequency(rightSpeed + 5)

# turnReverse(leftSpeed, rightSpeed): Moves backwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100
def turnReverse(leftSpeed, rightSpeed):
    p.ChangeDutyCycle(0)
    q.ChangeDutyCycle(leftSpeed)
    a.ChangeDutyCycle(0)
    b.ChangeDutyCycle(rightSpeed)
    q.ChangeFrequency(leftSpeed + 5)
    b.ChangeFrequency(rightSpeed + 5)

# go(leftSpeed, rightSpeed): controls motors in both directions independently using different positive/negative speeds. -100<= leftSpeed,rightSpeed <= 100
def go(leftSpeed, rightSpeed):
    if leftSpeed<0:
        p.ChangeDutyCycle(0)
        q.ChangeDutyCycle(abs(leftSpeed))
        q.ChangeFrequency(abs(leftSpeed) + 5)
    else:
        q.ChangeDutyCycle(0)
        p.ChangeDutyCycle(leftSpeed)
        p.ChangeFrequency(leftSpeed + 5)
    if rightSpeed<0:
        a.ChangeDutyCycle(0)
        b.ChangeDutyCycle(abs(rightSpeed))
        p.ChangeFrequency(abs(rightSpeed) + 5)
    else:
        b.ChangeDutyCycle(0)
        a.ChangeDutyCycle(rightSpeed)
        p.ChangeFrequency(rightSpeed + 5)

# go(speed): controls motors in both directions together with positive/negative speed parameter. -100<= speed <= 100
def goBoth(speed):
    if speed<0:
        reverse(abs(speed))
    else:
        forward(speed)

# End of Motor Functions
#======================================================================



#======================================================================
# RGB LED Functions
# (Full version only)
#
# setLED(LED, Red, Green, Blue): Sets the LED specified to required RGB value. 0 >= LED <= 3; 0 <= R,G,B <= 4095
def setLED(LED, red, green, blue):
    if PGType == PGFull:
        pca9685_p3.setRGBLED(LED, red, green, blue)

# setAllLEDs(Red, Green, Blue): Sets all LEDs to required RGB. 0 <= R,G,B <= 4095
def setAllLEDs (red, green, blue):
  for i in range(4):
    setLED(i, red, green, blue)

# End of RGB LED Functions
#======================================================================


#======================================================================
# White LED Functions
# (Pi2Go-Lite only)
#
# LsetLED(LED, value): Sets the LED specified to OFF == 0 or ON == 1
# TODO: take value from 0 to 100 and use as percentage PWM value
def LsetLED (LED, value):
    if PGType == PGLite:
        if value == 0:
            value = 1
        else:
            value = 0
        if LED == 0:
            GPIO.output (frontLED, value)
        else:
            GPIO.output (rearLED, value)

# LsetAllLEDs(value): Sets both LEDs to OFF == 0 or ON == 1

# End of White LED Functions
#======================================================================


#======================================================================
# IR Sensor Functions
#
# irLeft(): Returns state of Left IR Obstacle sensor
def irLeft():
    if GPIO.input(irFL)==0:
        return True
    else:
        return False

# irRight(): Returns state of Right IR Obstacle sensor
def irRight():
    if GPIO.input(irFR)==0:
        return True
    else:
        return False

# irCentre(): Returns state of Centre IR Obstacle sensor
# (Not available on Pi2Go-Lite)
def irCentre():
    if PGType != PGFull:
        return False
    if GPIO.input(irMID)==0:
        return True
    else:
        return False

# irAll(): Returns true if any of the Obstacle sensors are triggered
def irAll():
    if GPIO.input(irFL)==0 or GPIO.input(irFR)==0 or (PGType==PGFull and GPIO.input(irMID)==0):
        return True
    else:
        return False

# irLeftLine(): Returns state of Left IR Line sensor
def irLeftLine():
    if GPIO.input(lineLeft)==0:
        return True
    else:
        return False

# irRightLine(): Returns state of Right IR Line sensor
def irRightLine():
    if GPIO.input(lineRight)==0:
        return True
    else:
        return False

# End of IR Sensor Functions
#======================================================================


#======================================================================
# UltraSonic Functions
#
# getDistance(). Returns the distance in cm to the nearest reflecting object. 0 == no object
# (Both versions)
#
def getDistance():
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

# End of UltraSonic Functions
#======================================================================


#======================================================================
# Light Sensor Functions
# (Full Pi2Go Only)
#
# getLight(sensor). Returns the value 0..1023 for the selected sensor, 0 <= Sensor <= 3
def getLight(sensor):
    if PGType != PGFull:
        return False
    value  = pcfADC.readADC(sensor)
    return value

# getLightFL(). Returns the value 0..1023 for Front-Left light sensor
def getLightFL():
    if PGType != PGFull:
        return False
    value  = pcfADC.readADC(0)
    return value

# getLightFR(). Returns the value 0..1023 for Front-Right light sensor
def getLightFR(sensor):
    if PGType != PGFull:
        return False
    value  = pcfADC.readADC(1)
    return value

# getLightBL(). Returns the value 0..1023 for Back-Left light sensor
def getLightBL(sensor):
    if PGType != PGFull:
        return False
    value  = pcfADC.readADC(2)
    return value

# getLightBR(). Returns the value 0..1023 for Back-Right light sensor
def getLightBR(sensor):
    if PGType != PGFull:
        return False
    value  = pcfADC.readADC(3)
    return value

# End of Light Sensor Functions
#======================================================================


#======================================================================
# Switch Functions
#
# getSwitch(). Returns the value of the tact switch: True==pressed
def getSwitch():
    if PGType == 1:
        val = GPIO.input(switch)
    else:
        val = GPIO.input(Lswitch)
    return (val == 0)
#
# End of switch functions
#======================================================================



#======================================================================
# Servo Functions
# Pi2Go-Lite uses ServoD to control servos
# Pi2Go Full uses the PCA9685 hardware controller

def setServo(Servo, Degrees):
    #print "ServosActive:", ServosActive
    if ServosActive == False and PGType == PGLite:
        startServos()
    if (PGType == PGLite):
        pinServod (Servo, Degrees) # for now, simply pass on the input values
    else:
        pca9685_p3.setServo(Servo, Degrees)

def stopServos():
    if (PGType == PGLite):
        stopServod()
    else:
        for i in range(4):
            pca9685_p3.stopServo(i)

def startServos():
    startServod()

def startServod():
    global ServosActive
    #print "Starting servod. ServosActove:", ServosActive
    SCRIPTPATH = os.path.split(os.path.realpath(__file__))[0]
    initString = "sudo " + SCRIPTPATH +'/servod --pcm --idle-timeout=20000 --p1pins="18,22" > /dev/null'
    os.system(initString)
    #print (initString)
    ServosActive = True

def pinServod(pin, degrees):
    #print pin, degrees
    pinString = "echo " + str(pin) + "=" + str(50+ ((90 - degrees) * 200 / 180)) + " > /dev/servoblaster"
    #print (pinString)
    os.system(pinString)

def stopServod():
    global ServosActive
    os.system("sudo pkill -f servod")
    ServosActive = False
