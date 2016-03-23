# -*- coding: utf-8 -*-
from __future__ import print_function
import pca9685_p3

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


class Led(object):
    _MAX_LED_VALUE = 4096

    def __init__(self, LED):
        # TODO: _red doit-il contenir le pourcentage ou la valeur entière ?
        self._LED = LED
        self._red = 0
        self._green = 0
        self._blue = 0

    @classmethod
    def _integer_to_percentage(cls, n_value):
        """ transform an integer into percentage """
        n = n_value % cls._MAX_LED_VALUE
        perc = n/(cls._MAX_LED_VALUE-1)
        return round(perc ,2)

    @classmethod
    def _percentage_to_integer(cls, percentage):
        """ transform a percentage into an integer < 4096 """
        n = int(percentage * (cls._MAX_LED_VALUE-1))
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
