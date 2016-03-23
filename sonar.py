# -*- coding: utf-8 -*-

import time, RPi.GPIO as GPIO

class Sonar:
    def __init__(self):
        self.channel = 8

    def getDistance(self):
        """Return distance with sonar"""
        GPIO.setup(self.channel, GPIO.OUT)
        # Send 10us pulse to trigger
        GPIO.output(self.channel, True)
        time.sleep(0.00001)
        GPIO.output(self.channel, False)
        start = time.time()
        count=time.time()
        GPIO.setup(self.channel,GPIO.IN)
        while GPIO.input(self.channel)==0 and time.time()-count<0.1:
            start = time.time()
        count=time.time()
        stop=count
        while GPIO.input(self.channel)==1 and time.time()-count<0.1:
            stop = time.time()
        # Calculate pulse length
        elapsed = stop-start
        # Distance pulse travelled in that time is time
        # multiplied by the speed of sound 34000(cm/s) divided by 2
        distance = elapsed * 17000
        return distance
