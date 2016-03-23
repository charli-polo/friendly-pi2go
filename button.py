# -*- coding: utf-8 -*-

from __future__ import print_function
from board import Board
import RPi.GPIO as GPIO, time

class Push(object):
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
        if self.robot._board.PGType == 1:
            self.channel = Board.SWITCH
        elif self.robot._board.PGType == 2:
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
