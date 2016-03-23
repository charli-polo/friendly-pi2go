# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO, smbus, time


class Lcd7segment:
    # TODO: le mot SPEED est trompeur....
    _ADDR = 0x20 # I2C address of MCP23017
    _SPEED = 0.005
    _LETTERS = {
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

        self.bus.write_byte_data(self._ADDR, 0x00, 0x00) # Set all of bank 0 to outputs
        self.bus.write_byte_data(self._ADDR, 0x01, 0x00) # Set all of bank 1 to outputs
        self.bus.write_byte_data(self._ADDR, 0x13, 0xff) # Set all of bank 1 to High (Off)

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
            self._sendDigit(self._LETTERS[d1], 0) # '1'
            time.sleep(self._SPEED)
            self._sendDigit(0, 0)
            self._sendDigit(self._LETTERS[d2], 1) # '2'
            time.sleep(self._SPEED)
            self._sendDigit(0, 1)
            self._sendDigit(self._LETTERS[d3], 2) # '3'
            time.sleep(self._SPEED)
            self._sendDigit(0, 2)
            self._sendDigit(self._LETTERS[d4], 3) # '0'
            time.sleep(self._SPEED)
            self._sendDigit(0, 3)
            count2 += 1

    def _sendDigit(self, digit, pos):
        """Send a digit (bit) to the pos position of the lcd."""
        t = (1<<pos) ^ 255
        self.bus.write_byte_data(self._ADDR, 0x13, t) # Set bank 1 Pos to Low
        self.bus.write_byte_data(self._ADDR, 0x12, digit) # Set bank 0 to digit

    def setLetter(self, digit, pos):
        """ Set a letter (or number) to one of the 4 digits of the LCD """
        self._sendDigit(self._LETTERS[str(digit)[0]], pos)
