# friendly-pi2go
Some tools/notes for my pi2go robot

## To Do ##
- Use https (test for perf) on Jupyter ?
- make the libs python3 compatible

## Done ##
- call back example

## Not done ##
might be an issue ?
- sudo nano /etc/modules and add “i2c-dev” as the last line

## Libraries to consider ##
- RPI.GPIO
- RPIO
- gpiocrust
- https://github.com/adafruit/Adafruit_Python_GPIO/blob/master/Adafruit_GPIO/GPIO.py

## Stuff to do for my module ##
- Handle clic, long clic, double clic (it's enough...)
- Handle simultaneous action trigerred
- extend this to other sensor ?
- Multithread for sonar event callback ? (distance < x)
- option to print stuff (event detected, functions called) for debug
- object oriented style
  - robot.forward
  - robot.step_forward
  - robot.leds.front.light
  - robot.leds.light
  - robot.say

## stuff to program ##
- scan the room
- follow a color
- programm some lights show with the button
- tirage au sort (roue)
- menu sur le lcd
