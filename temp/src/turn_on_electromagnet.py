#! /usr/bin/python
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

# GPIO pin number
magnet_pin = 21

# set magnet_pin as an output
GPIO.setup(magnet_pin, GPIO.OUT)

# turn GPIO pin on
GPIO.output(magnet_pin, True)

while(1):
    keypressed = raw_input('Press q to quit: ')

    if keypressed == 'q':
        # turn GPIO pin off
        GPIO.output(magnet_pin, False)

        # sets any GPIO used back to input mode
        GPIO.cleanup()
        break
