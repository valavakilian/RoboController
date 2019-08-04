# Import libraries
import time
import numpy as np
import serial
from numpy import diff
import time
from RoboLoader import loadRobot
import RPi.GPIO as GPIO


def post_switch(robot):

    # Loading Robot as an object
    # Reads variables from the file
    poleContactPin = robot.pole_contact_pin

    # Setting the contact pole pin as a input pin
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(poleContactPin, GPIO.IN)
    

    # Wait for the proper pin to be turned on by the bluepill
    # indicating that the stone pickup is complete
    poleContactPinState = GPIO.input(poleContactPin)
    while(poleContactPinState is 0):
        
        #GPIO.output(dispenseStonePin, GPIO.LOW)
        poleContactPinState = GPIO.input(poleContactPin)
        print(poleContactPinState)
        
    
    # Cleaning the GPIO pins is required
    GPIO.cleanup()

    return True


robot = loadRobot('ROBOSON.json')
pickup_stone(robot)

