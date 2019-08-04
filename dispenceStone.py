# Import libraries
import time
import numpy as np
import serial
from numpy import diff
import time
from RoboLoader import loadRobot
import RPi.GPIO as GPIO


def pickup_stone(robot):

    # Loading Robot as an object
    # Reads variables from the file
    pickupStoneCompletePin = robot.objective_complete_pin
    dispenseStonePin = robot.dispense_stone_pin

    # Setting the contact pole pin as a input pin
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(dispenseStonePin, GPIO.OUT)
    GPIO.setup(pickupStoneCompletePin, GPIO.IN)
    

    # Bluepill starts the pickup stone process as this pin turns high
    # Only needs to be high for a second
    GPIO.output(dispenseStonePin, GPIO.LOW)
    GPIO.output(dispenseStonePin, GPIO.HIGH)
    print("Pin on")
    time.sleep(1)
    GPIO.output(dispenseStonePin, GPIO.LOW)

    # Wait for the proper pin to be turned on by the bluepill
    # indicating that the stone pickup is complete
    dispenseStoneState = GPIO.input(dispenseStonePin)
    while(dispenseStoneState is 0):
        print(dispenseStoneState)
        #GPIO.output(dispenseStonePin, GPIO.LOW)
        dispenseStoneState = GPIO.input(dispenseStonePin)
    
    # Cleaning the GPIO pins is required
    GPIO.cleanup()

    return True


robot = loadRobot('ROBOSON.json')
pickup_stone(robot)

