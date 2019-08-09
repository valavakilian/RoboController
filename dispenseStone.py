# Import libraries
import time
import numpy as np
import serial
from numpy import diff
import time
from RoboLoader import loadRobot
import RPi.GPIO as GPIO


def dispense_stone(robot):

    # Loading Robot as an object
    # Reads variables from the file
    startStonePickupPin = robot.start_pickup_stone_pin
    objectiveCompletePin = robot.objective_complete_pin
    dispenseStonePin = robot.dispense_stone_pin

    # Setting the contact pole pin as a input pin
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(startStonePickupPin, GPIO.OUT)
    GPIO.setup(dispenseStonePin, GPIO.OUT)
    GPIO.setup(objectiveCompletePin, GPIO.IN)
    
    

    # Bluepill starts the pickup stone process as this pin turns high
    # Only needs to be high for a second
    GPIO.output(startStonePickupPin, GPIO.LOW)
    GPIO.output(dispenseStonePin, GPIO.LOW)
    GPIO.output(dispenseStonePin, GPIO.HIGH)
    print("Pin on")
    time.sleep(1)
    GPIO.output(dispenseStonePin, GPIO.LOW)
    #GPIO.output(dispenseStonePin, GPIO.LOW)

    # Wait for the proper pin to be turned on by the bluepill
    # indicating that the stone pickup is complete
    time.sleep(5)
    stonePickupState = GPIO.input(objectiveCompletePin)
    while(stonePickupState is 1):
        print(stonePickupState)
        #GPIO.output(dispenseStonePin, GPIO.LOW)
        stonePickupState = GPIO.input(objectiveCompletePin)
    
    # Cleaning the GPIO pins is required
    GPIO.cleanup()

    return True

if __name__ == '__main__':
    robot = loadRobot('ROBOSON.json')
    dispense_stone(robot)
