# Import libraries
import time
import numpy as np
import serial
from numpy import diff
import time
from RoboLoader import loadRobot
import RPi.GPIO as GPIO


def pickup_stone():

	# Loading Robot as an object
    # Reads variables from the file
    robot = loadRobot('ROBOSON.json')
   	startStonePickupPin = robot.start_pickup_stone_pin
    pickupStoneCompletePin = robot.pickup_stone_complete_pin
    bluepillSafetyPin = robot.bluepill_safety_pin

    # Setting the contact pole pin as a input pin
    GPIO.setmode(GPIO.BCM)
	GPIO.setup(startStonePickupPin, GPIO.OUT)
	GPIO.setup(bluepillSafetyPin, GPIO.OUT)
	GPIO.setup(pickupStoneCompletePin, GPIO.IN)

    # This is a safety pin to ensure the bluepill only does action if this pin is turned on
    GPIO.ouput(bluepillSafetyPin, 1)

	# Bluepill starts the pickup stone process as this pin turns high
	# Only needs to be high for a second
	GPIO.ouput(poleContactPin, 1)
	time.sleep(0.1)
	GPIO.ouput(poleContactPin, 0)

	# Wait for the proper pin to be turned on by the bluepill
	# indicating that the stone pickup is complete
	stonePickupState = GPIO.input(startStonePickupPin)
	while(not stonePickupState):
		stonePickupState = GPIO.input(startStonePickupPin)


	# Now the program is complete, we can turn the safety pin off and exit
	GPIO.ouput(bluepillSafetyPin, 0)

	return True



