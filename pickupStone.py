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
    poleContactDutycycle = robot.pole_contact_dutycycle

    # Setting the contact pole pin as a input pin
    GPIO.setmode(GPIO.BCM)
	GPIO.setup(poleContactPin, GPIO.IN)