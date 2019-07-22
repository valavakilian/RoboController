# Import libraries
import time
import cv2
import numpy as np
import serial
import csv
from picamera.array import PiRGBArray
from picamera import PiCamera
from numpy import diff
import time
from RoboLoader import loadRobot
import RPi.GPIO as GPIO




def Contact_Pole(angle = 0):
	
	# Loading Robot as an object
    # Reads variables from the file
    robot = loadRobot('ROBOSON.json')
    GPIO.setmode(GPIO.BCM)
    poleContactPin = robot.pole_contact_pin
	GPIO.setup(poleContactPin, GPIO.IN)

	# We initially read the GPIO pin
	contactState = GPIO.input(poleContactPin)

	# The main program will start if the pin is not pushed 
	# meaing we are not in contact with the pole
	if (not contactState):

		# Preparing the array information sent to the bluepill
	    serialByteArray.append(abs(duty_cycle))
	    if(angle > 0):
	        serialByteArray.append(1)
	    else:
	        serialByteArray.append(0)
	    
	    serialByteArray.append(abs(duty_cycle))
	    if(angle < 0):
	        serialByteArray.append(1)
	    else:
	        serialByteArray.append(0)

		# Check serial port issues
	    # If the connection is not extablished, send in the angle back
	    # Otherwise start the motor
	    try: 
	        ser = serial.Serial("/dev/ttyUSB0", 9600)
	        ser.flushInput()
	        ser.write(serialByteArray)
	    except serial.SerialException:
	        return False
		while(not contactState):

			contactState = GPIO.input(poleContactPin)
    
    
