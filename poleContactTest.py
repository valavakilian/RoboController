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
import serial




def Contact_Pole(angle = 0):
	
	# Loading Robot as an object
    # Reads variables from the file
    robot = loadRobot('ROBOSON.json')
    poleContactPin = robot.pole_contact_pin
    poleContactDutycycle = robot.pole_contact_dutycycle

    # Setting the contact pole pin as a input pin
    GPIO.setmode(GPIO.BCM)
	GPIO.setup(poleContactPin, GPIO.IN)

	# We initially read the GPIO pin
	contactState = GPIO.input(poleContactPin)

	# These two times will be used to find the period 
	# when the motor was on
	motorStartTime = 0 
	motorEndTime = 0

	# The main program will start if the pin is not pushed 
	# meaing we are not in contact with the pole
	if (not contactState):

		# Preparing the array information sent to the bluepill
	    serialByteArray.append(poleContactDutycycle)
	    serialByteArray.append(1)
	    serialByteArray.append(poleContactDutycycle)
	    serialByteArray.append(1)

	    motorStartTime = time.time()


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


		# sending the value of zero to the motors in order to stop them
	    serialByteArray.append(0)
	    serialByteArray.append(1)
	    serialByteArray.append(0)
	    serialByteArray.append(1)

	    motorEndTime = time.time()


		# Check serial port issues
	    # If the connection is not extablished, send in the angle back
	    # Otherwise start the motor
	    try: 
	        ser = serial.Serial("/dev/ttyUSB0", 9600)
	        ser.flushInput()
	        ser.write(serialByteArray)
	    except serial.SerialException:
	        return False
    	
    	# Finiding the on time for the motors and returning them
    	timePeriod = motorEndTime - motorStartTime
	    return timePeriod

    else:
    	return 0

    
