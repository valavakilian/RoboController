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


def backingUp(motorRuntime):
	
	# Loading Robot as an object
    # Reads variables from the file
    robot = loadRobot('ROBOSON.json')
    backingupDutyCycle = robot.backing_up_dutycycle

	# Preparing the array information sent to the bluepill
    serialByteArray.append(backingupDutyCycle)
    serialByteArray.append(0)
    serialByteArray.append(backingupDutyCycle)
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

	# Motors will run for this period of time  
	time.sleep(motorRuntime)

	# sending the value of zero to the motors in order to stop them
    serialByteArray.append(0)
    serialByteArray.append(0)
    serialByteArray.append(0)
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
	
    return timePeriod


