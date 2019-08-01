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
import sys


def Rotate(rotationPeriod = 0, rotationSpeed = 0, rotationDirection):
    
    if rotationDirection == "L":
	    serialByteArray = [rotationSpeed, 1, rotationSpeed, 0]
	elif rotationDirection == "R":
		serialByteArray = [rotationSpeed, 1, rotationSpeed, 0]

    # Check serial port issues
    # if the connection is not extablished, send in the angle back
    try: 
        ser = serial.Serial("/dev/ttyS0", 9600)
        ser.flushInput()
        startTime = time.time()
        while(time.time() - startTime < rotationPeriod ):
            ser.write(serialByteArray)
    except serial.SerialException:
        print ("Execption")
        return False

    # After we are done, we will simply send the zero value to the motors
    serialByteArray = []
    serialByteArray.append(0)
    serialByteArray.append(1)
    serialByteArray.append(0)
    serialByteArray.append(1)


    # Check serial port issues
    # if the connection is not extablished, send in the angle back
    try: 
        ser.write(serialByteArray)
    except serial.SerialException:
        return False