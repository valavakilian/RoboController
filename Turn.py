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
from servoTest import *


def Rotate(robot ,rotationPeriod , rotationSpeed , rotationDirection):
    
    if rotationDirection == "L":
        serialByteArray = [rotationSpeed, 1, rotationSpeed, 0]
    elif rotationDirection == "R":
        serialByteArray = [rotationSpeed, 0, rotationSpeed, 1]

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

    GPIO.cleanup()
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
    
    
def goStr8(robot ,goTime, speed):
    
    serialByteArray = [speed, 1, speed, 1]

    # Check serial port issues
    # if the connection is not extablished, send in the angle back
    try: 
        ser = serial.Serial("/dev/ttyS0", 9600)
        ser.flushInput()
        startTime = time.time()
        while(time.time() - startTime < goTime ):
            ser.write(serialByteArray)
    except serial.SerialException:
        print ("Execption")
        return False

    GPIO.cleanup()
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

def backUp(robot,goTime, speed):
    
    serialByteArray = [int(speed), 0, int(speed), 0]

    # Check serial port issues
    # if the connection is not extablished, send in the angle back
    try: 
        ser = serial.Serial("/dev/ttyS0", 9600)
        ser.flushInput()
        startTime = time.time()
        while(time.time() - startTime < goTime ):
            ser.write(serialByteArray)
    except serial.SerialException:
        print ("Execption")
        return False

    GPIO.cleanup()
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



def goToPost(robot, speed):
    
    serialByteArray = [speed, 1, speed, 1]

    # Loading Robot as an object
    # Reads variables from the file
    poleContactPin = robot.pole_contact_pin

    # Setting the contact pole pin as a input pin
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(poleContactPin, GPIO.IN)
    

    # Wait for the proper pin to be turned on by the bluepill
    # indicating that the stone pickup is complete
    poleContactPinState = GPIO.input(poleContactPin)
    
    
    # Check serial port issues
    # if the connection is not extablished, send in the angle back
    try: 
        ser = serial.Serial("/dev/ttyS0", 9600)
        ser.flushInput()
        startTime = time.time()
        while(poleContactPinState is 0):
            ser.write(serialByteArray)
            poleContactPinState = GPIO.input(poleContactPin)
            
    except serial.SerialException:
        print ("Execption")
        return False
    
    GPIO.cleanup()
    
    """# After we are done, we will simply send the zero value to the motors
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
        return False"""
    
    
    


def curveRotate(robot ,rotationPeriod , rotationSpeed , rotationDirection, backwards):
    
    serialByteArray = []
    if backwards == 1:
        if rotationDirection == "L":
            serialByteArray = [rotationSpeed, 0, 0, 1]
        elif rotationDirection == "R":
            serialByteArray = [0, 1, rotationSpeed, 0]
    elif backwards == 0:
        if rotationDirection == "L":
            serialByteArray = [rotationSpeed, 1, 0, 0]
        elif rotationDirection == "R":
            serialByteArray = [0, 0, rotationSpeed, 1]
    else:
        print("Curve Turn Error")

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

    GPIO.cleanup()
    # After we are done, we will simply send the zero value to the motors
    serialByteArray = []
    serialByteArray.append(0)
    serialByteArray.append(0)
    serialByteArray.append(0)
    serialByteArray.append(0)


    # Check serial port issues
    # if the connection is not extablished, send in the angle back
    try: 
        ser.write(serialByteArray)
    except serial.SerialException:
        return False


def customMove(robot, arr, rotationPeriod):
    serialByteArray = arr

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
    
#def ShakeUrHips(robot, , 4)
