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


def Rotate(angle = 0):

    # Loading Robot as an object
    # Reads variables from the file
    angle = int(angle)
    robot = loadRobot('ROBOSON.json')
    wheelDiameter = robot.wheel_diameter / 2
    turningRadius = robot.wheel_to_wheel_distance / 2
    maxSpeed = robot.speed.max
    minSpeed = robot.speed.min
    rotationRPM = robot.rotation_rpm

    # The centimeter per second veolocity of the wheel at this rpm
    robotRotatingVelocity = 2 * np.pi * (wheelDiameter / 2) * rotationRPM

    # The distance the wheel travels during his rotation
    rotationDistance = np.pi * wheelDiameter * angle / 360

    # The amount of time the motors will be turned on 
    rotationPeriod = rotationDistance / robotRotatingVelocity


    # The signs for the left and right duty cycle
    duty_cycle_left_sign = np.sign(-1 * angle)
    duty_cycle_right_sign = np.sign(angle)

    # calculating the duty cycle from the required rpm
    duty_cycle = RPM_to_DutyCycle(rotationRPM)
    print(duty_cycle)
    serialByteArray = []
    # Serial communication to the BluePills
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
    # if the connection is not extablished, send in the angle back
    try: 
        ser = serial.Serial("/dev/ttyS0", 9600)
        ser.flushInput()
        startTime = time.time()
        while(time.time() - startTime < 0.2 ):
            ser.write(serialByteArray)
    except serial.SerialException:
        print ("Execption")
        return False

    # Sleep the time period required for the full rotation
    time.sleep(rotationPeriod)

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



def RPM_to_DutyCycle(rpm):
    dutycycle = rpm * 2
    return dutycycle

if __name__ == '__main__':
    Rotate(sys.argv[1])