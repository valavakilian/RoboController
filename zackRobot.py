import sys
import time
import io
import cv2
import numpy as np
import os
import stat
import serial 
from picamera.array import PiRGBArray
from picamera import PiCamera
from numpy import diff
from imutils.video import VideoStream
from imutils.video import FPS
import argparse
import imutils
import time

'''
This function is created as a test to see if we can control the robot like a remote control car
All for fun
'''

ser = serial.Serial("/dev/ttyUSB0", 9600)
ser.flushInput()

ser.write(bytes ([0]))
ser.write(bytes ([255]))

while (True):
    x = int(input())
    ser.write(bytes([x]))
    time.sleep(1.1)
    
