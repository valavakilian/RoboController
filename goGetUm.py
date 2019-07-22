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

def Contact_Pole(angle = 0):
	
	# Loading Robot as an object
    # Reads variables from the file
    robot = loadRobot('ROBOSON.json')
    
    while 
