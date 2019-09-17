#import picamera
#import picamera.array
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
import statistics


robot = loadRobot("ROBOSON.json")
picamera.color_effects = (128, 128)
cameraResolution = robot.line_finder.resolution
picamera.resolution = (cameraResolution[0], cameraResolution[1])
rawCapture = PiRGBArray(picamera, size = (cameraResolution[0], cameraResolution[1]))
    

cv2.namedWindow("Original_frame", cv2.WINDOW_NORMAL)
cv2.namedWindow("binary", cv2.WINDOW_NORMAL)
cv2.resizeWindow("binary" , 100,100)
cv2.resizeWindow("Original_frame", 100,100)


# Camera initialization and start
picamera.capture(rawCapture, format = 'bgr')
frame = rawCapture.array[:, :, 0]
rawCapture.truncate(0)

# Retrieving the height and the with of the frame
height = len(frame)
width = len(frame[0])

# Sensor lines used for the line follower 
# You could change the distances
linesY = np.linspace(int(2 * height / 3),height - 10, 5, dtype = int)

# Required by the camera function to runcate every time
rawCapture.truncate(0)

    
with picamera.PiCamera() as camera:
    camera.resolution = (640, 480)
    with picamera.array.PiMotionArray(camera) as output:
        camera.start_recording('/dev/null', format = 'h264', motion_output = output)
        camera.wait_recording(1)
        camera.stop_recording()
        print("captured %d frames" %output.array.shape[0])
        print("frames is ",  (output.array)) 