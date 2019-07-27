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
#import matplotlib.plot as plt
from imutils.video import VideoStream
from imutils.video import FPS
import argparse
import imutils
import time

from RoboLoader import loadRobot, dumpRobot

def updateLine(filename = 'ROBOSON.json'):
        robot = loadRobot(filename)

        camera = PiCamera()
        camera.resolution = robot.line_finder.resolution
        #camera.framerate = 32
        rawCapture = PiRGBArray(camera, size = robot.line_finder.resolution)
        camera.capture(rawCapture, format = 'bgr')
        frame = rawCapture.array
        rawCapture.truncate(0)
        kernel = np.array(robot.line_finder.kernel)

        for frame in camera.capture_continuous(rawCapture, format = "bgr", use_video_port = True):
            frame = frame.array
            frame_copy = frame.copy()
            gray = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2GRAY)
            kerneled_image = cv2.filter2D(gray, -1, kernel)
            ret, binary = cv2.threshold(kerneled_image, robot.line_finder.binary_cut, 255, cv2.THRESH_BINARY)
            captureBin = binary
            capture = frame_copy
            cv2.imshow('Binary', binary)
            rawCapture.truncate(0)
            if (cv2.waitKey(33)) == ord('a'):
                    break
        
        
        


        cv2.imshow('Binary', captureBin)
        cv2.waitKey(0)
        lineWidth= int(np.mean([sum(255-i)/255 for i in captureBin]))
        print (lineWidth)
        robot.line_finder.line_width = lineWidth
        dumpRobot(robot, fillename)

if __name__ == "__main__":
        updateLine(sys.argv[1])


    

