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

Kernel_size = 15
low_threshold = 40
high_threshold = 120

rho = 10
threshold = 15
theta = np.pi / 180
minLineLength = 10
maxLineLength = 1

#cap = cv2.VideoCapture(0)
#ret, frame = cap.read()

black_line_width = 236

currentDeltaX = 0
previousDeltaX = 0


ser = serial.Serial("/dev/ttyUSB0", 9600)
ser.flushInput()


BaseSpeed = int(128)
DCoefficient = 0.05
PCoefficient = 0.4
MultiCoefficient = 1
adjustment = 0


camera = PiCamera()
camera.resolution = (160, 120)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size = (160, 120))



camera.capture(rawCapture, format = 'bgr')
frame = rawCapture.array

print(frame)

height = len(frame)
width = len(frame[0])



linesY = np.linspace(10,int(height / 3), 5, dtype = int)

rawCapture.truncate(0)


currentTime = time.time()

for frame in camera.capture_continuous(rawCapture, format = "bgr", use_video_port = True):
    
    previousTime = currentTime
    previousDeltaX = currentDeltaX

    frame = frame.array
    frame_copy = frame.copy()[linesY]
    


    #for lineY in linesY:
    #    cv2.line(frame,(0,lineY),(width,lineY),[255,0,0])


    gray = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2GRAY)
    grayTime = time.time()
    print("gray time: ",grayTime - previousTime)
    blur = cv2.GaussianBlur(gray,(5,5),0)
    blurTime = time.time()
    print("blur time: ",blurTime-grayTime)
    ret, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY) 
    binTime = time.time()
    print("bin time: ",binTime-grayTime)
    
    
    pathMatrix = binary
    line_index = 0
    
    
    for line_index in range(0, len(linesY)):
        lineY = linesY[line_index]
        line = pathMatrix[line_index]
        dline = diff(line)

        #print(dline)
        #top_two_indices = sorted(range(len(dline)), key = lambda i: dline[i])[-2:]
        top_indices = [ i for i in range(0, len(dline)) if dline[i] != 0] 
        
        
        
        #print(top_indices)
        if (len(top_indices) >= 2):
            leftmostEdge = top_indices[0]
            rightmostEdge = top_indices[-1]
            #print(leftmostEdge - rightmostEdge)
            centerX = int((leftmostEdge + rightmostEdge) / 2)
            cv2.circle(frame, (centerX, lineY), 2, (255,0,0), -1)

        elif(len(top_indices) == 1):
            onlyEdge = top_indices[0]
            if( onlyEdge >= width / 2):
                centerX = onlyEdge + black_line_width / 2
            if( onlyEdge < width / 2):
                centerX = onlyEdge - black_line_width / 2
        else:
            centerX = 0

    forLoopTime = time.time()
    print("for loop time", forLoopTime - binTime)

    rawCapture.truncate(0)

    #print(centerX)
    

    
    currentDeltaX = centerX - width / 2
    currentTime = time.time()
    deltaTime = currentTime - previousTime
    dXdT = (currentDeltaX - previousDeltaX)/deltaTime
    #print(currentDeltaX, dXdT)
    
    
    #input()
    
    
    

    cv2.imshow("Original_frame", frame)
    #cv2.imshow("gray_frame", gray)
    #cv2.imshow("blur",blur)
    #cv2.imshow("binary", binary)
    
    error_value = int(MultiCoefficient * (DCoefficient * dXdT + PCoefficient * currentDeltaX))
    
    
    duty_cycle = 128 + error_value
    
    
    if duty_cycle > 255:
        duty_cycle = 255
    elif duty_cycle < 0:
        duty_cycle = 0
    
    calcTime = time.time()
    print("calculation time: ", calcTime - forLoopTime)
    
    #print(duty_cycle)
    
    output = bytes ([duty_cycle])
    print(output)
    print(duty_cycle)
    print(ser.write(output))
    while ser.in_waiting:
        print("read")
        print(ser.readline(1))
    #time.sleep(0.01)
    
    serialTime = time.time()
    print("serial time: ", serialTime - forLoopTime)
    
    key = cv2.waitKey(1) #& 0xFF
    print("key" + str(key))

    # if the `q` key was pressed, break from the loop
    if key == "q":
        break
    
    print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
    endTime = time.time()
    print("Full time: ", endTime - previousTime)
 
# otherwise, release the file pointer
cv2.release()
 
# close all windows
cv2.destroyAllWindows()



