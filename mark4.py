import sys
import time
import io
import cv2
import numpy as np
import os
import stat
import serial
import csv

from picamera.array import PiRGBArray
from picamera import PiCamera



from numpy import diff
#import matplotlib.plot as plt

from imutils.video import VideoStream
from imutils.video import FPS
import argparse
import imutils
import time
'''
Kernel_size = 15
low_threshold = 40
high_threshold = 120

rho = 10
threshold = 15
theta = np.pi / 180
minLineLength = 10
maxLineLength = 1
'''
#cap = cv2.VideoCapture(0)
#ret, frame = cap.read()

with open('LineWidth.txt') as f:
    black_line_width = int(f.readline())

#black_line_width = 5
currentDeltaX = 0
previousDeltaX = 0

baseSpeed = 150
maxSpeed = 200
minSpeed = -100

BIN_CUT = 50

ser = serial.Serial("/dev/ttyUSB1", 9600)
ser.flushInput()


byteArray = []

errorList = []
timeList = []

#ser.write(bytes ([BPmin]))
#ser.write(bytes ([BPmax]))

cv2.namedWindow("Original_frame", cv2.WINDOW_NORMAL)
cv2.namedWindow("binary", cv2.WINDOW_NORMAL)
cv2.resizeWindow("binary" , 100,100)
cv2.resizeWindow("Original_frame", 100,100)
cv2.namedWindow("Canny", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Canny" , 100,100)
          

def getfloat(filename):
    return float(filename.readline())
BaseSpeed = int(128)
with open('coeff.txt') as f:
    MultiCoefficient = getfloat(f)
    PCoefficient = getfloat(f)
    DCoefficient = getfloat(f)

adjustment = 0


camera = PiCamera()
camera.color_effects = (128, 128)
camera.resolution = (40, 32)
#camera.framerate = 32
rawCapture = PiRGBArray(camera, size = (40, 32))



camera.capture(rawCapture, format = 'bgr')
frame = rawCapture.array[:, :, 0]

height = len(frame)
width = len(frame[0])
n = 3

kernel = np.array([[0,1/n,0]*n])



linesY = np.linspace(int(5 * height / 9),height - 10, 10, dtype = int)

rawCapture.truncate(0)




currentTime = time.time()
frameEndTime = time.time()


forkDetected = False

for frame in camera.capture_continuous(rawCapture, format = "bgr", use_video_port = True):
    
    frameStartTime = time.time()

    timeList.append(frameStartTime)
    
    #print("frame taking time", frameStartTime - frameEndTime)
    firstTime = time.time()
    previousTime = currentTime
    previousDeltaX = currentDeltaX
    thisTime = time.time() 

    frame = frame.array
    frame_copy = frame.copy()[linesY][:, :, 0]
    edge = cv2.Canny(frame_copy,100,200)
    edgeTime = time.time()
    print("transform time", edgeTime - thisTime)
    

    #gray = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2GRAY)
    grayTime = time.time()
    #print("gray time: ",grayTime - previousTime)
    #kerneled_image = cv2.filter2D(frame_copy, -1, kernel)
    kernelTime = time.time()
    #print("kernel time: ",kernelTime-grayTime)
    #blur = cv2.GaussianBlur(kerneled_image,(5,5),0)
    blurTime = time.time()
    #print("blur time: ",blurTime-kernelTime)
    #ret, binary = cv2.threshold(kerneled_image, BIN_CUT, 255, cv2.THRESH_BINARY) 
    binTime = time.time()
    #print("bin time: ",binTime-blurTime)
    
    #print(kerneled_image[-1])
    #pathMatrix = binary
    pathMatrix = edge
    line_index = 0
    
    
    for line_index in range(0, len(linesY)):
        lineY = linesY[line_index]
        line = pathMatrix[line_index]
        #dline = diff(line)

        #top_two_indices = sorted(range(len(dline)), key = lambda i: dline[i])
        top_indices = [ i for i in range(0, len(line)) if line[i] != 0]        
        #print("AYA FUCK YA")
        #print(len(top_indices))
        if (len(top_indices) == 2):
            leftmostEdge = top_indices[0]
            rightmostEdge = top_indices[-1]
            #print("line width",leftmostEdge - rightmostEdge) 
            centerX = int((leftmostEdge + rightmostEdge) / 2)
            #cv2.circle(frame, (centerX, lineY), 2, (255,0,0), -1)
            currentDeltaX = centerX - width / 2

        elif(len(top_indices) == 1):
            onlyEdge = top_indices[0]
            if( onlyEdge >= width / 2):
                centerX = onlyEdge + black_line_width / 2
            if( onlyEdge < width / 2):
                centerX = onlyEdge - black_line_width / 2
            currentDeltaX = centerX - width / 2
        else:
            currentDeltaX = 1.01 * (abs(previousDeltaX))*(-1 if previousDeltaX < 0 else 1)

    if(forkDetected):
        break
    
    forLoopTime = time.time()
    #print("for loop time", forLoopTime - binTime)

    rawCapture.truncate(0)
    

    currentTime = time.time()
    deltaTime = currentTime - previousTime
    dXdT = (currentDeltaX - previousDeltaX) /deltaTime
    #print(currentDeltaX)
    
    

    #cv2.imshow("Original_frame", frame)
    #cv2.imshow("gray_frame", gray)
    #cv2.imshow("blur",blur)
    #cv2.imshow("binary", binary)
    cv2.imshow("Canny", edge)

    #if (abs(abs(rightmostEdge - leftmostEdge) - black_line_width) < 100):

    error_value = int(MultiCoefficient * (DCoefficient * dXdT + PCoefficient * currentDeltaX))
    

    errorList.append(error_value)    
    duty_cycle_left = baseSpeed + error_value
    duty_cycle_right = baseSpeed -1 * error_value
    
    
    
    
    if duty_cycle_left > maxSpeed:
        duty_cycle_left = maxSpeed
    elif duty_cycle_left < minSpeed:
        duty_cycle_left = minSpeed

    if duty_cycle_right > maxSpeed:
        duty_cycle_right = maxSpeed
    elif duty_cycle_right < minSpeed:
        duty_cycle_right = minSpeed
    
    calcTime = time.time()
    #print("calculation time: ", calcTime - forLoopTime)
    
    byteArray.append(abs(duty_cycle_left))
    if(duty_cycle_left > 0):
        byteArray.append(1)
    else:
        byteArray.append(0)
    
    byteArray.append(abs(duty_cycle_right))
    if(duty_cycle_right > 0):
        byteArray.append(1)
    else:
        byteArray.append(0)
    
    print(byteArray)
    ser.write(byteArray)
    byteArray = []
    
    #print(output)
    #print(duty_cycle)
    #ser.write(output)
    #while ser.in_waiting:
        #print("read")
        #ser.readline(1)

    
    serialTime = time.time()
    #print("serial time: ", serialTime - forLoopTime)
    
    key = cv2.waitKey(32) 
    #print("key" + str(key))

    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
    
    #print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
    endTime = time.time()
    print("Full time: ", endTime - firstTime)

    frameEndTime = time.time()


with open("error.csv", "w", newline = '') as errorFile:
    errorWriter = csv.writer(errorFile)
    errorWriter.writerows(errorList)

with open("times.csv", "w", newline = '') as timeFile:
    timesWriter = csv.writer(timeFile)
    timesWriter.writerows(timeList)




 
# close all windows
cv2.destroyAllWindows()
