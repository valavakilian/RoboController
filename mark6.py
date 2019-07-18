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
from RoboLoader import RoboLoader

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
robot = RoboLoader('ROBOSON.json')


with open('LineWidth.txt') as f:
    black_line_width = int(f.readline())

#black_line_width = 5
currentDeltaX = 0
previousDeltaX = 0

baseSpeed = robot.speed.base
maxSpeed = robot.speed.max
minSpeed = robot.speed.min

BIN_CUT = robot.line_finder.binary_cut

ser = serial.Serial("/dev/ttyUSB0", 9600)
ser.flushInput()


byteArray = []

#ser.write(bytes ([BPmin]))
#ser.write(bytes ([BPmax]))

cv2.namedWindow("Original_frame", cv2.WINDOW_NORMAL)
cv2.namedWindow("binary", cv2.WINDOW_NORMAL)
cv2.resizeWindow("binary" , 100,100)
cv2.resizeWindow("Original_frame", 100,100)
          

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



linesY = np.linspace(int(2 * height / 3),height - 10, 5, dtype = int)

rawCapture.truncate(0)


errorList = []
timeList = []


currentTime = time.time()
frameEndTime = time.time()

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
    transformTime = time.time()
    #print("transform time", transformTime - thisTime)
    
    #cv2.imshow("Original_frame", frame)

    #gray = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2GRAY)
    grayTime = time.time()
    #print("gray time: ",grayTime - previousTime)
    kerneled_image = cv2.filter2D(frame_copy, -1, kernel)
    kernelTime = time.time()
    #print("kernel time: ",kernelTime-grayTime)
    blur = cv2.GaussianBlur(kerneled_image,(5,5),0)
    blurTime = time.time()
    #print("blur time: ",blurTime-kernelTime)
    ret, binary = cv2.threshold(kerneled_image, BIN_CUT, 255, cv2.THRESH_BINARY) 
    binTime = time.time()
    #print("bin time: ",binTime-blurTime)
    
    #print(kerneled_image[-1])
    pathMatrix = binary
    line_index = 0
    
    
    for line_index in range(0, len(linesY)):
        lineY = linesY[line_index]
        line = pathMatrix[line_index]
        dline = diff(line)

        #print(dline)
        top_two_indices = sorted(range(len(dline)), key = lambda i: dline[i])[-2:]
        top_indices = [ i for i in range(0, len(dline)) if dline[i] != 0] 
        
        
        
        if (len(top_indices) >= 2):
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
            currentDeltaX = 1.4 * (abs(previousDeltaX))*(-1 if previousDeltaX < 0 else 1)

    forLoopTime = time.time()
    #print("for loop time", forLoopTime - binTime)

    rawCapture.truncate(0)
    

    currentTime = time.time()
    deltaTime = currentTime - previousTime
    dXdT = (currentDeltaX - previousDeltaX) /deltaTime
    #print(currentDeltaX)
    
    

    cv2.imshow("Original_frame", frame)
    #cv2.imshow("gray_frame", gray)
    #cv2.imshow("blur",blur)
    cv2.imshow("binary", binary)

    #if (abs(abs(rightmostEdge - leftmostEdge) - black_line_width) < 100):

    error_value = int(MultiCoefficient * (DCoefficient * dXdT + PCoefficient * currentDeltaX))
    
    
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
    
    key = cv2.waitKey(1) #& 0xFF
    #print("key" + str(key))

    # if the `q` key was pressed, break from the loop
    if key == "q":
        break
    
    #print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
    endTime = time.time()
    print("Full time: ", endTime - firstTime)

    frameEndTime = time.time()
 
# otherwise, release the file pointer
cv2.release()
 
# close all windows
cv2.destroyAllWindows()
