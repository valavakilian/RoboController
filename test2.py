import sys
import time
import cv2
import numpy as np
import os
import stat
import serial



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

cap = cv2.VideoCapture(0)
ret, frame = cap.read()

black_line_width = 236

currentDeltaX = 0
previousDeltaX = 0


ser = serial.Serial("/dev/video0", 9600)
ser.flushInput()


BaseSpeed = int(128)
DCoefficient = 0.05
PCoefficient = 0.4
MultiCoefficient = 1
adjustment = 0




currentTime = time.time()
while ret:
    #print(os.system("ls /dev/ttyU*"))
    previousTime = currentTime
    previousDeltaX = currentDeltaX

    ret, frame = cap.read()
    
    print(type(frame))
    height = len(frame)
    width = len(frame[0])

    frame_copy = frame.copy()

    linesY = np.linspace(10,int(height / 3), 5, dtype = int)

    for lineY in linesY:
        cv2.line(frame,(0,lineY),(width,lineY),[255,0,0])


    gray = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray,(5,5),0)
    ret, binary = cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY) 

    
    pathMatrix = []
    line_index = 0
    for lineY in linesY:
        pathMatrix.append([])
        for x in range(0, width):
            pathMatrix[line_index].append(binary[lineY][x])
        line_index += 1

    
    
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
            cv2.circle(frame, (centerX, lineY), 25, (255,0,0), -1)

        elif(len(top_indices) == 1):
            onlyEdge = top_indices[0]
            if( onlyEdge >= width / 2):
                centerX = onlyEdge + black_line_width / 2
            if( onlyEdge < width / 2):
                centerX = onlyEdge - black_line_width / 2
        else:
            centerX = 0

        #print(centerX)
        

    
    currentDeltaX = centerX - width / 2
    currentTime = time.time()
    deltaTime = currentTime - previousTime
    dXdT = (currentDeltaX - previousDeltaX)/deltaTime
    #print(currentDeltaX, dXdT)
    
    print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
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
    
    print(duty_cycle)
    
    output = bytes ([duty_cycle])
    print(ser.write(output))
    #time.sleep(0.01)
    
    key = cv2.waitKey(1) & 0xFF

    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
 
# otherwise, release the file pointer
cv2.release()
 
# close all windows
cv2.destroyAllWindows()




