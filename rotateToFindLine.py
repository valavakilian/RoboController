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


"""
Function defenition
"""
def moveToFindLine(testMode = False, moveArray, movetime , robot = loadRobot('ROBOSON.json')):
    

    serialByteArray = moveArray

    # Adjusting Robot variables
    numberOfLinesRequiredForIntersectionMode = robot.number_of_lines_required_for_inersection_mode


    # Value used for the binary filter
    BIN_CUT = robot.line_finder.binary_cut

    blackLineWidth = robot.line_width


    camera = PiCamera()
    camera.color_effects = (128, 128)
    cameraResolution = robot.line_finder.resolution
    camera.resolution = (cameraResolution[0], cameraResolution[1])
    rawCapture = PiRGBArray(camera, size = (cameraResolution[0], cameraResolution[1]))


    # Check serial port issues
    # if there is no serial connection, this function
    # simply returned the untouched queue
    try: 
        ser = serial.Serial("/dev/ttyS0", 9600)
        ser.flushInput()
        serialByteArray = []
    except serial.SerialException: 
        return intersectionQueue

    # Changinig the mode to showing the frames or not 
    if testMode:
        cv2.namedWindow("Original_frame", cv2.WINDOW_NORMAL)
        cv2.namedWindow("binary", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("binary" , 100,100)
        cv2.resizeWindow("Original_frame", 100,100)

    # Reads width of the line from the file
    with open('LineWidth.txt') as f:
        black_line_width = int(f.readline())


    # Minimum width for a fork 
    fork_min_width = robot.fork_black_line_min_width_multiplier * black_line_width
    post_line_width = robot.post_black_line_width * black_line_width

    numberOfLinesRequireToFindBlackLine = robot.number_of_lines_require_to_find_black_line

    # Kernel setup
    n = 3
    kernel = np.array([[0,1/n,0]*n])


    # Camera initialization and start
    camera.capture(rawCapture, format = 'bgr')
    frame = rawCapture.array[:, :, 0]
    rawCapture.truncate(0)

    # Retrieving the height and the with of the frame
    height = len(frame)
    width = len(frame[0])

    # Sensor lines used for the line follower 
    # You could change the distances
    linesY = np.linspace(2 / 3 * height - 5, height - 10 , 5, dtype = int)#np.linspace(20, height-5, 5, dtype = int)#

    # Required by the camera function to runcate every time
    rawCapture.truncate(0)

    # List of errors and times for PID plots
    if testMode:
        errorList = []
        timeList = []

    # Initializing distances for derivative calculation
    currentDeltaX = 0
    previousDeltaX = 0

    # Initializing times
    currentTime = time.time()
    frameEndTime = time.time()
    frameStartTime = time.time()


    # Number of lines detectecing a intersection where the left adn right 
    # spikes are more than a single line
    numberOfLinesDetectingBlackLine = 0
    
    # Used to defer the intersections if detected too soon
    movementStartTime = time.time()


    # There are two ways to exit this main loop
    # 1) The serial connection is lost
    # 2) queue of the turns has reached character "X"

    # Main for loop starting 
    # Frame is taken as a 3 channgel grayscaled image
    for frame in camera.capture_continuous(rawCapture, format = "bgr", use_video_port = True):

        # Algorithm start time
        algorithmStartTime = time.time()

        # Sample frame taken time
        frameStartTime = time.time()
        frameTakingTime = frameStartTime - frameEndTime
        #print("Frame taking time:", frameTakingTime)

        # Appending time to list
        timeList.append(frameStartTime)

        # Assignet previous delta X used for the derivative 
        previousDeltaX = currentDeltaX

        frameArray = frame.array
        # Creating a grayscale copy of the frame by taking only one of the channels
        # and only including the lines indicated for the line follwoing algorithm
        # This step is included to reduce the computer time 
        frameCopy = frameArray.copy()[linesY]
        frameCopyTime = time.time()
        #print("transform time", frameCopyTime - algorithmStartTime)

        # Taking the kernel of the image
        kerneledImage = cv2.filter2D(frameCopy, -1, kernel)[:, :, 0]
        kernelTime = time.time()
        #print("kernel time: ",kernelTime-frameCopyTime)

        # Blurring the image
        blurredImage = cv2.GaussianBlur(kerneledImage,(5,5),0)
        blurTime = time.time()
        #print("blur time: ",blurTime-kernelTime)

        # Binary filter for the image
        ret, binaryImage = cv2.threshold(blurredImage, BIN_CUT, 255, cv2.THRESH_BINARY) 
        binTime = time.time()
        #print("bin time: ",binTime-blurTime)

        # Creating the matrix 
        pathMatrix = binaryImage

        # Median center value
        # This value is a cumulative results from all the lines of sensors
        currentDeltaX = 0

        # list of deltaXs
        # A list of the deltaXs indicated by each line
        deltaXList = []

        # We zero this value to increment it in the for loop to find out
        # if the number of sensor lines detecting intersection goes to zero 
        # in which case we must quit the intersection mode
        numberOfLinesDetectingBlackLine = 0
        

        # A loop for the calculations for each line of the sensors
        for line_index in range(0, len(linesY)):
            lineY = linesY[line_index]
            line = pathMatrix[line_index]

            # taking the derivative of the lines
            dline = diff(line)
            
            if line[0] == 0:
                dline[0] = 1
            if line[-1] == 0:
                dline[-1] = 1

            # Here we can either choose the two heighest points
            # or we could simple trust that there will only be 2 values
            # This will indicate the two edges of the line
            #top_two_indices = sorted(range(len(dline)), key = lambda i: dline[i])[-2:]
            edgeIndices = [ i for i in range(0, len(dline)) if dline[i] != 0] 
            
            # Case for when two or more edges are detected         
            if (len(edgeIndices) >= 2):

                leftmostEdge = edgeIndices[0]
                rightmostEdge = edgeIndices[-1]

                lineWidthDetected = rightmostEdge - leftmostEdge
                #print("Line width detected: ",lineWidthDetected)
                #print(len(edgeIndices))
                #print("_______________________________________________")

                if (lineWidthDetected >= blackLineWidth)

                numberOfLinesDetectingBlackLine += 1
        
        

        forLoopTime = time.time()
        #print("for loop time: ", forLoopTime - binTime)

        
        if (numberOfLinesDetectingIntersection >= numberOfLinesRequiredForIntersectionMode):
            try:
                ser.write([0,0,0,0])
                return True
            except serial.SerialException: 
                print("Serial Exception")
                return time.time() - movementStartTime
        elif (time.time() - movementStartTime >= movetime):
            try:
                ser.write([0,0,0,0])
            except serial.SerialException: 
                print("Serial Exception")
                return time.time() - movementStartTime
        else:
            try:
                ser.write(serialByteArray)
            except serial.SerialException: 
                print("Serial Exception")
                return time.time() - movementStartTime
        
        #print("Byte array sent to the BluePill: ",serialByteArray)
        
        # This try cathc block will return the unfinished
        # queue in case the serial communication is lost in the middle
        
        
       
        serialTime = time.time()
        #print("serial time: ", serialTime - calcTime)
        

        # Loop is now complete
        key = cv2.waitKey(1)

        # if the `q` key was pressed, break from the loop
        if key == "q":
            break
        
        algorithmEndTime = time.time()
        #print("Full algorithm time: ", algorithmEndTime - algorithmStartTime)

        # Showing the frame in test mode only 
        if testMode:
            cv2.imshow("Original_frame", frameArray)
            cv2.imshow("binary", binaryImage)

        # Truncating reqiured for the frames
        rawCapture.truncate(0)

        # End of this frame 
        frameEndTime = time.time()




# Loading Robot as an object
# Reads variables from the file
if __name__ == '__main__':
    robot = loadRobot('ROBOSON.json')
    val = Follow_Line(True, ["L","L","R","X"], [80, 50, 10, 80, 0],robot)
    print(val)


