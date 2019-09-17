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
def Follow_Line(testMode = False, intersectionQueue = [], robot = loadRobot('ROBOSON.json')):
    
    # Adjusting Robot variables
    baseSpeed = robot.speed.base
    maxSpeed = robot.speed.max
    minSpeed = robot.speed.min
    numberOfLinesRequiredForIntersectionMode = robot.number_of_lines_required_for_inersection_mode


    # Value used for the binary filter
    BIN_CUT = robot.line_finder.binary_cut

    # Loading PID values
    MultiCoefficient = robot.pid.total
    PCoefficient = robot.pid.pro
    DCoefficient = robot.pid.der
    offlineExponential = robot.pid.off_line


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


    # Intersection mode determins the process of the line following
    intersectionMode = False
    # This value changes based on the last element in the intersection queue
    intersectionDirection = None
    # Number of lines detectecing a intersection where the left adn right 
    # spikes are more than a single line
    numberOfLinesDetectingIntersection = 0

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
        frameCopy = frameArray.copy()[:, :, 0]
        frameCopyTime = time.time()
        #print("transform time", frameCopyTime - algorithmStartTime)
        
        
        # Blurring the image
        blurredImage = cv2.GaussianBlur(frameCopy,(5,5),0)
        blurTime = time.time()
        #print("blur time: ",blurTime-frameCopyTime)
        
        
        # Taking the kernel of the image
        #kerneledImage = cv2.filter2D(frameCopy, -1, kernel)[:, :, 0]
        #kernelTime = time.time()
        #print("kernel time: ",kernelTime-frameCopyTime)
        
        testFrame = blurredImage.copy()[linesY]
        centerXs = []
        centerXs = [np.argmin(testFrameRow) for testFrameRow in testFrame ]
    
        
        for yIndex in range(len(linesY)):
            print((centerXs[yIndex], linesY[yIndex]))
            print("____________________")
            cv2.circle(frameArray, (centerXs[yIndex], linesY[yIndex]), 2, (0,0,255), thickness=1)





        # Median center value
        # This value is a cumulative results from all the lines of sensors
        currentDeltaX = 0

        # list of deltaXs
        # A list of the deltaXs indicated by each line
        deltaXList = []

        # We zero this value to increment it in the for loop to find out
        # if the number of sensor lines detecting intersection goes to zero 
        # in which case we must quit the intersection mode
        numberOfLinesDetectingIntersection = 0


        if not intersectionMode:
            deltaXList = centerXs
            currentDeltaX = statistics.mean(deltaXList) - width / 2
                    

            if (numberOfLinesDetectingIntersection >= numberOfLinesRequiredForIntersectionMode):
                intersectionMode = True
                intersectionDirection = intersectionQueue.pop(0)
                intersectionStartTime = time.time()
                print("enter intersection mode motherfucker at ! ", time.time())


        if intersectionMode:
            

            if (numberOfLinesDetectingIntersection == 0 and time.time() - intersectionStartTime >= 1):
                intersectionMode = False


        # Takingn the median of the delta Xs
        #currentDeltaX  = statistics.median(deltaXList)
        #print("Delta X measured: ", currentDeltaX)

        forLoopTime = time.time()
        #print("for loop time: ", forLoopTime - binTime)


        #
        # PID calculations
        #

        # Finding the derivative time 
        # The time it took from taking the last frame 
        derivativeDeltaTime = frameTakingTime + (forLoopTime - algorithmStartTime)

        # finding the derivative
        dXdT = (currentDeltaX - previousDeltaX) / derivativeDeltaTime

        # Finding the error value given the constants
        error_value = int(MultiCoefficient * (DCoefficient * dXdT + PCoefficient * currentDeltaX))

        duty_cycle_left = baseSpeed + error_value
        duty_cycle_right = baseSpeed - error_value


        # Reassigning the duty cycle values based on the cutoffs
        # Left Wheel
        if duty_cycle_left > maxSpeed:
            duty_cycle_left = maxSpeed
        elif duty_cycle_left < minSpeed:
            duty_cycle_left = minSpeed

        # Right wheel
        if duty_cycle_right > maxSpeed:
            duty_cycle_right = maxSpeed
        elif duty_cycle_right < minSpeed:
            duty_cycle_right = minSpeed

        calcTime = time.time()
        #print("Calculation time: ", calcTime - forLoopTime)


        # Serial communication to the BluePill
        serialByteArray.append(abs(duty_cycle_left))
        if(duty_cycle_left > 0):
            serialByteArray.append(1)
        else:
            serialByteArray.append(0)
        
        serialByteArray.append(abs(duty_cycle_right))
        if(duty_cycle_right > 0):
            serialByteArray.append(1)
        else:
            serialByteArray.append(0)
        
        #print("Byte array sent to the BluePill: ",serialByteArray)
        
        # This try cathc block will return the unfinished
        # queue in case the serial communication is lost in the middle
        try:
            ser.write(serialByteArray)
            serialByteArray = []
        except serial.SerialException: 
            return intersectionQueue
        
       
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
            #cv2.imshow("binary", binaryImage)

        # Truncating reqiured for the frames
        rawCapture.truncate(0)

        # End of this frame 
        frameEndTime = time.time()




# Loading Robot as an object
# Reads variables from the file
robot = loadRobot('ROBOSON.json')
val = Follow_Line(True, ["L","L","R","X"], robot)
print(val)

