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
def Follow_Line(testMode = False, intersectionQueue = [],intersectionSpeeds = [], robot = loadRobot('ROBOSON.json')):
    
    # Adjusting Robot variables
    IncreaseTime = False 
    firstInter = True
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
    post_line_width = robot.post_black_line_width * black_line_width

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
    
    # Used to defer the intersections if detected too soon
    lineFollowingStartTime = time.time()
    
    
    dontDetectIntersectionTime = robot.dont_detect_intersection_time

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
        numberOfLinesDetectingIntersection = 0


        if not intersectionMode:
            
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
                    
                    
                    # Checking to see if a for is detected 
                    if (lineWidthDetected >= fork_min_width and len(edgeIndices) >= 4):

                        intersectionDirection = intersectionQueue[0]
                        numberOfLinesDetectingIntersection += 1

                        if(intersectionDirection == "L"):
                            thisLineDeltaX = rightmostEdge - width / 2
                        elif(intersectionDirection == "R"):
                            thisLineDeltaX = leftmostEdge - width / 2
                        elif(intersectionDirection == "X"):
                            return intersectionQueue

                        deltaXList.append(thisLineDeltaX)
                        
                    elif (lineWidthDetected >= post_line_width):
                        intersectionDirection = intersectionQueue[0]
                        numberOfLinesDetectingIntersection += 1

                        if(intersectionDirection == "L"):
                            thisLineDeltaX = rightmostEdge - width / 2
                        elif(intersectionDirection == "R"):
                            thisLineDeltaX = leftmostEdge - width / 2
                        elif(intersectionDirection == "X"):
                            return intersectionQueue

                        deltaXList.append(thisLineDeltaX)
                     
                    else:
                        # Finding the denter of the line
                        lineCenterX = int((leftmostEdge + rightmostEdge) / 2)

                        # Draw circle for showing
                        # cv2.circle(frame, (lineCenterX, lineY), 2, (255,0,0), -1)

                        thisLineDeltaX = rightmostEdge - width / 2
                        deltaXList.append(thisLineDeltaX)

                # Case for when only one edge is detected
                elif(len(edgeIndices) == 1):
                    if dline[0] == 0:
                        onlyEdge = edgeIndices[-1]
                    if dline[-1] == 0:
                        onlyEdge = edgeIndices[0]

                    # Cases for when the edge is to the left or the right
                    if( onlyEdge >= width / 2):
                        lineCenterX = onlyEdge + black_line_width / 2
                    if( onlyEdge < width / 2):
                        lineCenterX = onlyEdge - black_line_width / 2
                    thisLineDeltaX = lineCenterX - width / 2
                    deltaXList.append(thisLineDeltaX)

                # Case for when we are offline 
                # We will exponentially increase the delta values to return the line
                else:
                    thisLineDeltaX = offlineExponential * (abs(previousDeltaX)) * (-1 if previousDeltaX < 0 else 1)
                    deltaXList.append(thisLineDeltaX)
            
            if  (IncreaseTime == False and time.time() - lineFollowingStartTime > robot.increase_speed_time):
                baseSpeed = robot.speed.ramp_speed
                IncreaseTime = True
            
            
            #going into intersection mode    
            if (numberOfLinesDetectingIntersection >= numberOfLinesRequiredForIntersectionMode and time.time() - lineFollowingStartTime > dontDetectIntersectionTime):
                if (firstInter):
                    baseSpeed = robot.speed.second_base
                    maxSpeed = robot.speed.second_max
                    minSpeed = robot.speed.second_min
                    MultiCoefficient = robot.pid.second_total
                    PCoefficient = robot.pid.second_pro
                    DCoefficient = robot.pid.second_der
                    offlineExponential = robot.pid.second_off_line
                    #PCoefficient -= 0.5
                    firstInter = False
                    print("time to get up the ramp: ",time.time() - dontDetectIntersectionTime)
                intersectionMode = True
                intersectionDirection = intersectionQueue.pop(0)
                intersectionSpeed = intersectionSpeeds.pop(0)
                thisTime = time.time() 
                goLeft = int(intersectionDirection == 'L')
                while (time.time() - thisTime < 0.01):
                    ser.write([intersectionSpeed,goLeft ,intersectionSpeed, int(not goLeft)])
                ser.write([int(intersectionSpeed/3),int(not goLeft),int(intersectionSpeed/3),goLeft])
                time.sleep(0.05)
                ser.write([0,0,0,0])
                time.sleep(0.5)       
                print("intersection mode timestamp", time.time())


        if intersectionMode:
            
             

            # Important !
            # Pay attention that in this loop now, the 
            # intersection direction is already determined
            # We must have poped it off in the previous iteration from
            # the intersectionQueue in order for the state of the machine 
            # to change to intersection mode

            # Also note,
            # if the intersection direction indicated "X", 
            # we should have quitted the program by now 
            # and there is no need to check 


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

                    # Checking to see if a for is detected 
                    if (lineWidthDetected >= fork_min_width):

                        numberOfLinesDetectingIntersection += 1

                        if(intersectionDirection == "L"):
                            thisLineDeltaX = rightmostEdge - width / 2
                        elif(intersectionDirection == "R"):
                            thisLineDeltaX = leftmostEdge - width / 2
                        elif(intersectionDirection == "X"):
                            return intersectionQueue

                        deltaXList.append(thisLineDeltaX)

                    else:
                        # Finding the denter of the line
                        lineCenterX = int((leftmostEdge + rightmostEdge) / 2)

                        # Draw circle for showing
                        # cv2.circle(frame, (lineCenterX, lineY), 2, (255,0,0), -1)

                        thisLineDeltaX = lineCenterX - width / 2
                        deltaXList.append(thisLineDeltaX)

                # Case for when only one edge is detected
                elif(len(edgeIndices) == 1):
                    
                    if dline[0] == 0:
                        onlyEdge = edgeIndices[-1]
                    if dline[-1] == 0:
                        onlyEdge = edgeIndices[0]


                    # Cases for when the edge is to the left or the right
                    if( onlyEdge >= width / 2):
                        lineCenterX = onlyEdge + black_line_width / 2
                    if( onlyEdge < width / 2):
                        lineCenterX = onlyEdge - black_line_width / 2
                    thisLineDeltaX = lineCenterX - width / 2
                    deltaXList.append(thisLineDeltaX)

                # Case for when we are offline 
                # We will exponentially increase the delta values to return the line
                else:
                    thisLineDeltaX = offlineExponential * (abs(previousDeltaX)) * (-1 if previousDeltaX < 0 else 1)
                    deltaXList.append(thisLineDeltaX)

            if (numberOfLinesDetectingIntersection == 0 ):#and time.time() - intersectionStartTime >= 0.3):
                print("exiting intersection mode!")
                intersectionMode = False


        # Takingn the median of the delta Xs
        currentDeltaX  = statistics.median(deltaXList)
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
        #print(duty_cycle_left, duty_cycle_right)

        # Reassigning the duty cycle values based on the cutoffs
        # Left Wheel
        if duty_cycle_left > maxSpeed:
            duty_cycle_left = maxSpeed
            #print("FUCK LEFT")
        elif duty_cycle_left < minSpeed:
            duty_cycle_left = minSpeed
            #print("FUCK LEFT")

        # Right wheel
        if duty_cycle_right > maxSpeed:
            duty_cycle_right = maxSpeed
            #print("FUCK RIGTH")
        elif duty_cycle_right < minSpeed:
            duty_cycle_right = minSpeed
            #print("FUCK RIGTH")

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
