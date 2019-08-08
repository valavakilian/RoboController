from pullLaryAndRandy import Follow_Line
#from pullLarryAndRandySmartTurn import Follow_Line
from RoboLoader import loadRobot
from Turn import *
from servoTest import *
from pickupStone import *
import sys

robot = loadRobot('ROBOSON.json')

if (len(sys.argv) == 1):
    print("Specify side")
    quit()

if (sys.argv[1] == '-l'):
    changeCameraAngle(70)
    val = Follow_Line(True, ["L","L","R", "R", "X"], [70, 60, 0, 0,0], [0.3,0.2,0.01,0.01,0.01], [0, 0.5,0.8, 1.1, 0], robot)
    #print(val)
    #changeCameraAngle(0)
    backUp(robot, 0.3, 70)
    Rotate(robot ,1, 60,"L")
    goToPost(robot, 150)
    pickup_stone(robot)
    print("Quitting first function")
    #curveRotate(robot, 0.1, 255, "R", 1)
    backUp(robot, 0.01,140)
    backUp(robot , 0.2, 70)
    print("Quitting backup")
    Rotate(robot, 1 , 60, "L")
    goToPost(robot, 100)
    pickup_stone(robot)
    Rotate(robot ,0.7, 60,"L")
    goStr8(robot, 4, 90)

elif (sys.argv[1] == "-r"):
    changeCameraAngle(70)
     # Loading Robot as an object
    # Reads variables from the file
    #changeCameraAngle(70)
    val = Follow_Line(True, ["R","R","L", "L", "X"], [70, 70, 0, 0,0], [0.3,0.2,0.01,0.01,0.01], [0, 0.5,0.8, 1.1, 0], robot)
    #print(val)
    #changeCameraAngle(0)
    #backUp(robot, 0.3, 70)
    #Rotate(robot ,1, 60,"L")
    goToPost(robot, 150)
    pickup_stone(robot)
    print("Quitting first function")
    #curveRotate(robot, 0.1, 255, "R", 1)
    backUp(robot, 0.01,140)
    backUp(robot , 0.2, 70)
    print("Quitting backup")
    Rotate(robot, 1 , 60, "L")
    goToPost(robot, 100)
    pickup_stone(robot)
    Rotate(robot ,0.7, 60,"L")
    goStr8(robot, 4, 90)