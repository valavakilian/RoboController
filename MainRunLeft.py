from pullLaryAndRandy import Follow_Line
#from pullLarryAndRandySmartTurn import Follow_Line
from RoboLoader import loadRobot
from Turn import *
from servoTest import *
from pickupStone import *
import sys
from rotateToFindLine import *
from UpTopLineFollow import Follow_Line_Up_Top
import time
from dispenseStone import *


robot = loadRobot('ROBOSONleft.json')

camera = PiCamera()
camera.color_effects = (128, 128)
cameraResolution = robot.line_finder.resolution
camera.resolution = (cameraResolution[0], cameraResolution[1])

changeCameraAngle(70)

input("BIG SEND LEFT")

val = Follow_Line(camera, True, ["R","R", "L", "L", "X"], [60, 60, 0, 0, 0], [0.8, 0.1, 0, 2, 0], [0, 0.1, 1, 1.5, 0], robot)
print(val)
changeCameraAngle(0)
backUp(robot, 0.5, 70)
Rotate(robot ,0.18, 60,"R")
goToPost(robot, 110)
pickup_stone(robot)
#time.sleep(2)



#print("Quitting first function")
backUp(robot, 0.01,155)
#this is rotate
customMove(robot, [90, 0 , 70 , 0], 0.133)
#backUp(robot , 0.5, 70)
backUp(robot, 0.01 ,0)
#print("Quitting backup")


Rotate(robot, 0.05 , 60, "R")
goToPost(robot, 80)
pickup_stone(robot)
#backUp(robot, 0.01, 155)
#customMove(robot, [155, 1, 155 , 1], 0.01)
#customMove(robot, [60, 0 , 100 , 0], 1.5)


# this is a curved backup instead of rotate
customMove(robot, [90, 0 , 70 , 0], 1)
#backUp(robot ,2, 70)
#Rotate(robot, 0.5 , 60, "R")

changeCameraAngle(70)
camera = moveToFindLine(camera ,True, [70, 1, 65, 1], 5 , robot = loadRobot('ROBOSON.json'))
customMove(robot, [70, 1 , 70 , 1], 0.5)
Rotate(robot, 0.3 , 60, "R")
camera = moveToFindLine(camera ,True, [70, 1, 70, 0], 10 , robot = loadRobot('ROBOSON.json'))
#goStr8(robot, 4, 90)
#Rotate(robot, 0.25 , 60, "L")

val = Follow_Line_Up_Top(camera, 100,  True, ["L", "X"], [70, 0], [0.1, 0], [0.5, 1], robot)

Rotate(robot, 1 , 70, "R")
#camera = moveToFindLine(camera ,True, [70, 0, 70, 1], 10 , robot = loadRobot('ROBOSON.json'))
backUp(robot, 0.3, 60)
#camera = moveToFindLine(camera ,True, [65, 1, 65, 0], 5 , robot = loadRobot('ROBOSON.json'))
val = Follow_Line_Up_Top(camera, 1, True, ["X"], [0], [0], [100], robot)
#customMove(robot, [70, 1, 70 , 1], 0.01)
#Rotate(robot, 1 , 70, "L")
#print(val)
#help, im stuck in this pi!!

#customMove(robot, [135 , 1, 100, 1], 4)
customMove(robot, [80, 0 , 80 , 0], 0.01)
customMove(robot, [80, 0 , 80 , 0], 0.01)
val = Follow_Line_Up_Top(camera, 4, True, ["X"], [0], [0], [100], robot)
#customMove(robot, [80, 0 , 80 , 0], 0.01)
#customMove(robot, [80, 0 , 80 , 0], 0.01)
#val = Follow_Line_Up_Top(camera, True, ["X"], [0], [0], [100], robot)
customMove(robot, [100, 1 , 100 , 1], 1)
customMove(robot, [20, 0 , 150 , 1], 1)
customMove(robot, [80, 0 , 80 , 0], 0.02)
customMove(robot, [80, 0 , 80 , 0], 0.02)
time.sleep(1)
val = Follow_Line_Up_Top(camera, 4, True, ["X"], [0], [0], [100], robot)
customMove(robot, [80, 0 , 80 , 0], 0.02)
customMove(robot, [80, 0 , 80 , 0], 0.02)
time.sleep(1)
val = Follow_Line_Up_Top(camera, 4, True, ["X"], [0], [0], [100], robot)
time.sleep(1)
customMove(robot, [70, 1 , 70 , 1], 1)
dispense_stone(robot)
customMove(robot, [70, 1 , 70 , 1], 1)
customMove(robot, [60, 0 , 60 , 0], 0.01)
customMove(robot, [60, 1 , 60 , 1], 0.01)
customMove(robot, [60, 0 , 60 , 0], 0.01)
customMove(robot, [60, 1 , 60 , 1], 0.01)
#ShakeUrHips(robot, 60, 4)


