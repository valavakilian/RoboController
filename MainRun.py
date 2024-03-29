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


robot = loadRobot('ROBOSON.json')

camera = PiCamera()
camera.color_effects = (128, 128)
cameraResolution = robot.line_finder.resolution
camera.resolution = (cameraResolution[0], cameraResolution[1])


changeCameraAngle(70)
input("SEND IT ")

val = Follow_Line(camera, True, ["L","L","R", "X"], [60, 60, 0, 0], [0.7, 0.1, 2, 0], [0, 0.1, 4, 0], robot)
print(val)
changeCameraAngle(0)
backUp(robot, 0.2, 70)
Rotate(robot ,0.4, 60,"L")
goToPost(robot, 110)
pickup_stone(robot)
#time.sleep(2)
#print("Quitting first function")
backUp(robot, 0.01,155)
backUp(robot , 0.1, 70)
backUp(robot, 0.01 ,0)
#print("Quitting backup")
Rotate(robot, 0.2 , 60, "L")
goToPost(robot, 80)
pickup_stone(robot)
#backUp(robot, 0.01, 155)
#customMove(robot, [155, 1, 155 , 1], 0.01)
#customMove(robot, [60, 0 , 100 , 0], 1.5)


backUp(robot ,1.1, 70)
Rotate(robot, 0.55 , 60, "L")
changeCameraAngle(70)
camera = moveToFindLine(camera ,True, [75, 1, 65, 1], 5 , robot = loadRobot('ROBOSON.json'))
Rotate(robot, 0.3 , 60, "L")
camera = moveToFindLine(camera ,True, [70, 1, 70, 0], 10 , robot = loadRobot('ROBOSON.json'))
#goStr8(robot, 4, 90)
#Rotate(robot, 0.25 , 60, "L")

val = Follow_Line_Up_Top(camera, 100,  True, ["R", "X"], [70, 0], [0.1, 0], [0.5, 1], robot)

Rotate(robot, 1 , 70, "L")
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
customMove(robot, [150, 1 , 20 , 0], 1)
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
customMove(robot, [70, 1 , 20 , 1], 1)
dispense_stone(robot)
customMove(robot, [70, 1 , 70 , 1], 1)
customMove(robot, [60, 0 , 60 , 0], 0.01)
customMove(robot, [60, 1 , 60 , 1], 0.01)
customMove(robot, [60, 0 , 60 , 0], 0.01)
customMove(robot, [60, 1 , 60 , 1], 0.01)
#ShakeUrHips(robot, 60, 4)

