from pullLaryAndRandy import Follow_Line
#from pullLarryAndRandySmartTurn import Follow_Line
from RoboLoader import loadRobot
from Turn import *
from servoTest import *
from pickupStone import *
from rotateToFindLine import *
from UpTopLineFollow import Follow_Line_Up_Top
import time


# Loading Robot as an object
# Reads variables from the file
robot = loadRobot('ROBOSON.json')

camera = PiCamera()
camera.color_effects = (128, 128)
cameraResolution = robot.line_finder.resolution
camera.resolution = (cameraResolution[0], cameraResolution[1])
changeCameraAngle(70)

val = Follow_Line(camera, True, ["L","L","R", "R", "X"], [65, 60, 0, 0,0], [0.4,0.01,0.01,0.01,0.01], [0, 0.5,0.8, 1.1, 0], robot)
print(val)
changeCameraAngle(0)
backUp(robot, 0.3, 70)
Rotate(robot ,0.6, 60,"L")
goToPost(robot, 110)
pickup_stone(robot)
#time.sleep(2)
#print("Quitting first function")
backUp(robot, 0.01,155)
backUp(robot , 0.25, 70)
backUp(robot, 0.01 ,0)
#print("Quitting backup")
Rotate(robot, 0.5 , 60, "L")
goToPost(robot, 80)
pickup_stone(robot)
#backUp(robot, 0.01, 155)
#customMove(robot, [155, 1, 155 , 1], 0.01)
#customMove(robot, [60, 0 , 100 , 0], 1.5)
backUp(robot ,2, 70)
Rotate(robot, 0.4 , 60, "L")
changeCameraAngle(70)
camera = moveToFindLine(camera ,True, [110, 1, 90, 0], 5 , robot = loadRobot('ROBOSON.json'))
#goStr8(robot, 4, 90)

val = Follow_Line_Up_Top(camera, True, ["L", "R", "X"], [60 ,60, 0], [0.1 ,0.1, 0], [0, 1, 0], robot)
Rotate(robot, 1.5 , 70, "L")
backUp(robot, 1.5, 60)
#camera = moveToFindLine(camera ,True, [65, 1, 65, 0], 5 , robot = loadRobot('ROBOSON.json'))
val = Follow_Line_Up_Top(camera, True, ["X"], [0], [0], [100], robot)
#customMove(robot, [70, 1, 70 , 1], 0.01)
#Rotate(robot, 1 , 70, "L")
#print(val)
#help, im stuck in this pi!!

#customMove(robot, [135 , 1, 100, 1], 4)
customMove(robot, [100, 0 , 100 , 0], 0.1)
customMove(robot, [100, 0 , 100 , 0], 0.1)
