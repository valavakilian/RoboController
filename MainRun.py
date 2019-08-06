from pullLaryAndRandy import Follow_Line
from RoboLoader import loadRobot
from Turn import *
from servoTest import *
from pickupStone import *


# Loading Robot as an object
# Reads variables from the file
robot = loadRobot('ROBOSON.json')
#val = Follow_Line(True, ["L","L","R","X"], [80, 40, 40, 0], robot)
#print(val)
changeCameraAngle(70)
val = Follow_Line(True, ["L","L","R", "R", "X"], [110, 50, 0, 0,0], robot)
print(val)
#goStr8(robot, 0.1, 60)
Rotate(robot ,1, 60,"L")
changeCameraAngle(0)
#goStr8(robot, 1, 100)
goToPost(robot, 100)
pickup_stone(robot)
backUp(robot, 0.5, 50)
Rotate(robot, 1 , 60, "L")
goToPost(robot, 100)
#Rotate(robot ,0.7, 60,"L")
#goStr8(robot, 4, 90)
