from youreWiferHasUglyShoesAndWeHaveTwoPIDSnickhassexwithjackdeng import Follow_Line 
from RoboLoader import loadRobot
from Turn import *
from servoTest import *


# Loading Robot as an object
# Reads variables from the file
robot = loadRobot('ROBOSON.json')
#val = Follow_Line(True, ["L","L","R","X"], [80, 40, 40, 0], robot)
#print(val)
changeCameraAngle(60)
val = Follow_Line(True, ["L","L", "R", "X"], [80, 40, 0, 0], robot)
#print(val)
goStr8(robot, 0.1, 60)
Rotate(robot ,0.4, 100,"L")
changeCameraAngle(0)
goToPost(robot, 130)
#Rotate(robot ,0.7, 60,"L")
#goStr8(robot, 4, 90)
