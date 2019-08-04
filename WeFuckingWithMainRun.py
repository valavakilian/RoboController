from youreWiferHasUglyShoesAndWeHaveTwoPIDSnickhassexwithjackdeng import Follow_Line 
from RoboLoader import loadRobot
from Turn import *
from servoTest import *
from pickupStone import pickup_stone


# Loading Robot as an object
# Reads variables from the file
robot = loadRobot('ROBOSON.json')
#val = Follow_Line(True, ["L","L","R","X"], [80, 40, 40, 0], robot)
#print(val)
changeCameraAngle(70)
#val = Follow_Line(True, ["L","L", "R", "X"], [50, 100, 20, 0], robot)
val = Follow_Line(True, ["R","R", "L", "X"], [0, 0, 20, 0], robot)
changeCameraAngle(0)
#print(val)
goStr8(robot, 0.05, 90)
Rotate(robot ,0.15, 110,"L")
#changeCameraAngle(0)
goToPost(robot, 130)
pickup_stone(robot)
#Rotate(robot ,0.7, 60,"L")
#goStr8(robot, 4, 90)
