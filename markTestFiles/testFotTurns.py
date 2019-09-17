from pullLarryAndRandySmartTurn import Follow_Line
from Turn import *
robot = loadRobot('ROBOSON.json')
changeCameraAngle(70)
val = Follow_Line(True, ["L","L", "R", "X"], [100, 100, 0,0], robot)
backUp(robot,0.5, 80)
curveRotate(robot ,2, 100,"L", 0)
changeCameraAngle(70)
val = Follow_Line(True, ["X"], [0], robot)

