import pullLarry


# Loading Robot as an object
# Reads variables from the file
robot = loadRobot('ROBOSON.json')
val = Follow_Line(True, ["L","L","R","L", "X"], [80, 50, 0, 40, 0], robot)
print(val)