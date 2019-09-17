#!usr/bin/bash python

'''
This function is used for loading and saving the robot from the json file.
'''

import json
from collections import namedtuple

'''
Used for loading the robot using the file name.
'''
def loadRobot(filename = 'ROBOSON.json'):
    with open(filename) as f:
        data = f.read()
# Parse JSON into an object with attributes corresponding to dict keys.
    return json.loads(data, object_hook=lambda d: namedtuple('Robot', d.keys())(*d.values()))

'''
Used for saving the new robot values intp the given file name.
'''
def dumpRobot(robot, filename):
    with open(filename, 'w') as f:
        json.dump(robot, f)
