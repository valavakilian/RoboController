#!usr/bin/bash python

import json
from collections import namedtuple

def loadRobot(filename = 'ROBOSON.json'):
    with open(filename) as f:
        data = f.read()
# Parse JSON into an object with attributes corresponding to dict keys.
    return json.loads(data, object_hook=lambda d: namedtuple('Robot', d.keys())(*d.values()))

def dumpRobot(robot, filename):
    with open(filename, 'w') as f:
        json.dump(robot, f)
