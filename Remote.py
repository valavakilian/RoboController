import sys, termios, tty, os, time, serial
from pickupStone import pickup_stone
from dispenseStone import dispense_stone
from RoboLoader import loadRobot

robot = loadRobot('ROBOSON.json')
 
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
 
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch
 
button_delay = 0.1

try: 
    ser = serial.Serial("/dev/ttyS0", 9600)
    ser.flushInput()
    serialByteArray = []
except serial.SerialException:
    pass
 
while True:
    char = getch()
    if (char == 'w'):
        ser.write([255,1,255,1])
    if (char == 'a'):
        ser.write([255,1,255,0])
    if (char == 'd'):
        ser.write([255,0,255,1])
    if (char == 's'):
        ser.write([255,0,255,0])
    if (char == 'x'):
        ser.write([0,1,0,1])
    if (char == 'p'):
        quit()
    if (char == 'q'):
        dispense_stone(robot)
    if (char == 'e'):
        pickup_stone(robot)
        


