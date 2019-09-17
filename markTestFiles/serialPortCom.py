import serial
import RPi.GPIO as GPIO
import time

ser=serial.Serial("/dev/ttyUSB0",9600)
ser.baudrate=9600


while True:
    write_success=ser.write(100)
    read_line = ser.readline()
    time.sleep(0.1)
    
    #value = ser.readLine()
    #if(read_ser=="Hello From Arduino!"):
     #   blink(11)
    #print(write_success)
    print(read_line)
