import RPi.GPIO as GPIO
import time
import sys

def changeCameraAngle(angle):
    dutyCycle = angle / 18 + 3
    servoPIN = 17
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(servoPIN, GPIO.OUT)
    p = GPIO.PWM(servoPIN, 50)
    p.start(2.5)

    try:
        p.ChangeDutyCycle(3)
        time.sleep(1)
        p.ChangeDutyCycle(dutyCycle)
        time.sleep(1)    
    except KeyboardInterrupt:
        print("HERE")
        p.stop()
        GPIO.cleanup()
        return
    
    return


#angle = int(input("Enter the desired angle: "))
#changeCameraAngle(angle)
