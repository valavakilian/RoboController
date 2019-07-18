import time
import GPIO
MOTOR1PWM1PIN = 18
MOTOR1PWM2PIN = 12
GPIO.setmode(GPIO.BOARD)
GPIO.setup(MOTOR1PWM1PIN, GPIO.OUT)
GPIO.setup(MOTOR1PWM2PIN, GPIO.OUT)


motor1p1 = GPIO.PWM(MOTOR1PWM1PIN, 0.5)
motor1p2 = GPIO.PWM(MOTOR1PWM2PIN, 0.5)

motor1p1.start(1)
motor1p1.start(1)
time.sleep(10)
motor1p1.stop()
motor1p1.stop()
GPIO.cleanup()
