import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

SLEEP_TIME = 0.005

class byte_sender():

    def __init__(self, data, clk, start):
        self.data = data
        self.clk = clk
        self.start = start
        GPIO.setwarnings(False)
        GPIO.setup(data,GPIO.OUT)
        GPIO.setup(clk,GPIO.OUT)
        GPIO.output(clk,GPIO.HIGH) 
        GPIO.setup(start,GPIO.OUT)
        GPIO.output(start,GPIO.LOW) 
        
    def send_byte(self, byt, sign):
        byt = str(sign) + byt
        #print(byt)
        
        GPIO.output(self.clk,GPIO.HIGH)
        GPIO.output(self.start,GPIO.HIGH) 
        
        for i in byt:
            if(i == "1"):
                GPIO.output(self.data,GPIO.HIGH)
                print("1")
            elif(i == "0"):
                GPIO.output(self.data,GPIO.LOW)
                print("0")
            GPIO.output(self.clk,GPIO.LOW)
            time.sleep(SLEEP_TIME)
            GPIO.output(self.clk,GPIO.HIGH)

            
        
        GPIO.setup(self.data, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        input_flag = GPIO.input(self.data)
        while(not input_flag):
            continue
        
        GPIO.setup(self.data,GPIO.OUT)
        
        GPIO.output(self.start,GPIO.LOW)
        GPIO.output(self.clk,GPIO.HIGH)

    def send_number(self, number):
        if(number < 0):
            sign = 1
            bin_num = bin(number)[3:].zfill(7)
        else:
            sign = 0
            bin_num = bin(number)[2:].zfill(7)

        self.send_byte(bin_num,sign)
        
