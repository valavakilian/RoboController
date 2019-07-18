import smbus
import time
import os

I2C_ADDRESS = 0x08
bus = smbus.SMBus(1)


#bus.write_byte(I2C_ADDRESS, 0xFF)

for i in range(0,1000):
    try:
        bus.write_byte(I2C_ADDRESS, 0x0F)
    except IOError:
        os.system("i2cdetect -y 1")
    
        
    value=bus.read_byte(I2C_ADDRESS)
    print("READ DATA IS ", value, i)
    time.sleep(0.1)
