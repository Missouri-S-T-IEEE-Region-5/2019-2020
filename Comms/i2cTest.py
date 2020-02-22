import smbus
import RPi.GPIO as GPIO
import time

bus = smbus.SMBus(1)

DEVICE_ADDRESS = 0x15
DEVICE_REG_LEDOUT0 = 0x1d

#GPIO.input(4)
GPIO.setmode(GPIO.BCM)
GPIO.setup(4, GPIO.IN)

while 1:
    while (GPIO.input(4) == False):
        time.sleep(0.1)
        
    ledA_timeOn = 0x01
    ledA_timeOff = 0x02
    ledA_loop = 0x03
    ledA_output = [ledA_timeOn, ledA_timeOff, ledA_loop]

    bus.write_i2c_block_data(DEVICE_ADDRESS, DEVICE_REG_LEDOUT0, ledA_output)
    
    time.sleep(2)