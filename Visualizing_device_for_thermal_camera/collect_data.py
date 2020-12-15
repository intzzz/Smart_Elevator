#!/usr/bin/env python
import time
import serial
from picamera import PiCamera
from time import sleep
import sys

camera = PiCamera()

encoding = 'utf-8'

ser = serial.Serial( ## Configure UART
 port='/dev/ttyACM0',
 baudrate = 115200,
 parity=serial.PARITY_NONE,
 stopbits=serial.STOPBITS_ONE,
 bytesize=serial.EIGHTBITS,
 timeout=1
)

time.sleep(2)
File = open("data/serial_log.txt","a") ## Open folder file for data writing
y = ser.readline()
print(y)
print("Empty first reading buffer") ## Empty first read buffer
i = 0
ser.flushInput()
while True: ## As soon as data is received from thermal camera through STM32, read it
 x = ser.readline() ## Read one line
 y = sys.getsizeof(x) ## Check if it has all the data
 if(y == 5137):
     camera.capture('data/' + str(i) + '_PICTURE.jpg') ## Take and save camera image to folder
     z = x.decode(encoding)
     File.write(z) ## Write temperature data to folder
     i = i + 1 ## Increase to save images with different names
 print (i)
