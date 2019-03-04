#Library import
import serial
import time
import FaBo9Axis_MPU9250
from datetime import datetime
from ctypes import *

#Connection
port = '/dev/ttyACM0'
baud = 115200
_timeout = 1
packetSize = 53

#Data Packet
class dataPacket(Structure):
    field = [('currentTime', c_uint32), ('aCoord', c_float * 3), ('gCoord', c_float * 3), ('mCoord', c_float * 3)]

data = dataPacket()


#Port
while True:
    try:
         ser = serial.Serial(port, baud, timeout = _timeout) #connect
         ser.flushInput()
         print(datetime.now().strftime('%Y/%m/%d %H:%M:%S.%f') + 'Connection Established')
       
    except ser.SerialException:
        time.sleep(0.05)


#Communication
while True:
    try:
        currentTime = data.currentTime
        aCoord = list(data.aCoord)
        gCoord = list(data.gCoord)
        mCoord = list(data.mCoord)

        print(datetime.now().strftime('%Y/%m/%d %H:%M:%S.%f') + ',' + str(aCoord[0]) + ',' +  str(aCoord[1]) + ',' + str(aCoord[2])
    
   # except ser.SerialException:
   #     ser.write('serial exception')

        
#ser.close()
        print("End")
