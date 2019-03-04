import serial
import time
from datetime import datetime

port = '/dev/ttyACM0'
baud = 115200

ser = serial.Serial(port, baud, timeout = 1)

while True:
    try:
        if(ser.isOpen()):
            line = ser.readline()
            print(line)
            #print(datetime.now().strftime('%Y/%m/%d %H:%M:%S.%f'))

    except serial.SerialException:
        print('serial exception')
        break

#Serial.flush()
print("End")
