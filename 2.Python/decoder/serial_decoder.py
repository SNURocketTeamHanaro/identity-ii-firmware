###############################################################################
# Filename       : serial_decoder.py
# Target Machine : Arduino Mega with GY91 sensor module
#
# Description    : 
#        Decoder of serial input from arduino implementing "Avionics.ino", with
#        default output format of CSV.
# Usage          : 
#         In command prompt, type "python serial_decoder". Be aware of the com-
#        munication port. The default is 'COM8'.
#
# Contributor    :
#        rev.1 @  2017.11.30.
#                 HANARO, SNU.
#                 Jaerin Lee
#        rev.2 @  2019.03.04.
#                 HANARO, SNU.
#                 Inbum Park
###############################################################################

# Imports
import serial
import time
from collections import deque
from datetime import datetime
from ctypes import (Union, Structure, c_uint8, c_uint32, c_float, cast, POINTER)
import sys

# Connection attributes
port = '/dev/ttyACM1'
baud = 115200
timeout = 1
packetSize = 53

# Data packet type
class DataPacket(Structure):
    _fields_ = [('currentTime', c_uint32),
                ('aCoord', c_float * 3),
                ('gCoord', c_float * 3),
                ('mCoord', c_float * 3),
                ('temp', c_float),
                #('pressure', c_uint32),
                ('altPress', c_float),
                ('altMax', c_float),
                ('state', c_uint8)]
data = DataPacket()

# Port
while True:
    try:
        # Connect to the device
        ser = serial.Serial(port, baud, timeout=timeout)

        # Connection info
        print(datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f') +
            "\tConnection established @ " + ser.name)

        # Field name
        print('deviceTime' +
            '\taCoord[x]\taCoord[y]\taCoord[z]' +
            '\tgCoord[x]\tgCoord[y]\tgCoord[z]' +
            '\tmCoord[x]\tmCoord[y]\tmCoord[z]' +
            '\ttemperature' +
            #'\tpressure' +
            '\taltPress\taltMax\tstate')

        # Start collecting data
        break

    except serial.SerialException:
        time.sleep(0.05)

# Initialize cache memory
buf = b''
lastbuf = b''
chunk = deque(maxlen=(packetSize + 4))

# Begin communication
try:
    # Read data with start('$') and end('#') symbol
    buf = ser.read(2)
    while buf != b'':
        chunk.append(buf)
    
        # Start and end character matches
        if chunk[0] == b'$' and chunk[1] == b'$' and buf == b'#' and lastbuf == b'#':
            # Remove start and end symbols
            chunk.pop()
            chunk.pop()
            chunk.popleft()
            chunk.popleft()

            # Cast raw data into a parsable form
            rawData = b''.join(list(chunk))
            data = cast(rawData, POINTER(DataPacket)).contents

            # Parse the data
            currentTime = data.currentTime
            aCoord = list(data.aCoord)
            gCoord = list(data.gCoord)
            mCoord = list(data.mCoord)
            temperature = float(data.temp)
            #pressure = ((data.pressure / 1024.0 * 5 / 331.0) - 0.004) * (100 / 0.016)
            altPress = float(data.altPress)
            altMax = float(data.altMax)
            state = data.state

            # Print out the data to the CSV file
            print(str(currentTime) + "\t" +
                str(aCoord[0]) + "\t" + str(aCoord[1]) + "\t" + str(aCoord[2]) + "\t" +
                str(gCoord[0]) + "\t" + str(gCoord[1]) + "\t" + str(gCoord[2]) + "\t" +
                str(mCoord[0]) + "\t" + str(mCoord[1]) + "\t" + str(mCoord[2]) + "\t" +
                str(temperature) + "\t" +
                # str(pressure) + "\t" +
                str(altPress) + "\t" + str(altMax) + "\t" +
                str(state))

        # Read next byte
        lastbuf = buf
        buf = ser.read(1)

except serial.SerialException:
    # Unintended connection problem occurred
    sys.stderr.write("SerialException")
    sys.exit(1)

# Close
ser.close()
