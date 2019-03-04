###############################################################################
# Filename       : sdcard_decoder.py
# Target Machine : Arduino Mega with GY91 sensor module
#
# Description    : 
#        Decoder of SD card out from arduino implementing "Avionics.ino", with
#        default output format of CSV.
# Usage          : 
#         In command prompt, type "python sdcard_decoder filename". Be aware of
#         the communication port. The default is 'COM8'.
#
# Contributor    :
#        rev.1 @    11/30/2017
#                 Jaerin Lee
#        rev.2 @ 06/30/2018
#                Jaerin Lee
#        HANARO, SNU.
###############################################################################

# Imports
import time
from collections import deque
from datetime import datetime
from ctypes import (Union, Structure, c_uint8, c_uint16, c_uint32, c_float, cast, POINTER)
import sys

# Basic constants
packetSize = 37
# Data packet type
class DataPacket(Structure):
    _fields_ = [('currentTime', c_uint32),
                ('aCoord', c_float * 3),
                ('gCoord', c_float * 3),
                #('mCoord', c_float * 3),
            #    ('temp', c_float),
            #   ('pressure', c_uint32),
                ('altPress', c_float),
                ('altMax', c_float),
                ('state', c_uint8)]
data = DataPacket()

# Receive argument
if len(sys.argv) > 2:
    print('Usage: "python sdcard_decoder [filename]')
    sys.exit(1)

# Open file
try:
    f = open(sys.argv[1], 'rb')
except FileNotFoundError:
    print("Error: file [" + sys.argv[1] + "] not found")
    sys.exit(1)

# Field name (header)
print('deviceTime' +
    '\taCoord[x]\taCoord[y]\taCoord[z]' +
    '\tgCoord[x]\tgCoord[y]\tgCoord[z]' +
    #'\tmCoord[x]\tmCoord[y]\tmCoord[z]' +
    #'\ttemperature\tpressure' +
    '\taltPress\taltMax\tstate')

# Initialize cache memory
chunk = deque(maxlen=(packetSize + 4))

# Begin communication
try:
    # Read data with start('$$') and end('##') symbol
    buf = f.read(2)
    lastbuf = b''
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
            #mCoord = list(data.mCoord)
            #temperature = float(data.temp)
            #pressure = ((data.pressure / 1024.0 * 5 / 331.0) - 0.004) * (100 / 0.016)
            altPress = float(data.altPress)
            altMax = float(data.altMax)
            state = data.state

            # Print out the data to the CSV file
            print(str(currentTime) + "\t" +
                str(aCoord[0]) + "\t" + str(aCoord[1]) + "\t" + str(aCoord[2]) + "\t" +
                str(gCoord[0]) + "\t" + str(gCoord[1]) + "\t" + str(gCoord[2]) + "\t" +
                #str(mCoord[0]) + "\t" + str(mCoord[1]) + "\t" + str(mCoord[2]) + "\t" +
                #str(temperature) + "\t" +
                # + str(pressure) + "\t" +
                str(altPress) + "\t" + str(altMax) + "\t" +
                str(state))

        # Read next byte
        lastbuf = buf
        buf = f.read(1)

finally:
    # Close
    f.close()
