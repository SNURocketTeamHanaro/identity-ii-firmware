###############################################################################
# Filename       : serial_decoder.py
# Target Machine : Arduino Mega with GY91 sensor module
#
# Description    : 
#		Decoder of serial input from arduino implementing "Avionics.ino", with
#		default output format of CSV.
# Usage          : 
# 		In command prompt, type "python serial_decoder". Be aware of the com-
#		munication port. The default is 'COM8'.
#
# Contributor    :
#		rev.1 @	2017.11.30.
# 				HANARO, SNU.
# 				Jaerin Lee
###############################################################################

# Imports
import serial
import time
from collections import deque
from datetime import datetime
from ctypes import (Union, Structure, c_uint8, c_uint32, c_float, cast, POINTER)
import sys

# Connection attributes
port = 'COM8'
baud = 115200
timeout = 1
packetSize = 53

# Data packet type
class DataPacket(Structure):
	_fields_ = [('currentTime', c_uint32),
				('aCoord', c_float * 3),
				('gCoord', c_float * 3),
				('mCoord', c_float * 3),
				('altPress', c_float),
				('altMax', c_float),
				('count', c_uint32),
				('on', c_uint8)]
data = DataPacket()

# Port
while True:
	try:
		# Connect to the device
		ser = serial.Serial(port, baud, timeout=timeout)

		# Connection info
		print(datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f') +
			",Connection established @ " + ser.name)

		# Field name
		print('realTime,deviceTime' +
			',aCoord[x],aCoord[y],aCoord[z]' +
			',gCoord[x],gCoord[y],gCoord[z]' +
			',mCoord[x],mCoord[y],mCoord[z]' +
			',altPress,altMax,count,isOn')

		# Start collecting data
		break

	except serial.SerialException:
		time.sleep(0.05)

# Initialize cache memory
buf = b''
chunk = deque(maxlen=(packetSize + 2))

# Begin communication
while True:
	try:
		# Read data with start('$') and end('#') symbol
		buf = ser.read()
		chunk.append(buf)
		
		# Start and end character matches
		if chunk[0] == b'$' and buf == b'#':
			# Remove start and end symbols
			chunk.pop()
			chunk.popleft()

			# Cast raw data into a parsable form
			rawData = b''.join(list(chunk))
			data = cast(rawData, POINTER(DataPacket)).contents

			# Parse the data
			currentTime = data.currentTime
			aCoord = list(data.aCoord)
			gCoord = list(data.gCoord)
			mCoord = list(data.mCoord)
			altPress = float(data.altPress)
			altMax = float(data.altMax)
			count = data.count
			isOn = bool(data.on)

			# Print out the data to the CSV file
			print(datetime.now().strftime('%H:%M:%S.%f') + "," +
				str(currentTime) + "," +
				str(aCoord[0]) + "," + str(aCoord[1]) + "," + str(aCoord[2]) + "," +
				str(gCoord[0]) + "," + str(gCoord[1]) + "," + str(gCoord[2]) + "," +
				str(mCoord[0]) + "," + str(mCoord[1]) + "," + str(mCoord[2]) + "," +
				str(altPress) + "," + str(altMax) + "," +
				str(count) + "," + str(isOn))

	except serial.SerialException:
		# Unintended connection problem occurred
		sys.stderr.write("SerialException")
		sys.exit(1)

# Close
ser.close()