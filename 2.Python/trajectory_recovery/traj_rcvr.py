###############################################################################
# Filename       : traj_recv.py
# Target Machine : Arduino Mega2560 with GY91 sensor module
#
# Description    : 
#		TODO
# Usage          : 
# 		TODO
#
# Contributor    :
#		rev.1 @	2018.02.23.
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
import numpy as np
from matplotlib import pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3

# TODO parameterize important variables
# Environmental constants
# Gravitational acceleration (Standard = 9.80665); Use regional constant
g = 9.80665

# Connection attributes
port = 'COM13'
baud = 115200
timeout = 1
packetSize = 37 # 53

# Plotting settings
frameRate = 24
frameTime = 1. / frameRate

# Data packet type
class DataPacket(Structure):
	_fields_ = [('currentTime', c_uint32),
				('aCoord', c_float * 3),
				('gCoord', c_float * 3),
				# ('mCoord', c_float * 3),
				('altPress', c_float),
				('altMax', c_float),
				# ('count', c_uint32),
				('on', c_uint8)]
data = DataPacket()

# Some transformation functions
# TODO refactor these massive calculation into more formally reusable one
def euler2quar(euler):
	quar = np.zeros(4)
	tmpCos = np.cos(euler / 2)
	tmpSin = np.sin(euler / 2)
	return np.array([
		tmpCos[0] * tmpCos[1] * tmpCos[2] + tmpSin[0] * tmpSin[1] * tmpSin[2],
		tmpSin[0] * tmpCos[1] * tmpCos[2] + tmpCos[0] * tmpSin[1] * tmpSin[2],
		tmpCos[0] * tmpSin[1] * tmpCos[2] + tmpSin[0] * tmpCos[1] * tmpSin[2],
		tmpCos[0] * tmpCos[1] * tmpSin[2] + tmpSin[0] * tmpSin[1] * tmpCos[2]
		])

def quar2dcm(quar):
	return np.array([
		[quar[0] ** 2 + quar[1] ** 2 - quar[2] ** 2 - quar[3] ** 2,
		2 * (quar[1] * quar[2] - quar[0] * quar[3]),
		2 * (quar[1] * quar[3] + quar[0] * quar[2])],
		[2 * (quar[1] * quar[2] + quar[0] * quar[3]),
		quar[0] ** 2 - quar[1] ** 2 + quar[2] ** 2 - quar[3] ** 2,
		2 * (quar[2] * quar[3] - quar[0] * quar[1])],
		[2 * (quar[1] * quar[3] - quar[0] * quar[2]),
		2 * (quar[2] * quar[3] + quar[0] * quar[1]),
		quar[0] ** 2 - quar[1] ** 2 - quar[2] ** 2 + quar[3] ** 2]
		])

def evalDeltaQuar(quar, angVel):
	return 0.5 * np.dot(np.array([
		[quar[0], -quar[1], -quar[2], -quar[3]],
		[quar[1], quar[0], -quar[3], quar[2]],
		[quar[2], quar[3], quar[0], -quar[1]],
		[quar[3], -quar[2], quar[1], quar[0]]]),
		np.array([0., angVel]))

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
			# ',mCoord[x],mCoord[y],mCoord[z]' +
			',altPress,altMax' +
			# ',count' + 
			',isOn')

		# Start collecting data
		break

	except serial.SerialException:
		time.sleep(0.05)

# Initialize cache memory
buf = b''
chunk = deque(maxlen=(packetSize + 2))

# Initialize global variables
numDataBeforeTakeoff = 0
aCoordBias = np.zeros(3)
gCoordBias = np.zeros(3)
# mCoordBias = np.zeros(3)
isTraceInitComplete = False
eulerAngle = np.array([0., - np.pi / 2, 0.])
quarternion = euler2quar(eulerAngle)
accelBody = np.zeros(3)
accelIner = np.zeros(3)
velocity = np.zeros(3)
posTrace = position = np.zeros(3)
angVel = np.zeros(3)
launchTime = 0;
previousTime = 0;
prevFrameTime = 0;
# Direction Cosine Matrix can be dependent to rocket orientation
dcm = np.array([
	[0., -1., 0.],
	[1., 0., 0.],
	[0., 0., 1.]])
gravAccel = np.array([0., 0., -1.])

# Begin communication
while True:
	try:
		# Read data with start('$') and end('#') symbol
		buf = ser.read()
		chunk.append(buf)
		
		#print(buf)

		# Start and end character matches
		if chunk[0] == b'$$' and buf == b'##':
			# Remove start and end symbols
			chunk.pop()
			chunk.popleft()

			# Cast raw data into a parsable form
			rawData = b''.join(list(chunk))
			data = cast(rawData, POINTER(DataPacket)).contents

			# Parse the data
			currentTime = data.currentTime
			aCoord = np.array(list(data.aCoord))
			gCoord = np.array(list(data.gCoord))
			# mCoord = np.array(list(data.mCoord))
			altPress = float(data.altPress)
			altMax = float(data.altMax)
			# count = data.count
			isOn = data.on

			# Print out the data to the CSV file
			print(datetime.now().strftime('%H:%M:%S.%f') + "," +
				str(currentTime) + "," +
				str(aCoord[0]) + "," + str(aCoord[1]) + "," + str(aCoord[2]) + "," +
				str(gCoord[0]) + "," + str(gCoord[1]) + "," + str(gCoord[2]) + "," +
				# str(mCoord[0]) + "," + str(mCoord[1]) + "," + str(mCoord[2]) + "," +
				str(altPress) + "," + str(altMax) + "," +
				# str(count) + "," +
				str(isOn))

			# # Evaluate Trajectory
			# if isOn == 0:
			# 	numDataBeforeTakeoff = numDataBeforeTakeoff + 1
			# 	aCoordBias = aCoordBias + aCoord;
			# 	gCoordBias = gCoordBias + gCoord;
			# 	# mCoordBias = mCoordBias + mCoord;
			# else:
			# 	if not isTraceInitComplete:
			# 		# Setup biases
			# 		launchTime = currentTime
			# 		aCoordBias = aCoordBias / numDataBeforeTakeoff
			# 		gCoordBias = gCoordBias / numDataBeforeTakeoff
			# 		# mCoordBias = mCoordBias / numDataBeforeTakeoff
			# 		isTraceInitComplete = True

			# 		# Setup plots
			# 		prevFrameTime = currentTime
			# 		fig = plt.figure()
			# 		axPos = fig.add_subplot(221, aspect='auto', autoscale_on=True)
			# 		axPos.grid()
			# 		axVel = fig.add_subplot(223, aspect='auto', autoscale_on=True)
			# 		axVel.grid()
			# 		axAcc = fig.add_subplot(224, aspect='auto', autoscale_on=True)
			# 		axAcc.grid()
			# 		axOri = fig.add_subplot(222, aspect='equal', autoscale_on=False,
			# 			xlim=(-1, 1), ylim=(-1, 1))
			# 		axOri.grid()

			# 	# Evaluate acceleration and angular velocity on body frame
			# 	accelBody = np.dot(dcm, aCoord - aCoordBias)
			# 	angVel = np.deg2rad(np.dot(dcm, gCoord - gCoordBias))
				
			# 	# Evaluate the size of sampling interval
			# 	deltaTime = (currentTime - previousTime) / 1000.0
			# 	previousTime = currentTime

			# 	# Evaluate quarternion based on euler angle
			# 	deltaQuar = evalDeltaQuar(quarternion, angVel)
			# 	quarternion = np.cumsum(deltaTime * deltaQuar)
			# 	dcm = quar2dcm(quarternion)

			# 	# Evaluate trajectory by simple integration
			# 	accelIner = np.dot(dcm, accelBody) + gravAccel
			# 	position = np.cumsum(deltaTime * velocity)
			# 	velocity = np.cumsum(deltaTime * accelIner)
			# 	posTrace = np.append(position, 0)

			# 	# Plot every frame rate
			# 	if currentTime - prevFrameTime >= frameTime:
			# 		# Plot trajectory in real time


			# 		# Plot orientation in real time
			# 		orientation = dcm;

	except serial.SerialException:
		# Unintended connection problem occurred
		sys.stderr.write("SerialException")
		sys.exit(1)

# Close
ser.close()
