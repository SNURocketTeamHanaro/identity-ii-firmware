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
from math import *
import numpy as np

# Mode of operation
#mode_plot = true
#mode_rot_anim = false
#mode_save_data = false

tilt = 84 * pi / 180

state_dp = 6
state_mp = 7

# Field names
f_dev_time = 1
f_A = np.array([2,3,4])
f_G = np.array([5,6,7])
#f_temp = 8
#f_press = 9
f_alt_press = 8
f_alt_max = 9
f_state = 10

M = np.random.random((1,10))
# Initialize sate variable
# g
g = np.array([0, 0, -1])
g0 = 9.80665
# Sensor to rocket body
C = np.array([[0, 0, -1], [0, 1 ,0], [1, 0, 0]])
# Inertial frame to Earth frame
E = np.array([[0, 0, -1], [0, 1, 0], [-1, 0, 0]])

# Connection attributes
port = '/dev/ttyACM0'
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

            # Data retrieval
            M = np.concatenate((M, np.array([[aCoord[0], aCoord[1], aCoord[2], gCoord[0], gCoord[1], gCoord[2], temperature, altPress, altMax, state]])), axis=0)
            # Retrieve fields
            dev_time = M[:, f_dev_time-1]
            A = M[:,f_A-1]
            G = M[:,f_G-1]
            #temp = M[:, f_temp-1];
            #press = M[:, f_press-1];
            alt_press = M[:, f_alt_press-1]
            alt_max = M[:, f_alt_max-1]
            state = np.uint8(M[:, f_state-1])
            n = dev_time.shape[0]
            print(n)
            
            ## Pre-process
            #ignition_start = (np.argwhere(state>0))[0][0]
            #print(ignition_start)
            #calib_ref = 2 #state = state[ignition_start:]

            #Aoff = np.mean(A[calib_ref-1,:], axis = 0)
            #Aoff_ref = np.array([cos(tilt), 0, sin(tilt)])
            #A = A-np.tile(Aoff-Aoff_ref, (A.shape[0], 1))
            #Goff = np.mean(G[calib_ref-1,:], axis = 0)
            #G = G-np.tile(Goff, (G.shape[0], 1))

            #alt_offset = np.mean(alt_press, axis = 0)
            #alt_press = alt_press - alt_offset
            #alt_max = alt_max - alt_offset

            # Pre-processing of pressure
            # min_current = .004   # A
            # max_current = .020   # A 
            # min_gauge = 0        # bar
            # max_gauge = 100      # bar
            # atm_press = 1.01325  # bar

            # press = (min_current/press[0:19].mean(axis = 0)) * press
            # gauge = (press-min_current)*(max_gauge-min_gauge)/(max_current-min_current)+min_gauge
            # press = gauge + atm_press

            ## Post-process
            # Post-process of device time
            dev_time = (dev_time - dev_time[0])/1000;

            # Eulerian angle and its variation
            Phi = np.zeros((n, 1))
            Theta = np.zeros((n, 1))
            Psi = np.zeros((n, 1))
            dPhi = np.zeros((n-1, 1))
            dTheta = np.zeros((n-1, 1))
            dPsi = np.zeros((n-1, 1))
            EPhi = np.zeros((n-1, 1))
            ETheta = np.zeros((n-1, 1))
            EPsi = np.zeros((n-1, 1))
            # Quaternion and its variation
            a = np.zeros((n, 1))
            b = np.zeros((n, 1))
            c = np.zeros((n, 1))
            d = np.zeros((n, 1))
            da = np.zeros((n-1, 1))
            db = np.zeros((n-1, 1))
            dc = np.zeros((n-1, 1))
            dd = np.zeros((n-1, 1))
            # Inertial frame
            ri = np.zeros((n, 3))
            vi = np.zeros((n, 3))
            ai = np.zeros((n-1, 3))
            # Direction cosine matrix from body frame to inertial frame
            C_Q = np.zeros((3,3,n-1))

            # Initial state
            Phi[0] = 0
            Theta[0] = -pi/2+tilt
            Psi[0] = 0
            EPhi[0] = Phi[0]
            ETheta[0] = Theta[0]
            EPsi[0] = Psi[0]
            a[0] = cos(Phi[0]/2) * cos(Theta[0]/2) * cos(Psi[0]/2) + sin(Phi[0]/2) * sin(Theta[0]/2) * sin(Psi[0]/2)
            b[0] = sin(Phi[0]/2) * cos(Theta[0]/2) * cos(Psi[0]/2) + cos(Phi[0]/2) * sin(Theta[0]/2) * sin(Psi[0]/2)
            a[0] = cos(Phi[0]/2) * sin(Theta[0]/2) * cos(Psi[0]/2) + sin(Phi[0]/2) * cos(Theta[0]/2) * sin(Psi[0]/2)
            a[0] = cos(Phi[0]/2) * cos(Theta[0]/2) * sin(Psi[0]/2) + sin(Phi[0]/2) * sin(Theta[0]/2) * cos(Psi[0]/2)

            # Acceleration body frame
            ab = (np.dot(C,A.transpose())).transpose()

            # Angular velocity body frame
            w = (np.dot(C,G.transpose())).transpose()
            wx = w[:,0]*pi/180
            wy = w[:,1]*pi/180
            wz = w[:,2]*pi/180

            # Headings update
            for i in range(1,n-1):
                dt = dev_time[i] - dev_time[i-1]

                # Update quaternion from angular velocity
                da[i-1] = -0.5*(b[i-1]*wx[i-1] + c[i-1]*wy[i-1] + d[i-1]*wz[i-1])
                db[i-1] = 0.5*(a[i-1]*wx[i-1] - d[i-1]*wy[i-1] + c[i-1]*wz[i-1])
                dc[i-1] = 0.5*(d[i-1]*wx[i-1] + a[i-1]*wy[i-1] - b[i-1]*wz[i-1])
                dd[i-1] = -0.5*(c[i-1]*wx[i-1] - b[i-1]*wy[i-1] - a[i-1]*wz[i-1])

                a[i] = a[i-1] + dt*da[i-1]
                b[i] = b[i-1] + dt*db[i-1]
                c[i] = c[i-1] + dt*dc[i-1]
                d[i] = d[i-1] + dt*dd[i-1]

                # Update direction cosine matrix
                C_Q[:,:,i-1][:,:,np.newaxis] = np.array([
                        [
                            a[i]*a[i] + b[i]*b[i] - c[i]*c[i]- d[i]*d[i],
                            2*(b[i]*c[i])-a[i]*d[i],
                            2*(b[i]*d[i] + a[i]*c[i])
                        ],
                        [
                            2*(b[i]*c[i] + a[i]*d[i]),
                            (a[i]*a[i] - b[i]*b[i] + c[i]*c[i] - d[i]*d[i]),
                            2*(b[i]*c[i] + a[i]*d[i])
                        ],
                        [
                            2*(b[i]*d[i]) - a[i]*c[i],
                            2*(c[i]*d[i] + a[i]*b[i]),
                            (a[i]*a[i] - b[i]*b[i] - c[i]*c[i] + d[i]*d[i])
                        ]
                        ])

                # Update Euler angle from angular velocity
                dPhi[i-1] = (wy[i-1]*sin(Phi[i-1]) + wz[i-1]*cos(Phi[i-1])*tan(Theta[i-1]) + wx[i-1])
                dTheta[i-1] = wy[i-1]*cos(Phi[i-1]) - wz[i-1]*sin(Phi[i-1])
                dPsi[i-1] = (wy[i-1]*sin(Phi[i-1]) + wz[i-1]*cos(Phi[i-1]))/cos(Theta[i-1])

                EPhi[i] = EPhi[i-1] + dPhi[i-1]*dt
                ETheta[i] = ETheta[i-1] + dTheta[i-1]*dt
                EPsi[i] = EPsi[i-1] + dPsi[i-1]*dt
                
                Phi[i] = atan2(C_Q[2,1,i-1], C_Q[2,2,i-1])
                Theta[i] = asin(-C_Q[2,0,i-1])
                Psi[i] = atan2(C_Q[1,0,i-1], C_Q[0,0,i-1])

                # Update accleration from inertial frame
                ai[i-1, :] = (np.dot(np.dot(E,(C_Q[:,:,i-1])),ab[i-1,:]) + g)*g0

            # Update velocity and position
            for i in range (1,n-1):
                dt = dev_time[i] - dev_time[i-1]

                vi[i,0] = vi[i-1, 0] + dt*ai[i-1,0]
                vi[i,1] = vi[i-1, 1] + dt*ai[i-1,1]
                vi[i,2] = vi[i-1, 2] + dt*ai[i-1,2]

                ri[i,0] = ri[i-1, 0] + dt*vi[i-1,0]
                ri[i,1] = ri[i-1, 1] + dt*vi[i-1,1]
                ri[i,2] = ri[i-1, 2] + dt*vi[i-1,2]
            print(vi)
            print(ri)
    # Read next byte
        lastbuf = buf
        buf = ser.read(1)

except serial.SerialException:
    # Unintended connection problem occurred
    sys.stderr.write("SerialException")
    sys.exit(1)

# Close
ser.close()
