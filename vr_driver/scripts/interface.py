#!/usr/bin/env python
###############################################
# This python program is a bidirectional 
# interface between ROS and the Panasonic 
# VR-006L controller. In this python script
# the user can send positions to the robot 
# through the terminal.
#          v--------------v-----------v
#          |   Panasonic  |    ROS    |
#          |     GII      |   Linux   |
#          |  Controller  |           |
#          v--------------v-----------v
#          |     DP       |    DP     |
#          |    Slave     |   Master  |
#          ^--------------^-----------^
#
###############################################

###############################################
#              import libraries               #
###############################################

import sys, threading, math, copy, array, time
from time import sleep
import numpy as np
import rospy
import pyprofibus
from pyprofibus import DpTelegram_SetPrm_Req, monotonic_time
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial

###########################################

def getInfoFromTerminal():
    while True:
        try:
            x = int(input("Give an integer value for the x-position in mm (between 200 and 1400):"))
        except ValueError:
            print("Sorry, I didn't understand that.")
        if not 200 <= x <= 1400:
            print("Sorry, x must be between 200 and 1400.")
            continue
        else:
            break
    while True:
        try:
            y = int(input("Give an integer value for the y-position in mm (between -800 and 800):"))
        except ValueError:
            print("Sorry, I didn't understand that.")
        if not -800 <= y <= 800:
            print("Sorry, y must be between -800 and 800.")
            continue
        else:
            break
    while True:
        try:
            z = int(input("Give an integer value for the z-position in mm (between 0 and 1100):"))
        except ValueError:
            print("Sorry, I didn't understand that.")
        if not 0 <= z <= 1100:
            print("Sorry, z must be between 0 and 1100.")
            continue
        else:
            break   
    while True:
        try:
            Rx = int(input("Give an integer value for the x-orientation in degrees (between -90 and 90):"))
        except ValueError:
            print("Sorry, I didn't understand that.")
        if not -90 <= Rx <= 90:
            print("Sorry, Rx must be between -90 and 90.")
            continue
        else:
            break
    while True:
        try:
            Ry = int(input("Give an integer value for the y-orientation in degrees (between -90 and 90):"))
        except ValueError:
            print("Sorry, I didn't understand that.")
        if not -90 <= Ry <= 90:
            print("Sorry, Ry must be between -90 and 90.")
            continue
        else:
            break
    while True:
        try:
            Rz = int(input("Give an integer value for the z-orientation in degrees (between -90 and 90):"))
        except ValueError:
            print("Sorry, I didn't understand that.")
        if not -90 <= Rz <= 90:
            print("Sorry, Rz must be between -90 and 90.")
            continue
        else:
            break
    while True:
        try:
            mType = int(input("Now give a movetype: Point-to-point (1), Continuous-Linear (2), Continuous-Circular (3) --> "))
        except ValueError:
            print("Sorry, I didn't understand that.")
            continue
        if not (mType == 1 or mType == 2 or mType == 3):
            print("Sorry, movetype must be 1 or 2 or 3.")
            continue
        else:
            #we can exit the loop
            break
    
    return x, y, z, Rx, Ry, Rz, mType


def convertValueFirstByte(value):
    
    convertedValue = int('{0:016b}'.format(((1<<16) -1) & value)[8:],2) 

    return convertedValue


def convertValueSecondByte(value):
    
    convertedValue = int('{0:016b}'.format(((1<<16) -1) & value)[:8],2) 

    return convertedValue


def convertMovementType(value):

    convertedValue = int('{0:08b}'.format(value),2)

    return convertedValue


def convertFlag():

    convertedValue = int('{0:08b}'.format(1),2) 

    return convertedValue


def getValues():
    data = getInfoFromTerminal()

    x1 = convertValueFirstByte(data[0])
    x2 = convertValueSecondByte(data[0])

    y1 = convertValueFirstByte(data[1])
    y2 = convertValueSecondByte(data[1])

    z1 = convertValueFirstByte(data[2])
    z2 = convertValueSecondByte(data[2])

    Rx1 = convertValueFirstByte(data[3])
    Rx2 = convertValueSecondByte(data[3])

    Ry1 = convertValueFirstByte(data[4])
    Ry2 = convertValueSecondByte(data[4])

    Rz1 = convertValueFirstByte(data[5])
    Rz2 = convertValueSecondByte(data[5])

    flag = convertFlag()
    mType = convertMovementType(data[6])

    posRotArray = [x1, x2, y1, y2, z1, z2, Rx1, Rx2, Ry1, Ry2, Rz1, Rz2, flag, mType]

    return posRotArray 


def setupCommunication():

    try:
        config  = pyprofibus.PbConf.fromFile("/home/david/robot_ws/src/vr_driver/config/panaprofi.conf")
        
        phy = config.makePhy()

        master = pyprofibus.DPM1(phy=phy, masterAddr=config.dpMasterAddr, debug=False)

        # slaveConf = config.slaveConfs[0]
        # slaveConf = config.slaveConfs.__next__()
        for slaveConf in config.slaveConfs:
            gsd = slaveConf.gsd

            slaveDesc = pyprofibus.DpSlaveDesc(identNumber=gsd.getIdentNumber(), slaveAddr=slaveConf.addr)
            slaveDesc.setCfgDataElements(gsd.getCfgDataElements())
            slaveDesc.setSyncMode(slaveConf.syncMode)
            slaveDesc.setFreezeMode(slaveConf.freezeMode)
            slaveDesc.setGroupMask(slaveConf.groupMask)
            slaveDesc.setWatchdog(slaveConf.watchdogMs)

        master.addSlave(slaveDesc)

        master.initialize()
        return master, slaveDesc

    except pyprofibus.ProfibusError as e:
        print("Terminating: %s" % str(e))

        return None, None


def sendValues(master, slaveDesc, dataArray):
    outputArray = None
    while (outputArray is None):
        outputArray = master.runSlave(slaveDesc, dataArray)

    return outputArray


def visualizeMovement():

    return


# fig, ax = plt.subplots()
# line, = ax.plot(np.random.rand(10))
# ax.set_ylim(-5000,5000)
# xdata, ydata = [0]*100, [0]*100
# ser = serial.Serial("/dev/ttyUSB0", 19200)
# #ser.open()

# def updateData(data):
#     line.set_ydata(data)

#     return line


# def runDataUSB(data):
#     t, y = data
#     del xdata[0]
#     del ydata[0]
#     xdata.append(t)
#     ydata.append(y)
#     line.set_data(xdata, ydata)
    
#     return line

# def captureData():
#     t = 0
#     while True:
#         t = t + 1
#         try:
#             dat = int(ser.read())
#         except:
#             dat = 0
#         yield t, dat
        
#def printDataTraffic():
    
    


def main():
    try:
        rospy.init_node('interface', anonymous=True)
        
        rospy.loginfo("Initializing position streaming interface %s")
        
        rospy.loginfo("Setting up communication with Panasonic robot controller %s")
        
        master, slaveDesc = setupCommunication()

        print('hier ben ik de eerste keer')
        
        if master is not None:

            while True:
                rospy.loginfo("Getting user information from terminal %s")
                dataArray = getValues()
                rospy.loginfo("Sending position and orientation to robot")
                #print('ik ben hier ook')
                
                outputArray = sendValues(master, slaveDesc, dataArray)
                
                # ani = animation.FuncAnimation(fig, runDataUSB, captureData, interval=0, blit=True)
                # plt.show()
                #if outputArray[1] != 0 or outputArray[2] != 0 or outputArray[3] != 0 or outputArray[4] != 0 or outputArray[5] != 0:
                print outputArray 
                #print('ik heb verzonden')

                
                # while (outputArray is None):
                #     sleep(1)


    except KeyboardInterrupt:
        rospy.logerr("Keyboardinterrupt")
        rospy.signal_shutdown("KeyboardInterrupt")
        raise


if __name__ == '__main__': main()
