#!/usr/bin/env python

#############################
# This python program reads and writes I/O's of
# the Panasonic VR-006L controller to obtain
# a ROS driver using Profibus
# The hardware configuration is as follows:
#
#   v--------------v-----------v
#   |   Panasonic  |    ROS    |
#   |     GII      |   Linux   |
#   |  Controller  |           |
#   v--------------v-----------v
#   |     DP       |    DP     |
#   |    Slave     |   Master  |
#   ^--------------^-----------^
#
#############################

#############################
#      import libraries     #
#############################

import sys, threading, math, copy, array
import serial
import numpy as np
import rospy
import std_msgs
from std_msgs.msg import String
import trajectory_msgs
import moveit_msgs.msg
import moveit_commander
import moveit_ros_planning
import visualization_msgs
import geometry_msgs.msg
import roslib; roslib.load_manifest('vr_driver')
import pyprofibus
from pyprofibus import DpTelegram_SetPrm_Req, monotonic_time, DpSlaveState

##############################

'''
def move_robot_moveit():
    
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("manipulator")

    display_position_streaming_publisher = rospy.Publisher('/move_group/display_planned_path', 
                                                            moveit_msgs.msg.DisplayRobotState)
    
    print "=== Waiting for RViz ==="
    print "=== Current robot pose:" 
    print group.get_current_pose()

    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = 0.006  # put a x-position (in ROS) in meter 
    pose_target.position.y = -0.50  # put a y-position (in ROS) in meter
    pose_target.position.z = 0.5    # put a z-position (in ROS) in meter

    group.set_pose_target(pose_target)

    plan1 = group.plan()

    print "=== Visualising plan1 ==="
    display_position_streaming = moveit_msgs.msg.DisplayTrajectory()
    display_position_streaming.trajectory_start = robot.get_current_state()
    display_position_streaming.trajectory.append(plan1)
    display_position_streaming_publisher.publish(display_position_streaming)
    
    print "=== Current Pose ==="
    print group.get_current_pose()

    moveit_commander.roscpp_shutdown()
'''    

def setup_communication():
    master = None
    try:
        # Parse the config file (see /config/panasonic_profibus.conf)
        config = pyprofibus.PbConf.fromFile("/home/david/robot_ws/src/vr_driver/scripts/panaprofi.conf")

        # Create a PHY (layer 1 of OSI) interface object
        phy = config.makePhy()

        # Create a DP class 1 master with DP address 1 (DP1 Master) 
        master = pyprofibus.DPM1(phy = phy, masterAddr = config.dpMasterAddr, debug = True)

        # Create a slave description
        for slaveConf in config.slaveConfs:
            gsd = slaveConf.gsd

            # Create a slave description for the Panasonic VR-006L controller
            slaveDesc = pyprofibus.DpSlaveDesc(identNumber = gsd.getIdentNumber(), slaveAddr = slaveConf.addr)

            # Create Chk_Cfg telegram (not available in gsd of pana)
            slaveDesc.setCfgDataElements(gsd.getCfgDataElements())

            # Set User_Prm_Data (not available in gsd of pana)
            slaveDesc.setUserPrmData(gsd.getUserPrmData())
            
            # Set various standard Profibus parameters (syncmode, watchdog...)
            slaveDesc.setSyncMode(slaveConf.syncMode)
            slaveDesc.setFreezeMode(slaveConf.freezeMode)
            slaveDesc.setGroupMask(slaveConf.groupMask)
            slaveDesc.setWatchdog(slaveConf.watchdogMs)

            # Register the Panasonic controller as slave at the DP1
            master.addSlave(slaveDesc)
            
        # Initializing the DPM
        master.initialize()
        slaveDescs = master.getSlaveList()
        print "== Registered slaves ===:" 
        print slaveDescs

        # Cyclically run data exchange with slave and send position from ROS to controller
        rtSum, runtimes, nextprint = 0, [0, ] * 512, monotonic_time() + 1.0

        # a size of 14 bytes or 7 integers will be send
        in1 = 1000
        in2 = 0
        in3 = 0
        in4 = 0
        in5 = 0
        in6 = 0
        in7 = 0
        in8 = 0
        in9 = 0
        in10 = 0
        in11 = 0
        in12 = 0
        in13 = 0
        in14 = 0


        dataIn = [format(in1, '016b'), format(in2, '016b'), format(in3, '016b'), format(in4, '016b'),
                format(in5, '016b'), format(in6, '016b'), format(in7, '016b'), format(in8, '016b'), format(in9, '016b'),
                format(in10, '016b'), format(in11, '016b'), format(in12, '016b'), format(in13, '016b'), format(in14, '016b') ]    # send x[mm]= 1000 to robot as an array of 14 bytes
        print "=== Data in ==="
        print dataIn        # check if dataIn is converted correctly from WORD to BINARY      
        
        while True:
            start = monotonic_time()      


            # Run data exchange
            for slaveDesc in slaveDescs:
                dataOut = dataIn
                dataInTemp = master.runSlave(slaveDesc, dataOut)
                
                print dataInTemp        # check if data is tranceived or not. This variable has the value of 'None'  

                if dataInTemp is None:
                    dataIn = dataInTemp 
                                        

            # Print Profibus cycle statistics
            end = monotonic_time()
            runtimes.append(end-start)
            rtSum = rtSum - runtimes.pop(0) + runtimes[-1]
            if end > nextprint:
                nextprint = end + 3.0
                sys.stderr.write("Profibus cycle time = %.3f ms\n" %\
                        (rtSum / len(runtimes) * 1000.0))


    except pyprofibus.ProfibusError as e:
        print("Terminating: %s" % str(e))
    finally:
        if master:
            master.destroy()


def main():
    try:
        rospy.init_node('vr_driver', anonymous=True)   
        
        while not rospy.is_shutdown():
            # Show position streaming in moveit
        #move_robot_moveit()

            # Stream position to robot over Profibus
            setup_communication()

    except rospy.ROSInterruptException or KeyboardInterrupt :
        pass



if __name__=='__main__':main()
