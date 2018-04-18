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
from pyprofibus import DpTelegram_SetPrm_Req, monotonic_time, DpSlaveState, DPM1

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
	    # Parse the config file.
	    config = pyprofibus.PbConf.fromFile("/home/david/robot_ws/src/vr_driver/scripts/et200s.conf")

	    # Create a PHY (layer 1) interface object
	    phy = config.makePhy()

	    # Create a DP class 1 master with DP address 1
	    master = pyprofibus.DPM1(phy = phy,
				    masterAddr = config.dpMasterAddr,
				    debug = True)

	    # Create a slave descriptions.
	    for slaveConf in config.slaveConfs:
		    gsd = slaveConf.gsd

		    # Create a slave description for an ET-200S.
		    # The ET-200S has got the DP address 8 set via DIP-switches.
		    slaveDesc = pyprofibus.DpSlaveDesc(identNumber = gsd.getIdentNumber(),
						        slaveAddr = slaveConf.addr)

		    # Create Chk_Cfg telegram
		    slaveDesc.setCfgDataElements(gsd.getCfgDataElements())

		    # Set User_Prm_Data
		    dp1PrmMask = bytearray((DpTelegram_SetPrm_Req.DPV1PRM0_FAILSAFE,
					DpTelegram_SetPrm_Req.DPV1PRM1_REDCFG,
					0x00))
		    dp1PrmSet  = bytearray((DpTelegram_SetPrm_Req.DPV1PRM0_FAILSAFE,
					DpTelegram_SetPrm_Req.DPV1PRM1_REDCFG,
					0x00))
		    slaveDesc.setUserPrmData(gsd.getUserPrmData(dp1PrmMask = dp1PrmMask,
							    dp1PrmSet = dp1PrmSet))

		    # Set various standard parameters
		    slaveDesc.setSyncMode(slaveConf.syncMode)
		    slaveDesc.setFreezeMode(slaveConf.freezeMode)
		    slaveDesc.setGroupMask(slaveConf.groupMask)
		    slaveDesc.setWatchdog(slaveConf.watchdogMs)

		    # Register the ET-200S slave at the DPM
		    master.addSlave(slaveDesc)

	    # Initialize the DPM
	    master.initialize()
	    slaveDescs = master.getSlaveList()

	    # Cyclically run Data_Exchange.
	    # 4 input bits from the 4-DI module are copied to
	    # the DO module.
	    inData = 0
	    rtSum, runtimes, nextPrint = 0, [ 0, ] * 512, monotonic_time() + 1.0
	    while True:
		    start = monotonic_time()

		    # Run slave state machines.
		    for slaveDesc in slaveDescs:
			    outData = [inData & 3]
			    inDataTmp = master.runSlave(slaveDesc, outData)
			    if inDataTmp is not None:
				    inData = inDataTmp[0]

		    # Print statistics.
		    end = monotonic_time()
		    runtimes.append(end - start)
		    rtSum = rtSum - runtimes.pop(0) + runtimes[-1]
		    if end > nextPrint:
			    nextPrint = end + 3.0
			    sys.stderr.write("pyprofibus cycle time = %.3f ms\n" %\
				    (rtSum / len(runtimes) * 1000.0))

    except pyprofibus.ProfibusError as e:
	    print("Terminating: %s" % str(e))
    finally:
	    if master:
		    master.destroy()
'''    
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
            #slaveDesc.setCfgDataElements(gsd.getCfgDataElements())

            # Set User_Prm_Data (not available in gsd of pana)
            #slaveDesc.setUserPrmData(gsd.getUserPrmData())
            
            # Set various standard Profibus parameters (syncmode, watchdog...)
            slaveDesc.setSyncMode(slaveConf.syncMode)
            slaveDesc.setFreezeMode(slaveConf.freezeMode)
            #slaveDesc.setGroupMask(slaveConf.groupMask)
            #slaveDesc.setWatchdog(slaveConf.watchdogMs)

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
        in1 = 0b11101000
        in2 = 0b00000011
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

        # send x[mm]= 1000 to robot as an array of 14 bytes
        dataIn = [format(in1, '08b'), format(in2, '08b'), format(in3, '08b'), format(in4, '08b'),
                format(in5, '08b'), format(in6, '08b'), format(in7, '08b'), format(in8, '08b'), format(in9, '08b'),
                format(in10, '08b'), format(in11, '08b'), format(in12, '08b'), format(in13, '08b'), format(in14, '08b') ]    
        print "=== Data in ==="
        print dataIn        # check if dataIn is converted correctly from WORD to BINARY      
        
        while True:
            start = monotonic_time()      

            # Run data exchange
            for slaveDesc in slaveDescs:
                dataOut = dataIn
                dataInTemp = master.runSlave(slaveDesc, dataOut)
                
                #print dataInTemp        # check if data is tranceived or not. This variable must have the value of 'not None' to be send 

                if dataInTemp is not None:
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
'''
###############################
#       MAIN-PROGRAM          #
###############################

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
