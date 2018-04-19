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
rospy.init_node('vr_driver', anonymous=True)   

while not rospy.is_shutdown():
    master = None

    try:
        config = pyprofibus.PbConf.fromFile("/home/david/robot_ws/src/vr_driver/scripts/panaprofi.conf")
        phy = config.makePhy()
        master = pyprofibus.DPM1(phy=phy, masterAddr=config.dpMasterAddr, debug=True)

        for slaveConf in config.slaveConfs:
            gsd = slaveConf.gsd
            slaveDesc = pyprofibus.DpSlaveDesc(identNumber=gsd.getIdentNumber(), slaveAddr = slaveConf.addr)

            #dp1PrmMask = bytearray((DpTelegram_SetPrm_Req.DPV1PRM0_FAILSAFE, DpTelegram_SetPrm_Req.DPV1PRM1_REDCFG,0x00))
            #dp1PrmSet = bytearray((DpTelegram_SetPrm_Req.DPV1PRM0_FAILSAFE, DpTelegram_SetPrm_Req.DPV1PRM1_REDCFG,0x00))
            #slaveDesc.setUserPrmData(gsd.getUserPrmData(dp1PrmMask = dp1PrmMask, dp1PrmSet = dp1PrmSet))
	    


            slaveDesc.setCfgDataElements(gsd.getCfgDataElements())
            slaveDesc.setUserPrmData(gsd.getUserPrmData())

            slaveDesc.setSyncMode(slaveConf.syncMode)
            slaveDesc.setFreezeMode(slaveConf.freezeMode)
            slaveDesc.setGroupMask(slaveConf.groupMask)
            slaveDesc.setWatchdog(slaveConf.watchdogMs)

            master.addSlave(slaveDesc)
        
        master.initialize()
        slaveDescs = master.getSlaveList()

        in1 = 0b00001111
	in2 = 0b00000000
	in3 = 0b00000000
	in4 = 0b00000000
	in5 = 0b00000000
	in6 = 0b00000000
	in7 = 0b00000000
	in8 = 0b00000000
	in9 = 0b00000000
	in10 = 0b0000000
	in11 = 0b00000000
	in12 = 0b00000000
	in13 = 0b00000000
	in14 = 0b00000000
	indata = [in1, in2, in3, in4, in5, in6, in7, in8, in9, in10, in11, in12, in13, in14]
        rtSum, runtimes, nextPrint = 0, [0, ] * 512, monotonic_time() + 1.0
        while True:
            start = monotonic_time()

            for slaveDesc in slaveDescs:
                outdata = indata
                indatatmp = master.runSlave(slaveDesc, outdata)
                if indatatmp is not None:
                    indata = indatatmp

            end = monotonic_time()
            runtimes.append(end-start)
            rtSum = rtSum - runtimes.pop(0) + runtimes[-1]
            if end>nextPrint:
                nextPrint = end + 3.0
		sys.stderr.write("pyprofibus cycle time = %.3f ms\n" %\
			(rtSum / len(runtimes) * 1000.0))
    except pyprofibus.ProfibusError as e:
    	print("Terminating: %s" % str(e))
    finally:
    	if master:
        	master.destroy()
