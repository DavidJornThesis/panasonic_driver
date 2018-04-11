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

import sys, threading, math, copy
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
from pyprofibus import DpTelegram_SetPrm_Req, monotonic_time

##############################
rospy.init_node('driver', anonymous=True)
rate = rospy.Rate(1) # A frequency of 1Hz is chosen as a begin value


try:
    while not rospy.is_shutdown():
        # Sending and receiving data over PROFIBUS
        

    except KeyboardInterrupt:
        