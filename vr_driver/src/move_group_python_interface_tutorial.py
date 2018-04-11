#!/usr/bin/env python
########################################################################
# The Move group tutorial in python for the Panasonic vr-006L manipulator
########################################################################

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

# initialize moveit_commander and rospy

def move_group_python_interface_tutorial():

 print "================ Starting tutorial setup"
 moveit_commander.roscpp_initialize(sys.argv)
 rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

 robot = moveit_commander.RobotCommander()
 scene = moveit_commander.PlanningSceneInterface()
 group = moveit_commander.MoveGroupCommander("arm")

 display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                               moveit_msgs.msg.DisplayTrajectory)

 print "================ Waiting for RViz..."
 rospy.sleep(10)
 print "================ Starting Tutorial"

 print "============ Reference frame: %s" % group.get_planning_frame()
 print "============ Robot Groups:"
 print robot.get_group_names()
 print "==========================="

 print "============ Printing robot state"
 print robot.get_current_state()
 print "============"

 print "============ Generating plan 1"
 pose_target = geometry_msgs.msg.Pose()

 #pose_target.orientation.x = 1.0
 pose_target.position.x = 0.07
 pose_target.position.y = -0.05
 pose_target.position.z = 0.1
 
 group.set_pose_target(pose_target)

 plan1 = group.plan()

 print "============ Waiting while RVIZ displays plan1..."
 rospy.sleep(5)

 print "============ Visualizing plan1"
 display_trajectory = moveit_msgs.msg.DisplayTrajectory()

 display_trajectory.trajectory_start = robot.get_current_state()
 display_trajectory.trajectory.append(plan1)
 display_trajectory_publisher.publish(display_trajectory)

 print "============ Waiting while plan1 is visualized (again)..."
 rospy.sleep(5)

 print "====================== End of tutorial"


 moveit_commander.roscpp_shutdown()

if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass

