/* this cpp-file implements the ros-driver for the Panasonic robot
*
*/

#include <iostream>
#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <string.h>
#include <math.h>

#include <pluginlib/class_loader.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <boost/scoped_ptr.hpp>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

using namespace std;

/*
A first implementation of the driver is to make a offline way of communication possible.
Herefore there is a way of converting ROS-commands to a text-file which can be send to 
the DTPS-software using a USB-stick.
*/


/*
ofstream csrfile;

void write_to_csr_description(){
    csrfile.open("programma.txt","a");
    csrfile << "[Description]\n" ;
    csrfile << "Robot, VR-006L\n" ;
    csrfile << "Comment,\n" ;
    csrfile << "Mechanism,\n" ;
    csrfile << "Tool,\n" ;
    csrfile << "Creator, robot\n" ;
    csrfile << "Update,\n" ;
    csrfile << "Original,\n" ;
    csrfile << "Edit, 0\n" ;
    csrfile.close();
}


double joint0 = roundf(group_variable_values[0]*180/3.14);
double joint1 = roundf(group_variable_values[1]*180/3.14);
double joint2 = roundf(group_variable_values[2]*180/3.14);
double joint3 = roundf(group_variable_values[3]*180/3.14);
double joint4 = roundf(group_variable_values[4]*180/3.14);

void write_to_csr_pose(){
    csrfile.open("programma.txt","a");
    csrfile << "[Pose]/n";
    csrfile << "P1,AJ, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00/n";
    csrfile << "P2,AJ," << joint0, joint1, joint2, joint3, joint4, << "0.00 \n";   
    csrfile.close();
}

void write_to_csr_command(){
    csrfile.open("programma.txt","a");
    csrfile << "[Command]\n";
    csrfile << "TOOL,\n";
    csrfile << "LABL0001\n";
    csrfile << "MOVEP,P1,10.00,m/min,0,N\n";
    csrfile << "MOVEP,P2,10.00,m/min,0,N\n";
    csrfile.close();
}


void tekst(){
    ofstream csrfile;
    csrfile.open("programma.csr",ios::app);
    csrfile << "This is a line. \n";
    csrfile.close();

}
*/


//bool writeTrajectoryFile = true;

/*//////////////////////////////
/////////     MAIN       ///////
////////////////////////////////
*/
int main(int argc, char *argv[]) 
{
   
    ros::init(argc, argv, "driver");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    sleep(15.0);

    ofstream outf("/home/david/workspace_driver/src/panasonic_driver/files/text.csr",ios::app);

    //Writing the desciption-part of the csr file
    outf << "[Description]" << endl;
    outf << "Robot, TA1400(G3)" << endl; //defining the robot in DTPS
    outf << "Comment," << endl;
    outf << "Mechanism," << endl;
    outf << "Tool, 1:TOOL01 " << endl; //defining the tool for the robot in DTPS
    outf << "Creator, DTPS" << endl;
    outf << "User coordinates, none"<< endl;
    outf << "Update, 2018, 3, 7, 9, 35 55 " << endl; //the update time
    outf << "Original, 2018, 3, 7, 9, 36, 39  " << endl; //the original time
    outf << "Edit, 0" << endl;
    outf << "" << endl;
        
    // Defining some poses to were robot has to move   

    moveit::planning_interface::MoveGroup group("manipulator");

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::Publisher display_publisher = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1);
    moveit_msgs::DisplayTrajectory display_trajectory;

    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);
    
    std::vector<double> group_variable_values;
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
    
    ROS_INFO("Visualizing (joint space goal 1) %s",success?"":"FAILED");

    group_variable_values[0] = -1.00;
    double joint0 = roundf((group_variable_values[0]*18.00)/3.14);
    group.setJointValueTarget(group_variable_values);
    success = group.plan(my_plan);

    outf<< "[Pose]"<< endl;
    outf<< "/Name, Type, RT, UA, FA, RW, BW, TW, G4, G5"<< endl;
    outf<< "P1, AJ, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00" << endl;
    outf<< "P2, AJ," << joint0 << "0.00, 0.00, 0.00, 0.00, 0.00" << endl;

    sleep(2.0);

   

    ROS_INFO("Visualizing (joint space goal 1) %s",success?"":"FAILED");
    group_variable_values[2]= 0.20;
    double joint3 = roundf((group_variable_values[2]*18.00)/3.14);
    group.setJointValueTarget(group_variable_values);
    success = group.plan(my_plan);
    
    outf<< "P3, AJ, 0.00, 0.00," << joint3 << "0.00, 0.00, 0.00" << endl;
    outf << "" << endl;
   
    //Writing the pose-part of the csr file
    
    outf << "[Command]" << endl;
    outf << "TOOL, 1:TOOL01" << endl;
    //outf << ":LABL0001" << endl;
    outf << "MOVEP,P1,10.00,m/min,N" << endl;
    outf << "MOVEP,P2,10.00,m/min,N" << endl;
    outf << "MOVEP,P3,10.00,m/min,N" << endl;

    outf.close();

    sleep(5.0);
    ROS_INFO("Done");
    ros::shutdown();
    return 0;
}
