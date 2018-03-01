/* this cpp-file implements the ros-driver for the Panasonic robot
*
*/

#include <iostream>
#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <string.h>
#include <math.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

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


bool writeTrajectoryFile = true;

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

    sleep(10.0);

    std::string local_path;
    
    //save results in a textfile
    if(n.getParam("/local_path",local_path)){
        ROS_INFO_STREAM("Local path found:" << local_path);
    }
    else
    {
        ROS_ERROR("Path not found");
    }
    
    std::string bagReadFilePath = local_path;


    //Get settings from launch file

    if (!n.getParam("/writeTrajectoryFile", writeTrajectoryFile)){
        ROS_WARN_STREAM("Writetrajectoryfile parameter not found, using default: " << writeTrajectoryFile);
    }


    // Define sequence of points were the robot has to move (joint space)
    //vector<double> group_variable_values;
    //vector<visualization_msgs::Marker> markerVec;

    moveit::planning_interface::MoveGroup group("manipulator");
    moveit_msgs::PlanningScene planning_scene;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;          
            
     /////////////////////////////////////////////////////////////////////////   


    double i,j,k,l,m;

    cout << "Put in the value of joint RT (in radians)\n";
    cin >> i;
    cout << "Put in the value of joint UA (in radians)\n";
    cin >> j;
    cout << "Put in the value of joint FA (in radians)\n";
    cin >> k;
    cout << "Put in the value of joint RW (in radians)\n";
    cin >> l;
    cout << "Put in the value of joint BW (in radians)\n";
    cin >> m;

    //moveit::planning_interface::MoveGroup group("manipulator");

    //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::Publisher display_publisher = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path",1,true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);
    
    
    std::vector<double> group_variable_values; 
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
    

    group_variable_values[0] = i;
    group_variable_values[1] = j;
    group_variable_values[2] = k;
    group_variable_values[3] = l;
    group_variable_values[4] = m;

    group.setJointValueTarget(group_variable_values);
    
    success = group.plan(my_plan);

    ROS_INFO("Visualizing plan (joint space goal) (all joints go to new position) %s",success?"":"FAILED");
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);

    /*
    write_to_csr_description();
    write_to_csr_pose();
    write_to_csr_command();
    */
    //tekst();
    ros::shutdown();
    return 0;
}
