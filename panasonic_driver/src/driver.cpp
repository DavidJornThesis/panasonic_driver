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
   
    //std::string local_path;
    /*
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
    */

    // Define sequence of points were the robot has to move (joint space)
    //vector<double> group_variable_values;
    //vector<visualization_msgs::Marker> markerVec;

    //moveit::planning_interface::MoveGroup group("manipulator");
    //moveit_msgs::PlanningScene planning_scene;
    //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;          
            
     /////////////////////////////////////////////////////////////////////////   

    /* 
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
    //sleep(5.0);

    
    
    ros::init(argc, argv, "driver");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    sleep(15.0);

    ofstream outf("/home/david/workspace_driver/src/panasonic_driver/files/text.csr",ios::app);

    //Writing the desciption-part of the csr file
    outf << "[Description]" << endl;
    outf << "Robot, VR-006L" << endl;
    outf << "Comment," << endl;
    outf << "Mechanism," << endl;
    outf << "Tool, " << endl;
    outf << "Creator, robot" << endl;
    outf << "Update, " << endl;
    outf << "Original, " << endl;
    outf << "Edit, 0" << endl;
    outf << "" << endl;
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
    outf<< "P1, AJ," << joint0 << "0.00, 0.00, 0.00, 0.00, 0.00" << endl;

    sleep(2.0);

    //group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

    ROS_INFO("Visualizing (joint space goal 1) %s",success?"":"FAILED");
    group_variable_values[2]= 0.20;
    double joint3 = roundf((group_variable_values[2]*18.00)/3.14);
    group.setJointValueTarget(group_variable_values);
    success = group.plan(my_plan);
    
    outf<< "P2, AJ, 0.00, 0.00," << joint3 << "0.00, 0.00, 0.00" << endl;
    outf << "" << endl;
    outf << "" << endl;
    

    /*
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name;

    if(!n.getParam("planning_plugin",planner_plugin_name))
        ROS_FATAL_STREAM("Could not find planner plugin name");
    try
    {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planjing_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader"<< ex.what());
    }
    try
    {
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
            if(!planner_instance->initialize(robot_model, n.getNamespace()))
                ROS_FATAL_STREAM("Could not initialize planner instance");
                ROS_INFO_STREAM("Using planning interface '"<< planner_instance->getDescription()<< "'");
    }
    catch(pluginlib::PluginlibException& ex)
    {
        const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for(std::size_t i=0; i<classes.size(); i++)
            ss << classes[i] << "";
            ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "':"<< ex.what()<< std::endl
                << "Available plugins:" << ss.str());
    }
    
    sleep(15.0);

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "link5";
    pose.pose.position.x = 0.75;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;

    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);

    req.group_name = "manipulator";
    moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("link5", pose, tolerance_pose, tolerance_angle);
    req.goal_constraints.push_back(pose_goal);

    planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    context->solve(res);
    if(res.error_code_.val != res.error_code_.SUCCESS)
    {
        ROS_ERROR("Could not compute plan successfully");
        return 0;
    }

    ros::Publisher display_publisher = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    ROS_INFO("Visualizing the trajectory");
    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);

    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    display_publisher.publish(display_trajectory);

    sleep(5.0);


    // joint space goals
    /*
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    req.group_name = "manipulator";

    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);

    robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
    const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("manipulator");
    robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
    
    robot_state::RobotState goal_state(robot_model);
    std::vector<double> joint_values(5,0.0);
    joint_values[0] = -1.0;
    joint_values[1] = 0.3;
    joint_values[2] = 0.2;
    joint_values[3] = 0.8;
    joint_values[4] = 1.0;
    goal_state.setJointGroupPositions(joint_model_group, joint_values);
    moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

    req.goal_constraints.clear();
    req.goal_constraints.push_back(joint_goal);

    planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    context->solve(res);

    if(res.error_code_.val != res.error_code_.SUCCESS)
    {
        ROS_ERROR("Could not compute plan successfully");
        return 0;
    }
    
    ROS_INFO("Visualizing the trajectory");
    

    ros::Publisher display_publisher = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    display_publisher.publish(display_trajectory);

    */
    //Writing the pose-part of the csr file
    
    outf << "[Command]" << endl;
    outf << "TOOL," << endl;
    outf << ":LABL0001" << endl;
    outf << "MOVEP,P1,10.00,m/min,N" << endl;
    outf << "MOVEP,P2,10.00,m/min,N" << endl;
           
    outf.close();

    sleep(5.0);
    ROS_INFO("Done");
    ros::shutdown();
    return 0;
}
