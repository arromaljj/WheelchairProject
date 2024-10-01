// include header file for empty planner class 
#include "EmptyLocalPlanner.h"
// include header for plugin registration
#include <pluginlib/class_list_macros.h>
#include <cstddef>


// Regsiter empty planner class as plugin -- make available to ROS 

PLUGINLIB_EXPORT_CLASS(empty_local_planner::EmptyLocalPlanner, nav_core::BaseLocalPlanner)

namespace empty_local_planner{

// initialse memeber variables fpr EmptyLocalPlanner  
EmptyLocalPlanner::EmptyLocalPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

// init planner with ROS objects 
void EmptyLocalPlanner::initialize(std::string, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){
    //check if already init 
    if(!initialized_){
        // store the provided ROS objects for later use 
        tf_ = tf;
        //set inital flag to true
        initialized_ = true;
  }
}
bool EmptyLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& /*plan*/) {
    return true; // always true, no velocity commands will be generated. 
}
bool EmptyLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
    // sets all velocity components to 0 for empty movement 
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    return true; // indicates a success 
}
// check if goal is reached -- return false
bool EmptyLocalPlanner::isGoalReached(){
    return false;
    //can be changed to true so it sees move_base as the goal == immediately reached
    // but i think that might fuck with some things 
}
// Register this planner as a plugin for ROS navigation.
PLUGINLIB_EXPORT_CLASS(empty_local_planner::EmptyLocalPlanner, nav_core::BaseLocalPlanner)



