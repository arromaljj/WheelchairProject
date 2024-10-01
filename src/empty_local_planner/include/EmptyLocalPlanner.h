
// header gaurd to prevent multiple inclusions
#ifndef EMPTY_LOCAL_PLANNER_H_
#define EMPTY_LOCAL_PLANNER_H_


// include base class definition for local planners 
#include <nav_core/base_local_planner.h>

// ros related headers 
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>

// message type headers for pose stamped and twist 
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

// name space for local planner class
namespace empty_local_planner {

//class define for local planner inheriting from baselocalplanner 
 class EmptyLocalPlanner : public nav_core::BaseLocalPlanner{
 public:
    EmptyLocalPlanner();
        //initialise the planner with ROS objects
        void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

        // recieve and store global plan (override ro return true )
        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& /*plan*/)override;

        // compute velocity commands, overriden to true without commands
        bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
        

        // check goal is reached, always return false 
        bool isGoalReached() override;

    private:
        // member variables to store ROS objects 
        costmap_2d::Costmap2DROS* costmap_ros_;
        tf2_ros::Buffer* tf_;
        bool initialized_;

};
}

#endif
