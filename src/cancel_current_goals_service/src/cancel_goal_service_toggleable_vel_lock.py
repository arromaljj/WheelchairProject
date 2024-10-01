#!/usr/bin/env python

import rospy 
from actionlib import GoalID
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import Twist, PoseStamped

def service_callback(req):

    if req.data == False:

        # toggle off twist mux lock
        twistMuxLockTogglePublisher.publish(req.data)
    
    elif req.data == True:

        zeroVel = Twist()
        zeroVel.linear.x = 0.0
        zeroVel.linear.y = 0.0
        zeroVel.linear.z = 0.0
        zeroVel.angular.x = 0.0
        zeroVel.angular.y = 0.0
        zeroVel.angular.z = 0.0

        cancelGoal = GoalID()

        emptyLocalGoal = PoseStamped()

        # cancel move_base goal and send empty local goal to DWA planner
        dwaEmptyLocalGoal.publish(emptyLocalGoal)
        moveBaseCurrentGoalCancel.publish(cancelGoal)

        # toggle twist mux lock and publish zero velocity twist command to clear any previous commands
        twistMuxLockTogglePublisher.publish(req.data)
        twistMuxZeroVelocityPublisher.publish(zeroVel)
    
    else:
        print("failed data not True or False")

    

def cancel_goals_toggle_service():

    cancelGoalsToggleService = rospy.Service('cancelGoalsToggleService', SetBool, service_callback)
    rospy.spin()

if __name__ == '__main__':

    rospy.init_node("cancel_goals_toggleable_service", anonymous=False)
    
    moveBaseCurrentGoalCancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
    dwaEmptyLocalGoal = rospy.Publisher('/local_goal', PoseStamped, queue_size=10)
    twistMuxLockTogglePublisher = rospy.Publisher('/twist_mux_stop_toggle', Bool, queue_size=10)
    twistMuxZeroVelocityPublisher = rospy.Publisher('/cancel_goals_zero_velocity', Twist, queue_size=10)
    

    cancelGoalsToggleService = cancel_goals_toggle_service()

    rospy.spin()

