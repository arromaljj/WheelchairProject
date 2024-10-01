#!/usr/bin/env python

import rospy 
from actionlib import GoalID
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import Twist, PoseStamped

def service_callback(req):

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

    # send zero velocity Twist command to clear previous velocity commands
    twistMuxZeroVelocityPublisher.publish(zeroVel)

def cancel_goals_service():

    cancelGoalsService = rospy.Service('cancelGoalsService', SetBool, service_callback)
    rospy.spin()

if __name__ == '__main__':

    rospy.init_node("cancel_goals_service_node", anonymous=False)
    
    moveBaseCurrentGoalCancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
    dwaEmptyLocalGoal = rospy.Publisher('/local_goal', PoseStamped, queue_size=10)
    twistMuxZeroVelocityPublisher = rospy.Publisher('/cancel_goals_zero_velocity', Twist, queue_size=10)
    

    cancelGoalsToggleService = cancel_goals_service()

    rospy.spin()

