#!/usr/bin/env python

import rospy 
import math
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

def callback(msg):

    localGoal = msg.poses[1]
    pathIndexLength = len(msg.poses)

    currentX = odomPose.pose.position.x
    currentY = odomPose.pose.position.y

    fianlX = msg.poses[pathIndexLength-1].pose.position.x
    finalY = msg.poses[pathIndexLength-1].pose.position.y

    distanceToFinalGoal = math.sqrt((fianlX-currentX)**2 + (finalY-currentY)**2)

    if (distanceToFinalGoal <= 1.0):

        localGoalPublisher.publish(msg.poses[pathIndexLength-1])

    else:

        for position in msg.poses:

            localX = position.pose.position.x
            localY = position.pose.position.y

            distanceToGoal = math.sqrt((localX-currentX)**2 + (localY-currentY)**2)

            if (distanceToGoal >= 1.0):

                localGoalPublisher.publish(position)
                break

    print(localGoal)


def odomCallback(msg):
    
    odomPose.pose = msg.pose.pose




#while(rospy.is_shutdown() == False):
if __name__ == '__main__':

    rospy.init_node("getLocalGoal", anonymous=False)

    odomPose = PoseStamped()
    odomPose.header.frame_id = 'map'

    navigationPathSubscriber = rospy.Subscriber("/move_base/NavfnROS/plan", Path, callback)
    odomertySubscriber = rospy.Subscriber("/odom", Odometry, odomCallback)
    localGoalPublisher = rospy.Publisher('/local_goal', PoseStamped, queue_size=10)

    rospy.spin()

