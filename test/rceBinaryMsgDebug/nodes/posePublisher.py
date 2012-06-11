#!/usr/bin/env python
import roslib; roslib.load_manifest('rceBinaryMsgDebug')
import rospy

from geometry_msgs.msg import Pose2D

import random

def talker():
    pub = rospy.Publisher('ccPos', Pose2D) #circle center position
    rospy.init_node('ccPosPublisher')
    while not rospy.is_shutdown():
        pose = Pose2D()
        pose.x = 0.2 + 0.6*random.random()
        pose.y = 0.2 + 0.6*random.random()
        pose.theta = 0.0
        pub.publish(pose)
        rospy.sleep(2.0)
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
