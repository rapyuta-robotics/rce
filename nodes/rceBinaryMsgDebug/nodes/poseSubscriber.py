#!/usr/bin/env python
import roslib; roslib.load_manifest('rceBinaryMsgDebug')
import rospy

from geometry_msgs.msg import Pose2D

import random

def callback(data):
	print data

if __name__ == '__main__':
	rospy.init_node('Pose2DSubscriber')
	rospy.Subscriber('Pose2D', Pose2D, callback)
	rospy.spin()
