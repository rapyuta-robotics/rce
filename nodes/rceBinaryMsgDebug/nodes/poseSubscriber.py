#!/usr/bin/env python
import roslib; roslib.load_manifest('rceBinaryMsgDebug')
import rospy
from geometry_msgs.msg import Pose2D

def callback(data):
	print data

if __name__ == '__main__':
	rospy.init_node('ccPosSubscriber') #circle center position subscriber
	rospy.Subscriber('est_ccPos', Pose2D, callback)
	rospy.spin()
