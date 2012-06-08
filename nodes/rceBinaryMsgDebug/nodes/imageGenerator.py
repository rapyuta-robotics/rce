#!/usr/bin/env python
import roslib
import roslib; roslib.load_manifest('rceBinaryMsgDebug')

import sys
import rospy
import cv
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class imageGenerator:

    def __init__(self):
        self.image_pub = rospy.Publisher("image_generated",Image)

    def callback(self,data):
        print data
    
if __name__ == '__main__':
    rospy.init_node('Pose2DSubscriber')
    ig = imageGenerator()
    rospy.Subscriber('Pose2D', Pose2D, ig.callback)
    rospy.spin()
