#!/usr/bin/env python
import roslib; roslib.load_manifest('rceBinaryMsgDebug')
import rospy

import cv

from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class imageAnalyzer(object):
    
  def __init__(self):
    self.bridge = CvBridge()
    
    # Estimated circle center point pulisher
    self.circleCenterPublisher = rospy.Publisher("est_ccPos",Pose2D) 
    
  def callback(self,data):
    try:
        img = self.bridge.imgmsg_to_cv(data, "mono8")
    except CvBridgeError, e:
        print e

    moments = cv.Moments(img)
    
    data = Pose2D()
    m00 = cv.GetSpatialMoment(moments, 0, 0)
    data.x = cv.GetSpatialMoment(moments, 1, 0) / m00
    data.y = cv.GetSpatialMoment(moments, 0, 1) / m00
    data.theta = 0.0
    
    self.circleCenterPublisher.publish(data)
    
    print data.x / 500
    print data.y / 500
    

if __name__ == "__main__":
    rospy.init_node('image_converter')
    
    ia = imageAnalyzer()
    rospy.Subscriber('circleIn', Image, ia.callback)

    rospy.spin()
