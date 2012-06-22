#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       imageAnalyzer.py
#       
#       This file is part of the RoboEarth Cloud Engine tests.
#       
#       This file was originally created for RoboEearth
#       http://www.roboearth.org/
#       
#       The research leading to these results has received funding from
#       the European Union Seventh Framework Programme FP7/2007-2013 under
#       grant agreement no248942 RoboEarth.
#       
#       Copyright 2012 RoboEarth
#       
#       Licensed under the Apache License, Version 2.0 (the "License");
#       you may not use this file except in compliance with the License.
#       You may obtain a copy of the License at
#       
#       http://www.apache.org/licenses/LICENSE-2.0
#       
#       Unless required by applicable law or agreed to in writing, software
#       distributed under the License is distributed on an "AS IS" BASIS,
#       WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#       See the License for the specific language governing permissions and
#       limitations under the License.
#       
#       \author/s: Gajamohan Mohanarajah 
#       
#       

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
