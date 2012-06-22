#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       imageGenerator.py
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
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge, CvBridgeError

class imageGenerator(object):

    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("circleOut",Image)
        
        self.imgDim = (500,500)
        self.img = cv.CreateImage(self.imgDim,cv.IPL_DEPTH_8U,1)

    def callback(self,data):
        posX = self.imgDim[0]*data.x
        posY = self.imgDim[1]*data.y
        
        cv.Circle(self.img, (int(posX),int(posY)), int(self.imgDim[0]*0.1), 255, -1)
        
        try:
            self.image_pub.publish(self.bridge.cv_to_imgmsg(self.img, "mono8"))
        except CvBridgeError, e:
            print e
        cv.Set(self.img,0)
        
if __name__ == '__main__':
    rospy.init_node('Pose2DSubscriber')
    ig = imageGenerator()
    rospy.Subscriber('ccPos', Pose2D, ig.callback)
    rospy.spin()
