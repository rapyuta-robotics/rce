#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       posePublisher.py
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
