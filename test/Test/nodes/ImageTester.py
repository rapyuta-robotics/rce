#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       ImageTester.py
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
#       \author/s: Dominique Hunziker 
#       
#       

import time

import roslib; roslib.load_manifest('Test')

from Test.msg import TestData
from Test.srv import ImgEcho, Testrun, TestrunResponse
import sensor_msgs.msg
import rospy


class TestCenter(object):
    def __init__(self):
        self._data = []
        self._tests = {}
    
    def registerData(self, data):
        print len(data.imgs)
        print data.imgNames
        self._data = zip(data.imgNames, data.imgs)
    
    def runTest(self, req):
        name = req.testName
        rospy.wait_for_service(name, timeout=5)
        resp = TestrunResponse()
        
        serviceFunc = rospy.ServiceProxy(name, ImgEcho)
        
        for imgName, img in self._data:
            start = time.time()
            response = serviceFunc(img)
            end = time.time()
            
            resp.imgNames.append(imgName)
            resp.times.append(end-start)
        
        return resp

def iamge_echo_server():
    rospy.init_node('imageTesterNode')
    
    testCenter = TestCenter()
    
    rospy.Subscriber('imageData', TestData, testCenter.registerData)
    rospy.Service('imageTest', Testrun, testCenter.runTest)
    rospy.spin()

if __name__ == "__main__":
    iamge_echo_server()
