#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       StringTester.py
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

import string
import random
import time

import roslib; roslib.load_manifest('Test')

from Test.srv import StringEcho, StringTest, StringTestResponse
from Test.msg import StringData
import rospy


class TestCenter(object):
    def __init__(self):
        self._data = []
    
    def registerData(self, data):
        self._data = data.size
    
    def runTest(self, req):
        name = req.testName
        resp = StringTestResponse()
        rospy.wait_for_service(name, timeout=5)
        
        serviceFunc = rospy.ServiceProxy(name, StringEcho)
        
        for size in self._data:
            s = ''.join(random.choice(string.lowercase) for _ in xrange(size))
            start = time.time()
            response = serviceFunc(s)
            end = time.time()
            
            if response.data != s:
                resp.times.append(-1)
            else:
                 resp.times.append(end-start)
            
            resp.sizes.append(size)
        
        return resp

def string_tester_server():
    rospy.init_node('stringTesterNode')
    
    testCenter = TestCenter()
    rospy.Subscriber('stringData', StringData, testCenter.registerData)
    rospy.Service('stringTest', StringTest, testCenter.runTest)
    rospy.spin()

if __name__ == "__main__":
    string_tester_server()
