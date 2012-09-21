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
from threading import Event

import roslib; roslib.load_manifest('Test')
import rospy
from std_msgs.msg import String

from Test.srv import StringEcho, StringTest
from Test.msg import StringData


class TestCenter(object):
    def __init__(self):
        self._data = []
        self._times = []
        
        self._counter = 0
        self._event = None
        self._pub = None
        self._strs = {}
    
    def registerData(self, data):
        self._data = data.size
    
    def _runService(self, name):
        name = '{0}Service'.format(name)
        rospy.wait_for_service(name, timeout=5)
        
        serviceFunc = rospy.ServiceProxy(name, StringEcho)
        
        for size in self._data:
            s = ''.join(random.choice(string.lowercase) for _ in xrange(size))
            start = time.time()
            response = serviceFunc(s)
            end = time.time()
            
            if response.data != s:
                self._times.append(-1)
            else:
                self._times.append((end-start)*1000)
    
    def _req(self):
        if self._counter >= len(self._data):
            self._event.set()
        else:
            s = ''.join(random.choice(string.lowercase)
                        for _ in xrange(self._data[self._counter]))
            start = time.time()
            self._pub.publish(s)
            self._strs[s] = start
    
    def _resp(self, msg):
        stop = time.time()
        
        if not self._pub:
            return
        
        if msg.data not in self._strs:
            self._times.append(-1)
        else:
            self._times.append((stop-self._strs[msg.data])*1000)
        
        self._counter += 1
        self._req()
    
    def _runTopic(self, name):
        self._counter = 0
        self._event = Event()
        self._pub = rospy.Publisher('{0}Req'.format(name), String, latch=True)
        rospy.Subscriber('stringEchoResp'.format(name), String, self._resp)
        self._req()
        self._event.wait()
        self._pub = None
    
    def runTest(self, req):
        test = req.testType
        name = req.testName
        
        self._times = []
        
        if test == 'service':
            self._runService(name)
        elif test == 'topic':
            self._runTopic(name)
        
        return self._data, self._times

def string_tester_server():
    rospy.init_node('stringTesterNode')
    
    testCenter = TestCenter()
    rospy.Subscriber('stringData', StringData, testCenter.registerData)
    rospy.Service('stringTest', StringTest, testCenter.runTest)
    rospy.spin()

if __name__ == "__main__":
    string_tester_server()
