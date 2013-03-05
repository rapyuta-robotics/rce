#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     stringTester.py
#     
#     This file is part of the RoboEarth Cloud Engine test.
#     
#     This file was originally created for RoboEearth
#     http://www.roboearth.org/
#     
#     The research leading to these results has received funding from
#     the European Union Seventh Framework Programme FP7/2007-2013 under
#     grant agreement no248942 RoboEarth.
#     
#     Copyright 2012 RoboEarth
#     
#     Licensed under the Apache License, Version 2.0 (the "License");
#     you may not use this file except in compliance with the License.
#     You may obtain a copy of the License at
#     
#     http://www.apache.org/licenses/LICENSE-2.0
#     
#     Unless required by applicable law or agreed to in writing, software
#     distributed under the License is distributed on an "AS IS" BASIS,
#     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#     See the License for the specific language governing permissions and
#     limitations under the License.
#     
#     \author/s: Dominique Hunziker 
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
    TIMEOUT = 8
    
    def __init__(self):
        self._data = []
        self._times = []
        
        self._counter = 0
        self._event = None
        self._pub = None
        self._str = None
        self._time = None
        
        rospy.Subscriber('stringEchoResp', String, self._resp)
    
    def registerData(self, data):
        self._data = data.size
    
    def _runService(self, name):
        name = '{0}Service'.format(name)
        
        try:
            rospy.wait_for_service(name, timeout=5)
        except rospy.ROSException:
            return [-1.0]*len(self._data)
        
        times = []
        
        serviceFunc = rospy.ServiceProxy(name, StringEcho)
        
        for size in self._data:
            if size > 10:
                sample = ''.join(random.choice(string.lowercase)
                                 for _ in xrange(10))
                s = sample*int(size/10)+'A'*(size % 10)
            else:
                s = ''.join(random.choice(string.lowercase)
                            for _ in xrange(size))
            
            start = time.time()
            response = serviceFunc(s)
            end = time.time()
            
            if response.data != s:
                times.append(-1)
            else:
                times.append((end-start)*1000)
        
        return times
    
    def _req(self):
        if self._counter >= len(self._data):
            self._event.set()
        else:
            if self._data[self._counter] > 10:
                sample = ''.join(random.choice(string.lowercase)
                                 for _ in xrange(10))
                rep = int(self._data[self._counter]/10)
                tail = 'A'*(self._data[self._counter] % 10)
                
                self._str = sample*rep+tail
            else:
                self._str = ''.join(random.choice(string.lowercase)
                                    for _ in xrange(self._data[self._counter]))
            
            self._time = time.time()
            self._pub.publish(self._str)
    
    def _resp(self, msg):
        stop = time.time()
        
        if not self._pub:
            return
        
        if msg.data == self._str:
            self._times.append((stop-self._time)*1000)
            self._time = None
            self._counter += 1
            self._req()
    
    def _runTopic(self, name):
        self._counter = 0
        self._times = []
        self._event = Event()
        self._pub = rospy.Publisher('{0}Req'.format(name), String, latch=True)
        self._req()
        
        if (not self._event.wait(self.TIMEOUT) or
                len(self._times) != len(self._data)):
            times = [-1.0-1.0*len(self._times)]*len(self._data)
        else:
            times = self._times[:]
        
        self._pub = None
        return times
    
    def runTest(self, req):
        test = req.testType
        name = req.testName
        
        if test == 'service':
            times = self._runService(name)
        elif test == 'topic':
            times = self._runTopic(name)
        
        return self._data, times


def string_tester_server():
    rospy.init_node('stringTesterNode')
    
    testCenter = TestCenter()
    rospy.Subscriber('stringData', StringData, testCenter.registerData)
    rospy.Service('stringTest', StringTest, testCenter.runTest)
    rospy.spin()


if __name__ == "__main__":
    string_tester_server()
