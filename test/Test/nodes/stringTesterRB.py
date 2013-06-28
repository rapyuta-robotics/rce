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
import uuid

import roslib; roslib.load_manifest('Test')
import rospy
from std_msgs.msg import String

from Test.srv import StringEcho, StringTest
from Test.msg import StringData

from twisted.internet import reactor
from autobahn.websocket import WebSocketClientFactory, \
                               WebSocketClientProtocol, \
                               connectWS
import json


class TestCenter(WebSocketClientProtocol):
    TIMEOUT = 8
    
    def __init__(self):
        self._data = [3, 10, 50, 100, 500, 1000, 5000,
                        10000, 50000, 100000, 500000]
        self._times = {}
        
        self._counter = 0
        self._event = None
        self._pub = None
        self._str = None
        self._time = None
        
        rospy.Service('stringTest', StringTest, self.runTest)
    
    def _runService(self, name):
        name = '{0}Service'.format(name)
        
        for size in self._data:
            a = uuid.uuid4()
            if size > 10:
                sample = ''.join(random.choice(string.lowercase)
                                 for _ in xrange(10))
                s = sample*int(size/10)+'A'*(size % 10)
            else:
                s = ''.join(random.choice(string.lowercase)
                            for _ in xrange(size))
            
            start = time.time()
            self._times[str(a)] = [start,0,size]
            service_call = json.dumps({'op': 'call_service', 'service': '/'+str(name), 'args': {'uid': str(a), 'data': s}})

            self.sendMessage(service_call)

        def predicate():
            h = [v[1] for k,v in self._times.items()]
            return all(i is 1 for i in h)

        def waituntil(somepredicate, period=0.25):
            while True:
                if somepredicate(): return True
                time.sleep(period)
        
        waituntil(predicate, 0.25)
        g = sorted([(v[2], v[0]) for k,v in self._times.items()])
        return [i[1] for i in g]
    
    def _req(self, name):
        for size in self._data:
            a = uuid.uuid4()
            
            if size > 10:
                sample = ''.join(random.choice(string.lowercase)
                                 for _ in xrange(10))
                rep = int(size/10)
                tail = 'A'*(size % 10)
                
                s = sample*rep+tail
            else:
                s = ''.join(random.choice(string.lowercase)
                                    for _ in xrange(size))
                
            start = time.time()
            self._times[str(a)] = [start,0,size]
            publish = json.dumps({'op': 'publish', 'topic': '/'+str(name), 'msg': {'uid': str(a), 'data': s}})

            self.sendMessage(publish)
    
    def _resp(self, msg):
        
        end = time.time()
        data = json.loads(msg)
        if not self._pub:
            return
        end = time.time()

        uid = str(data["msg"]["uid"])
        
        self._times[uid] = [end - self._times[uid][0], 1, self._times[uid][2] ]
    
    def _runTopic(self, name):
        self._pub = rospy.Publisher('{0}Req'.format(name), String, latch=True)
        self.sendMessage(json.dumps({'op': 'advertise', 'topic': '/'+'{0}Req'.format(name), 'type': 'Test/StringEchoMsg'}))
        self._req('{0}Req'.format(name))
        
        def predicate():
            h = [v[1] for k,v in self._times.items()]
            return all(i is 1 for i in h)

        def waituntil(somepredicate, period=0.25):
            while True:
                if somepredicate(): return True
                time.sleep(period)
        
        waituntil(predicate, 0.25)
        self._pub = None
        g = sorted([(v[2], v[0]) for k,v in self._times.items()])
        return [i[1] for i in g]       
        
    
    def runTest(self, req):
        test = req.testType
        name = req.testName
        final_times = []
        for i in range(10):
            if test == 'service':
                final_times.append(self._runService(name))
                
            elif test == 'topic':
                final_times.append(self._runTopic(name))
            self._times = {}
        
        return self._data, [ sum(x)/10.0 for x in zip(*final_times) ], final_times
    
    def onOpen(self):
        print 'connection'
        subscribes = json.dumps({'op': 'subscribe', 'topic': '/stringEchoResp', 'type': 'Test/StringEchoMsg'})
        self.sendMessage(subscribes)
 
    def onMessage(self, msg, binary):
        data = json.loads(msg)
        if data["op"] == "service_response":
            end = time.time()
            #print self._times
            uid = str(data["values"]["uid"])
            
            self._times[uid] = [end - self._times[uid][0], 1, self._times[uid][2] ]
        elif data["op"] == "publish":
            
            self._resp(msg)


def string_tester_server():
    rospy.init_node('stringTesterNode')
    
    factory = WebSocketClientFactory("ws://54.216.152.74:9000", debug = False)
    factory.protocol = TestCenter
    connectWS(factory)
    
    def terminate():
        reactor.callFromThread(reactor.stop)

    rospy.on_shutdown(terminate)
    reactor.run(installSignalHandlers=False)


if __name__ == "__main__":
    string_tester_server()
