#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     benchmark.py
#     
#     This file is part of the RoboEarth Cloud Engine benchmark.
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

import time
import random
import string
import json

from twisted.internet.defer import Deferred

import sys
sys.path.append('../pyrce')

from connection import Connection


class TestBase(object):
    def __init__(self, conn, name, style, color):
        self._conn = conn
        self._name = name
        self._style = style
        self._color = color
        
        self._deferred = []
        self._data = []
    
    def registerDeferred(self, d):
        self._deferred.append(d)
    
    def _activate(self):
        raise NotImplementedError('Test has to implement this method.')
    
    def _deactivate(self):
        raise NotImplementedError('Test has to implement this method.')
    
    def _run(self):
        raise NotImplementedError('Test has to implement this method.')
    
    def run(self, _):
        print('Run test: {0}'.format(self._name))
        self._conn.reactor.callLater(1, self._activate)
        self._conn.reactor.callLater(2, self._run)
    
    def _done(self):
        self._deactivate()
        self._deferred.pop(0).callback(None)
    
    def __str__(self):
        return json.dumps({'label' : self._name, 'linestyle' : self._style,
                           'color' : self._color, 'data' : self._data})


class RemoteTest(TestBase):
    def __init__(self, conn, name, style, color, testType, testName):
        super(RemoteTest, self).__init__(conn, name, style, color)
        
        self._testType = testType
        self._testName = testName
        
        self._srv = None
    
    def _activate(self):
        self._srv = self._conn.service('testConv', 'Test/StringTest',
                                       self._process)
    
    def _deactivate(self):
        self._srv = None
    
    def _run(self):
        self._srv.call({'testType' : self._testType,
                        'testName' : self._testName})
    
    def _process(self, msg):
        self._data.append(msg['times'])
        self._done()


class LocalTest(TestBase):
    def __init__(self, conn, name, style, color, sizes):
        super(LocalTest, self).__init__(conn, name, style, color)
        
        self._sizes = sizes
    
    def _run(self):
        self._data.append([])
        self._req()
    
    def _resp(self, resp):
        stop = time.time()
        
        if self._str != resp['data']:
            delta = -1
        else:
            delta = (stop-self._time)*1000
        
        self._data[-1].append(delta)
        self._req()


class LocalServiceTest(LocalTest):
    def __init__(self, conn, name, style, color, sizes):
        super(LocalServiceTest, self).__init__(conn, name, style, color, sizes)
        
        self._srv = None
    
    def _activate(self):
        self._srv = self._conn.service('testStringSrvConv', 'Test/StringEcho',
                                       self._resp)
    
    def _deactivate(self):
        self._srv = None
    
    def _req(self):
        count = len(self._data[-1])
        
        if count >= len(self._sizes):
            self._done()
            return
        
        self._str = ''.join(random.choice(string.lowercase)
                            for _ in xrange(self._sizes[count]))
        
        self._time = time.time()
        self._srv.call({'data' : self._str})


class LocalTopicTest(LocalTest):
    def __init__(self, conn, name, style, color, sizes):
        super(LocalTopicTest, self).__init__(conn, name, style, color, sizes)
        
        self._pub = None
        self._sub = None
    
    def _activate(self):
        self._pub = self._conn.publisher('testStringTopicReqConv',
                                         'std_msgs/String')
        self._sub = self._conn.subscriber('testStringTopicRespConv',
                                          'std_msgs/String', self._resp)
    
    def _deactivate(self):
        self._pub = None
        self._sub.unsubscribe()
        self._sub = None
    
    def _req(self):
        count = len(self._data[-1])
        
        if len(self._data[-1]) >= len(self._sizes):
            self._done()
            return
        
        self._str = ''.join(random.choice(string.lowercase)
                            for _ in xrange(self._sizes[count]))
        
        self._time = time.time()
        self._pub.publish({'data' : self._str})
    

class Benchmark(object):
    TYPES = [('service', 'red'), ('topic', 'blue')]
    REMOTE_TESTS = [('N-to-N', ':', 'stringEcho'), ('C-to-C-1', '--', 'local'),
                    ('C-to-C-2', '-', 'remote')]
    LOCAL_TEST = ('R-to-C', '-.', '')
    
    
    def __init__(self, runs, conn, reactor):
        self._runs = runs
        self._sizes = [3, 10, 50, 100, 500, 1000, 5000,
                       10000, 50000, 100000, 500000]
        
        self._conn = conn
        self._reactor = reactor
        
        self._tests = []
        
        for testType, color in self.TYPES:
            for name, style, testName in self.REMOTE_TESTS:
                self._tests.append(RemoteTest(conn,
                    '{0} {1}'.format(testType, name), style, color, testType,
                    testName))
            
            if testType == 'service':
                cls = LocalServiceTest
            elif testType == 'topic':
                cls = LocalTopicTest
            else:
                raise ValueError('Test type is invalid.')
            
            self._tests.append(
                cls(conn, '{0} {1}'.format(testType, self.LOCAL_TEST[0]),
                    self.LOCAL_TEST[1], color, self._sizes))
    
    def run(self, _):
        print('Create containers...')
        
        for cTag in ['m1c1', 'm2c1', 'm1c2']:
            self._conn.createContainer(cTag)
        
        self._reactor.callLater(30, self._setup)
    
    def _setup(self):
        print('Setup environments...')
        
        for cTag in ['m1c1', 'm2c1', 'm1c2']:
            self._conn.addNode(cTag, 'strEcho', 'Test', 'stringEcho.py', '')
        
        self._conn.addNode('m1c1', 'strTester', 'Test', 'stringTester.py', '')
        
        # Connections Robot - StringTester
        self._conn.addInterface('testRobot', 'testConv',
                                'ServiceProviderConverter', 'Test/StringTest',
                                '')
        self._conn.addInterface('m1c1', 'testInrfc', 'ServiceInterface',
                                'Test/StringTest', 'stringTest')
        self._conn.addConnection('testConv', 'testInrfc')
        
        self._conn.addInterface('testRobot', 'testDataConv',
                                'SubscriberConverter', 'Test/StringData', '')
        self._conn.addInterface('m1c1', 'testDataInrfc', 'PublisherInterface',
                                'Test/StringData', 'stringData')
        self._conn.addConnection('testDataConv', 'testDataInrfc')
        
        # Connections StringTester - StringEcho (service)
        self._conn.addInterface('m1c2', 'testLocalSrvRecv', 'ServiceInterface',
                                'Test/StringEcho', 'stringEchoService')
        self._conn.addInterface('m1c1', 'testLocalSrvSend',
                                'ServiceProviderInterface', 'Test/StringEcho',
                                'localService')
        self._conn.addConnection('testLocalSrvRecv', 'testLocalSrvSend')
        
        self._conn.addInterface('m2c1', 'testRemoteSrvRecv',
                                'ServiceInterface', 'Test/StringEcho',
                                'stringEchoService')
        self._conn.addInterface('m1c1', 'testRemoteSrvSend',
                                'ServiceProviderInterface', 'Test/StringEcho',
                                'remoteService')
        self._conn.addConnection('testRemoteSrvRecv', 'testRemoteSrvSend')
        
        # Connections StringTester - StringEcho (topic)
        self._conn.addInterface('m1c1', 'testLocalRespSend',
                                'PublisherInterface', 'std_msgs/String',
                                'stringEchoResp')
        
        self._conn.addInterface('m1c2', 'testLocalRespRecv',
                                'SubscriberInterface', 'std_msgs/String',
                                'stringEchoResp')
        self._conn.addConnection('testLocalRespRecv', 'testLocalRespSend')
        
        self._conn.addInterface('m2c1', 'testRemoteRespRecv',
                                'SubscriberInterface', 'std_msgs/String',
                                'stringEchoResp')
        self._conn.addConnection('testRemoteRespRecv', 'testLocalRespSend')
        
        self._conn.addInterface('m1c2', 'testLocalReqSend',
                                'PublisherInterface', 'std_msgs/String',
                                'stringEchoReq')
        self._conn.addInterface('m1c1', 'testLocalReqRecv',
                                'SubscriberInterface', 'std_msgs/String',
                                'localReq')
        self._conn.addConnection('testLocalReqRecv', 'testLocalReqSend')
        
        self._conn.addInterface('m2c1', 'testRemoteReqSend',
                                'PublisherInterface', 'std_msgs/String',
                                'stringEchoReq')
        self._conn.addInterface('m1c1', 'testRemoteReqRecv',
                                'SubscriberInterface', 'std_msgs/String',
                                'remoteReq')
        self._conn.addConnection('testRemoteReqRecv', 'testRemoteReqSend')
        
        # Connections Robot - StringEcho (service)
        self._conn.addInterface('testRobot', 'testStringSrvConv',
                                'ServiceProviderConverter', 'Test/StringEcho',
                                '')
        self._conn.addInterface('m1c1', 'testStringSrvInrfc',
                                'ServiceInterface', 'Test/StringEcho',
                                'stringEchoService')
        self._conn.addConnection('testStringSrvConv', 'testStringSrvInrfc')
        
        # Connections Robot - StringEcho (topic)
        self._conn.addInterface('testRobot', 'testStringTopicReqConv',
                                'SubscriberConverter', 'std_msgs/String',
                                '')
        self._conn.addConnection('testLocalReqSend',
                                 'testStringTopicReqConv')
        
        self._conn.addInterface('testRobot', 'testStringTopicRespConv',
                                'PublisherConverter', 'std_msgs/String',
                                '')
        self._conn.addConnection('testLocalRespRecv',
                                 'testStringTopicRespConv')
        
        self._reactor.callLater(5, self._load)
    
    def _load(self):
        self._conn.publisher('testDataConv', 'Test/StringData').publish(
            {'size' : self._sizes})
        
        if self._tests:
            self._reactor.callLater(2, self._run)
    
    def _run(self):
        if not self._tests:
            return
        
        i = 0
        d = Deferred()
        
        while i < self._runs:
            for test in self._tests:
                d.addCallback(test.run)
                d = Deferred()
                test.registerDeferred(d)
            
            i += 1
            
            if i >= self._runs:
                d.addCallback(self._processData)
        
        self._tests[0].run(None)
    
    def _processData(self, _):
        with open('benchmark.data', 'w') as f:
            f.write(json.dumps(self._sizes))
            f.write('\n')
            f.write('\n'.join(str(test) for test in self._tests))
        
        for cTag in ['m1c1', 'm2c1', 'm1c2']:
            self._conn.destroyContainer(cTag)
        
        self._reactor.callLater(2, self._reactor.stop)
        print('\ndone')


def _get_argparse():
    from argparse import ArgumentParser

    parser = ArgumentParser(prog='benchmark',
                            description='Run communication benchmark using a '
                                        'string message for RCE.')

    parser.add_argument('passes', help='Number of passes to do.',
                        type=int)
    parser.add_argument('ipMaster', help='IP address of master process.',
                        type=str)

    return parser
    

def main(reactor, passes, ip):
    connection = Connection('testUser', 'testRobot', reactor)
    benchmark = Benchmark(passes, connection, reactor)
    
    print('Connect...')
    deferred = Deferred()
    deferred.addCallback(benchmark.run)
    connection.connect('ws://{0}:9000/'.format(ip), deferred)
    
    reactor.run()


if __name__ == '__main__':
    from twisted.internet import reactor
    
    args = _get_argparse().parse_args()
    
    main(reactor, args.passes, args.ipMaster)
