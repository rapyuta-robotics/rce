#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     benchmark.py
#     
#     This file is part of the RoboEarth Cloud Engine framework.
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

import sys
import os.path
import time
from StringIO import StringIO

import sensor_msgs.msg

from twisted.internet.defer import Deferred

from client import Connection
from util.loader import Loader
from util.converter import Converter
from util.converters.image import ImageConverter


class Benchmark(object):
    def __init__(self, conn, reactor):
        self._conn = conn
        self._reactor = reactor
        
        self._imgNames = []
        self._imgs = []
        self._counter = 0
        self._time = None
        
        self._state = 0
        self._desc = ['inside container', 'inside machine', 'inside RCE',
                      'outside']
        self._testNames = ['imageEcho', 'local', 'remote']
    
    def run(self, _):
        print('Create containers...')
        
        for cTag in ['m1c1', 'm2c1', 'm1c2']:
            self._conn.createContainer(cTag)
        
        self._reactor.callLater(30, self._setup)
    
    def _setup(self):
        print('Setup environments...')
        
        for cTag in ['m1c1', 'm2c1', 'm1c2']:
            self._conn.addNode(cTag, 'imgEcho', 'Test', 'ImageTest.py', '')
        
        self._conn.addNode('m1c1', 'imgTester', 'Test', 'ImageTester.py', '')
        
        self._conn.addInterface('testRobot', 'testrunConv',
                                'ServiceProviderConverter', 'Test/Testrun', '')
        self._conn.addInterface('m1c1', 'testrunInrfc', 'ServiceInterface',
                                'Test/Testrun', 'imageTest')
        self._conn.addConnection('testrunConv', 'testrunInrfc')
        
        self._conn.addInterface('m1c2', 'testLocalRecv', 'ServiceInterface',
                                'Test/ImgEcho', 'imageEcho')
        self._conn.addInterface('m1c1', 'testLocalSend',
                                'ServiceProviderInterface', 'Test/ImgEcho',
                                self._testNames[1])
        self._conn.addConnection('testLocalRecv', 'testLocalSend')
        
        self._conn.addInterface('m2c1', 'testRemoteRecv', 'ServiceInterface',
                                'Test/ImgEcho', 'imageEcho')
        self._conn.addInterface('m1c1', 'testRemoteSend',
                                'ServiceProviderInterface', 'Test/ImgEcho',
                                self._testNames[2])
        self._conn.addConnection('testRemoteRecv', 'testRemoteSend')
        
        self._conn.addInterface('testRobot', 'testImageConv',
                                'ServiceProviderConverter', 'Test/ImgEcho', '')
        self._conn.addInterface('m1c1', 'testImageInrfc', 'ServiceInterface',
                                'Test/ImgEcho', 'imageEcho')
        self._conn.addConnection('testImageConv', 'testImageInrfc')
        
        self._conn.addInterface('testRobot', 'testDataConv',
                                'SubscriberConverter', 'Test/TestData', '')
        self._conn.addInterface('m1c1', 'testDataInrfc', 'PublisherInterface',
                                'Test/TestData', 'imageData')
        self._conn.addConnection('testDataConv', 'testDataInrfc')
        
        self._reactor.callLater(2, self._load)
    
    def _load(self):
        for nr in [10, 20, 30, 40, 50, 60, 70, 80, 90, 99]:
            name = 'logo_{:03}.png'.format(nr)
            img = StringIO()
            
            with open(name, 'r') as f:
                img.write(f.read())
            
            self._imgNames.append(name)
            self._imgs.append(img)
        
        self._conn.publish('testDataConv', 'Test/TestData',
                           {'imgNames' : self._imgNames, 'imgs' : self._imgs})
        
        self._reactor.callLater(2, self._sendReq)
    
    def _sendReq(self):
        print('\nStart benchmark part {0} '
              '({1}).'.format(self._state+1, self._desc[self._state]))
        
        self._conn.callService('testrunConv', 'Test/Testrun',
                               {'testName' : self._testNames[self._state]},
                               self._gotResp)
    
    def _gotResp(self, resp):
        print('Benchmark results part {0} '
              '({1}):'.format(self._state+1, self._desc[self._state]))
        
        for name, time in zip(resp['imgNames'], resp['times']):
            print('\t{0}: {1:>8.03f} [ms]'.format(name, time*1000))
        
        self._state += 1
        
        if self._state == len(self._testNames):
            self._outsideReq()
        else:
            self._sendReq()
    
    def _outsideReq(self):
        if not self._counter:
            print('\nStart benchmark part {0} '
                  '({1}).'.format(self._state+1, self._desc[self._state]))
        
        self._time = time.time()
        self._conn.callService('testImageConv', 'Test/ImgEcho',
                               {'img' : self._imgs[self._counter]},
                               self._outsideResp)
    
    def _outsideResp(self, resp):
        stop = time.time()
        
        if not self._counter:
            print('Benchmark results part {0} '
                  '({1}):'.format(self._state+1, self._desc[self._state]))
        
        print('\t{0}: {1:>8.03f} [ms]'.format(self._imgNames[self._counter],
                                              (stop-self._time)*1000))
        
        self._counter += 1
        
        if self._counter < len(self._imgs):
            self._outsideReq()
        else:
            self._conversion()
    
    def _conversion(self):
        l = Loader()
        c = Converter(l)
        c.addCustomConverter(ImageConverter)
        
        print('\nBenchmark results part {0} '
              '(PNG <-> ROS Message converter):'.format(self._state+1))
        
        for img, name in zip(self._imgs, self._imgNames):
            start = time.time()
            ros = c.decode(sensor_msgs.msg.Image, img)
            im = c.encode(ros)
            stop = time.time()
            
            print('\t{0}: {1:>8.03f} [ms]'.format(name, (stop-start)*1000))
        
        print('\ndone')


def main():
    h = 'Usage: {0} [ipMaster]'.format(os.path.basename(sys.argv[0]))
    
    if len(sys.argv) != 2:
        print(h)
        return
    
    from twisted.internet import reactor
    
    connection = Connection('testUser', 'testRobot', reactor)
    benchmark = Benchmark(connection, reactor)
    
    print('Connect...')
    deferred = Deferred()
    deferred.addCallback(benchmark.run)
    connection.connect('ws://{0}:9000/'.format(sys.argv[1]), deferred)
    
    reactor.run()


if __name__ == '__main__':
    main()
