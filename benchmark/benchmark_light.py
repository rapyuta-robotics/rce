#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     benchmark_light.py
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

# Python specific imports
import time
import random
import string

# twisted specific imports
from twisted.internet.defer import Deferred

# rce specific imports
from rce.client.connection import Connection


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
        return str(self._data)


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
            delta = (stop - self._time) * 1000

        self._data[-1].append(delta)
        self._req()


class LocalServiceTest(LocalTest):
    def __init__(self, conn, name, style, color, sizes):
        super(LocalServiceTest, self).__init__(conn, name, style, color, sizes)

        self._srv = None

    def _activate(self):
        self._srv = self._conn.serviceClient('test', 'Test/StringEcho',
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
        self._pub = self._conn.publisher('testReqTopic',
                                         'std_msgs/String')
        self._sub = self._conn.subscriber('testRespTopic',
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

        if self._sizes[count] > 10:
            sample = ''.join(random.choice(string.lowercase)
                             for _ in xrange(10))
            rep = int(self._sizes[count] / 10)
            tail = 'A' * (self._sizes[count] % 10)

            self._str = sample * rep + tail
        else:
            self._str = ''.join(random.choice(string.lowercase)
                                for _ in xrange(self._sizes[count]))

        self._time = time.time()
        self._pub.publish({'data' : self._str})


class Benchmark(object):
    TYPES = [('service', 'red'), ('topic', 'blue')]
    LOCAL_TEST = ('R-to-C', '-.', '')

    def __init__(self, runs, conn, robot, reactor):
        self._runs = runs
        self._sizes = [3, 10, 50, 100, 500, 1000, 5000,
                       10000, 50000, 100000, 500000]
        #self._sizes = [3, 10, 100, 1000, 10000, 100000, 500000,
        #               1000000, 5000000, 10000000]

        self._conn = conn
        self._robot = robot
        self._reactor = reactor

        self._cTag = 'm1c1'

        self._tests = []

        for testType, color in self.TYPES:
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
        print('Setup environment...')

        # Add containers
        self._conn.createContainer(self._cTag)

        # Add the node
        self._conn.addNode(self._cTag, 'strEcho', 'Test', 'stringEcho.py')

        # Connections Robot - StringEcho (service)
        tag = 'test'
        cls = 'Test/StringEcho'
        self._conn.addInterface(self._cTag, tag, 'ServiceClientInterface',
                                cls, 'stringEchoService')
        self._conn.addInterface(self._robot, tag, 'ServiceProviderConverter',
                                cls)
        self._conn.addConnection('{0}/{1}'.format(self._cTag, tag),
                                 '{0}/{1}'.format(self._robot, tag))

        # Connections Robot - StringEcho (topic)
        tag = 'testReqTopic'
        cls = 'std_msgs/String'
        self._conn.addInterface(self._cTag, tag, 'PublisherInterface',
                                cls, 'stringEchoReq')
        self._conn.addInterface(self._robot, tag, 'SubscriberConverter',
                                cls)
        self._conn.addConnection('{0}/{1}'.format(self._cTag, tag),
                                 '{0}/{1}'.format(self._robot, tag))
        tag = 'testRespTopic'
        cls = 'std_msgs/String'
        self._conn.addInterface(self._cTag, tag, 'SubscriberInterface',
                                cls, 'stringEchoResp')
        self._conn.addInterface(self._robot, tag, 'PublisherConverter',
                                cls)
        self._conn.addConnection('{0}/{1}'.format(self._cTag, tag),
                                 '{0}/{1}'.format(self._robot, tag))

        self._reactor.callLater(20, self._run)

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
        print('\n\n------------------------------')
        print(self._sizes)
        print('\n'.join(str(test) for test in self._tests))
        print('------------------------------\n')

        self._conn.destroyContainer(self._cTag)

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
    user = 'testUser'
    robot = 'testRobot'
    connection = Connection(user, robot, user, reactor)
    benchmark = Benchmark(passes, connection, robot, reactor)

    print('Connect...')
    deferred = Deferred()
    deferred.addCallback(benchmark.run)
    connection.connect('http://{0}:9000/'.format(ip), deferred)

    reactor.run()


if __name__ == '__main__':
    from twisted.internet import reactor

    args = _get_argparse().parse_args()

    main(reactor, args.passes, args.ipMaster)
