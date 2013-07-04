#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     r2c.py
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
#     Copyright 2013 RoboEarth
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
import json

# twisted specific imports
from twisted.internet.defer import Deferred

# rce specific imports
from rce.client.connection import Connection

# local imports
from base import PASSES, SIZES, delay, LocalTopicTest, LocalServiceTest


class Measurement(object):
    TYPES = ((LocalServiceTest, 'test'),
             (LocalTopicTest, ('testReqTopic', 'testRespTopic')))

    def __init__(self, runs, conn, robot, reactor):
        self._conn = conn
        self._robot = robot

        self._tests = [cls(conn, iTag) for cls, iTag in self.TYPES]

        self._deferred = d = Deferred()

        d.addCallback(self._setup)
        d.addCallback(delay, 20, reactor)

        for _ in xrange(runs):
            for test in self._tests:
                d.addCallback(test.run)

        d.addCallback(self._postProcess)
        d.addCallback(delay, 2, reactor)

        def stop(_):
            reactor.stop()
            print('\ndone')

        d.addCallback(stop)

    def run(self, _):
        if not self._tests:
            print('No tests to run.')
            return

        if self._deferred.called:
            print('Can run the measurement only once.')

        self._deferred.callback(None)

    def _setup(self, _):
        print('Setup environment...')

        # Add containers
        self._cTag = 'c1'
        self._conn.createContainer(self._cTag)

        # Add the node
        self._conn.addNode(self._cTag, 'strEcho', 'Test', 'stringEcho.py')

        # Connections Robot - StringEcho (service)
        tag = self.TYPES[0][1]
        cls = 'Test/StringEcho'
        self._conn.addInterface(self._cTag, tag, 'ServiceClientInterface',
                                cls, 'stringEchoService')
        self._conn.addInterface(self._robot, tag, 'ServiceProviderConverter',
                                cls)
        self._conn.addConnection('{0}/{1}'.format(self._cTag, tag),
                                 '{0}/{1}'.format(self._robot, tag))

        # Connections Robot - StringEcho (topic)
        tag = self.TYPES[1][1][0]
        cls = 'std_msgs/String'
        self._conn.addInterface(self._cTag, tag, 'PublisherInterface',
                                cls, 'stringEchoReq')
        self._conn.addInterface(self._robot, tag, 'SubscriberConverter',
                                cls)
        self._conn.addConnection('{0}/{1}'.format(self._cTag, tag),
                                 '{0}/{1}'.format(self._robot, tag))

        tag = self.TYPES[1][1][1]
        cls = 'std_msgs/String'
        self._conn.addInterface(self._cTag, tag, 'SubscriberInterface',
                                cls, 'stringEchoResp')
        self._conn.addInterface(self._robot, tag, 'PublisherConverter',
                                cls)
        self._conn.addConnection('{0}/{1}'.format(self._cTag, tag),
                                 '{0}/{1}'.format(self._robot, tag))

    def _postProcess(self, _):
        with open('r2c.data', 'w') as f:
            f.write(json.dumps(SIZES))
            f.write('\n')

            for test in self._tests:
                f.write(str(test))
                f.write('\n')

        self._conn.destroyContainer(self._cTag)


def _get_argparse():
    from argparse import ArgumentParser

    parser = ArgumentParser(prog='r2c',
                            description='Run communication measurement for RCE '
                                        'between a robot and a container using '
                                        'a string message.')

    parser.add_argument('--passes', help='Number of passes to do.',
                        type=int, default=PASSES)
    parser.add_argument('ipMaster', help='IP address of master process.',
                        type=str)

    return parser


def main(reactor, passes, ip):
    user = 'testUser'
    robot = 'testRobot'
    connection = Connection(user, robot, user, reactor)
    measurement = Measurement(passes, connection, robot, reactor)

    print('Connect...')
    d = Deferred()
    d.addCallback(measurement.run)
    connection.connect('http://{0}:9000/'.format(ip), d)

    reactor.run()


if __name__ == '__main__':
    from twisted.internet import reactor

    args = _get_argparse().parse_args()

    main(reactor, args.passes, args.ipMaster)
