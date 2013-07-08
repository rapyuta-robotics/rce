#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     c2c.py
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
from base import PASSES, SIZES, delay, RemoteTest


class Measurement(object):
    TAG = 'tester'
    TYPES = ('service', 'topic')

    def __init__(self, runs, conn, robot, reactor):
        self._conn = conn
        self._robot = robot

        self._tests = [RemoteTest(conn, self.TAG, name) for name in self.TYPES]

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
        print('Setup environments...')

        ### Add containers
        self._cTag = ('c1', 'c2')
        for cTag in self._cTag:
            self._conn.createContainer(cTag)

        ### Add the nodes
        self._conn.addNode(self._cTag[0], 'strTester', 'Test',
                           'stringTester.py')
        self._conn.addNode(self._cTag[1], 'strEcho', 'Test', 'stringEcho.py')

        ### Add interfaces and connections
        # Connections Robot - StringTester
        tag = self.TAG
        cls = 'Test/StringTest'
        self._conn.addInterface(self._cTag[0], tag, 'ServiceClientInterface',
                                cls, 'stringTest')
        self._conn.addInterface(self._robot, tag, 'ServiceProviderConverter',
                                cls)
        self._conn.addConnection('{0}/{1}'.format(self._cTag[0], tag),
                                 '{0}/{1}'.format(self._robot, tag))

        # Connections StringTester - StringEcho (service)
        tag = 'testEchoSrv'
        cls = 'Test/StringEcho'
        srv = 'stringEchoService'
        self._conn.addInterface(self._cTag[0], tag, 'ServiceProviderInterface',
                                cls, srv)
        self._conn.addInterface(self._cTag[1], tag, 'ServiceClientInterface',
                                cls, srv)
        self._conn.addConnection('{0}/{1}'.format(self._cTag[0], tag),
                                 '{0}/{1}'.format(self._cTag[1], tag))

        # Connections StringTester - StringEcho (topic)
        tag = 'testEchoReq'
        cls = 'std_msgs/String'
        tpc = 'stringEchoReq'
        self._conn.addInterface(self._cTag[0], tag, 'SubscriberInterface',
                                cls, tpc)
        self._conn.addInterface(self._cTag[1], tag, 'PublisherInterface',
                                cls, tpc)
        self._conn.addConnection('{0}/{1}'.format(self._cTag[0], tag),
                                 '{0}/{1}'.format(self._cTag[1], tag))

        tag = 'testEchoResp'
        cls = 'std_msgs/String'
        tpc = 'stringEchoResp'
        self._conn.addInterface(self._cTag[0], tag, 'PublisherInterface',
                                cls, tpc)
        self._conn.addInterface(self._cTag[1], tag, 'SubscriberInterface',
                                cls, tpc)
        self._conn.addConnection('{0}/{1}'.format(self._cTag[0], tag),
                                 '{0}/{1}'.format(self._cTag[1], tag))

    def _postProcess(self, _):
        with open('c2c.data', 'w') as f:
            f.write(json.dumps(SIZES))
            f.write('\n')

            for test in self._tests:
                f.write(str(test))
                f.write('\n')

        for cTag in self._cTag:
            self._conn.destroyContainer(cTag)


def _get_argparse():
    from argparse import ArgumentParser

    parser = ArgumentParser(prog='c2c',
                            description='Run communication measurement for RCE '
                                        'between two containers using '
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
