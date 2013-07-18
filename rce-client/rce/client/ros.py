#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-client/rce/client/ros.py
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

# ROS specific imports
try:
    import rospy
except ImportError:
    print('Can not import ROS Python client.')
    exit(1)

# twisted specific imports
from twisted.internet.defer import Deferred

# rce specific imports
from rce.util.ros import decorator_has_connection
from rce.client.connection import ConnectionError, ROSConnection


# Patch the method 'rospy.topics._TopicImpl.has_connection'
rospy.topics._TopicImpl.has_connection = \
    decorator_has_connection(rospy.topics._TopicImpl.has_connection)


_MAP = {'ServiceClientConverter'   : 'serviceProvider',
        'ServiceProviderConverter' : 'serviceClient',
        'PublisherConverter'       : 'subscriber',
        'SubscriberConverter'      : 'publisher'}


class Environment(object):
    def __init__(self, reactor, conn, config):
        self._reactor = reactor
        self._conn = conn

        self._containers = config.get('containers', [])
        self._nodes = config.get('nodes', [])
        self._parameters = config.get('parameters', [])
        self._interfaces = config.get('interfaces', [])
        self._connections = config.get('connections', [])

        self._ifs = []

    def run(self, _):
        try:
            for container in self._containers:
                self._conn.createContainer(**container)

            for parameter in self._parameters:
                self._conn.addParameter(**parameter)

            for node in self._nodes:
                self._conn.addNode(**node)

            for interface in self._interfaces:
                self._conn.addInterface(**interface)

            for connection in self._connections:
                self._conn.addConnection(**connection)

            for ros in self._interfaces:
                iType = ros['iType']

                if iType in _MAP:
                    self._ifs.append(
                        getattr(self._conn, _MAP[iType])(ros['iTag'],
                                                         ros['iCls'],
                                                         ros['addr'])
                    )
        except Exception as e:
            import traceback
            print(''.join(traceback.format_exception_only(type(e), e)))
            rospy.signal_shutdown('Error')

    def terminate(self):
        try:
            for parameter in self._parameters:
                self._conn.removeParameter(parameter['cTag'],
                                           parameter['name'])

            for container in self._containers:
                self._conn.destroyContainer(container.get('cTag'))
        except ConnectionError:
            pass

        self._ifs = []

        self._reactor.callLater(1, self._reactor.callFromThread,
                                self._reactor.stop)


def main(config, reactor):
    rospy.init_node('RCE_ROS_Client', anonymous=True)

    try:
        userID = config['userID']
        robotID = config['robotID']
        password = config['password']
        url = config['url']
    except KeyError as e:
        print('Configuration is missing the key {0}.'.format(e))
        return 1

    conn = ROSConnection(userID, robotID, password, reactor)
    env = Environment(reactor, conn, config)

    deferred = Deferred()
    deferred.addCallback(env.run)

    conn.connect(url, deferred)

    rospy.on_shutdown(env.terminate)

    reactor.run(installSignalHandlers=False)
