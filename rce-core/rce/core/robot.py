#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-core/rce/core/robot.py
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

# twisted specific imports
from twisted.internet.address import IPv4Address

# rce specific imports
from rce.util.settings import getSettings
from rce.util.network import isLocalhost
from rce.core.error import InvalidRequest
from rce.core.network import Endpoint, Namespace, EndpointAvatar


class Robot(Namespace):
    """ Representation of a namespace which has a WebSocket connection from a
        robot assigned and is part of the cloud engine internal communication.
    """
    def __init__(self, endpoint):
        """ Initialize the Robot.

            @param endpoint:    Endpoint in which the robot was created.
            @type  endpoint:    rce.core.network.Endpoint
        """
        super(Robot, self).__init__(endpoint)

    def getWebsocketAddress(self):
        """ Get the address which can be used to connect to this robot
            namespace.

            @return:            Address of the endpoint containing the robot
                                namespace. The address has the form
                                    [IP]:[port] (type: str)
            @rtype:             twisted.internet.defer.Deferred
        """
        return self._endpoint.getWebsocketAddress()


class RobotEndpoint(Endpoint):
    """ Representation of an endpoint which is a process that acts as a server
        for WebSocket connections from robots and is part of the cloud engine
        internal communication.
    """
    def __init__(self, network, distributor, port):
        """ Initialize the Environment Endpoint.

            @param network:     Network to which the endpoint belongs.
            @type  network:     rce.master.network.Network

            @param distributor: Distributor which is responsible for assigning
                                new robot WebSocket connections to robot
                                endpoints.
            @type  distributor: rce.core.robot.Distributor

            @param port:        Port where the robot process is listening for
                                connections to other endpoints.
            @type  port:        int
        """
        super(RobotEndpoint, self).__init__(network)

        self._distributor = distributor
        distributor.registerRobotProcess(self)

        self._port = port

    @property
    def active(self):
        """ The number of active robot websocket connections in the
            robot process.
        """
        return len(self._namespaces)

    def getAddress(self):
        """ Get the address of the robot endpoint's internal communication
            server.

            @return:            Address of the robot endpoint's internal
                                communication server.
                                (type: twisted.internet.address.IPv4Address)
            @rtype:             twisted.internet.defer.Deferred
        """
        def cb(remote):
            ip = remote.broker.transport.getPeer().host
            ip = getSettings().internal_IP if isLocalhost(ip) else ip
            return IPv4Address('TCP', ip, self._port)

        return self().addCallback(cb)

    def getWebsocketAddress(self):
        """ Get the address which can be used to connect to the robot
            namespaces which belong to this endpoint.

            @return:            Address of the endpoint process. The address
                                has the form [IP]:[port] (type: str)
            @rtype:             twisted.internet.defer.Deferred
        """
        return self.callRemote('getWebsocketAddress')

    def registerRemoteRobot(self, remoteRobot):
        """ Register a Namespace object of the endpoint.

            @param remoteRobot: Reference to Robot namespace in Robot process.
            @type  remoteRobot: twisted.spread.pb.RemoteReference

            @return:            New Robot instance.
            @rtype:             rce.core.robot.Robot
                                (subclass of rce.core.base.Proxy)
        """
        robot = Robot(self)
        robot.callback(remoteRobot)
        return robot

    def destroy(self):
        """ Method should be called to destroy the robot endpoint and will take
            care of destroying all objects owned by this RobotEndpoint as well
            as deleting all circular references.
        """
        if self._distributor:
            print('Destroying Connection to Robot Process.')
            self._distributor.unregisterRobotProcess(self)
            self._distributor = None
            super(RobotEndpoint, self).destroy()
        else:
            print('robot.RobotEndpoint destroy() called multiple times...')


class RobotEndpointAvatar(EndpointAvatar):
    """ Avatar for internal PB connection form a Robot Endpoint.
    """
    def perspective_setupNamespace(self, remoteRobot, userID, robotID):
        """ Register a Robot namespace with the Master process.

            @param remoteRobot: Reference to the Robot namespace in the Robot
                                process.
            @type  remoteRobot: twisted.spread.pb.RemoteReference

            @param userID:      User ID of the robot owner.
            @type  userID:      str

            @param robotID:     Unique ID which is used to identify the robot.
            @type  robotID:     str
        """
        user = self._realm.getUser(userID)
        robot = self._endpoint.registerRemoteRobot(remoteRobot)

        try:
            user.registerRobot(robot, robotID)
        except InvalidRequest:
            robot.destroy()
            raise
