#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-core/rce/core/user.py
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
from twisted.spread.pb import Avatar

# rce specific imports
from rce.util.name import validateName, IllegalName
from rce.core.error import InvalidRequest
from rce.core.base import MonitoredDict
from rce.core.view import MonitorView, AdminMonitorView, ControlView
from rce.core.wrapper import Robot


class User(Avatar):
    """ Represents a User avatar. It has references to all objects the User is
        currently using. For this to happen, each Robot object in the robot
        processes has a remote reference to on of the User instances in the
        Master. To modify the data the Robot objects should used Views.
    """
    def __init__(self, realm, userID):
        """ Initialize the User.

            @param realm:       The realm of the RoboEarth Cloud Engine master.
            @type  realm:       rce.core.RoboEarthCloudEngine

            @param userID:      The user ID associated with the user.
            @type  userID:      str
        """
        self._realm = realm
        self._userID = userID

        self._listeners = set()

        self.robots = MonitoredDict()
        self.containers = MonitoredDict()
        self.connections = MonitoredDict()

        self.robots.addListener = self._addRobot
        self.robots.removeListener = self._removeRobot

        self.containers.addListener = self._addContainer
        self.containers.removeListener = self._removeContainer

        self.connections.addListener = self._addConnection
        self.connections.removeListener = self._removeConnection

    @property
    def realm(self):
        """ Realm to which this User belongs. """
        return self._realm

    @property
    def userID(self):
        """ User ID of this User. """
        return self._userID

    def perspective_getUserView(self, console=True):
        """
        """
        if self._userID == 'admin' and not console:
            raise InvalidRequest('Administrator cannot login via robot')
        elif self._userID == 'admin':
            return AdminMonitorView()
        elif not console:
            return ControlView()
        else:
            return {'console' : MonitorView(), 'robot' : ControlView()}

    def registerRobot(self, robot, robotID):
        """ Create a new Robot Wrapper.

            # TODO: Add description of arguments

            @raise:             rce.core.error.InvalidRequest
        """
        try:
            validateName(robotID)
        except IllegalName as e:
            raise InvalidRequest('Robot ID is invalid: {0}'.format(e))

        if robotID in self.robots or robotID in self.containers:
            raise InvalidRequest('ID is already used for a container '
                                 'or robot.')

        # FIXME: Hack to get traffic info
        wrapped = Robot(robot)
        wrapped.changeListener = self._publishUpdate
        self.robots[robotID] = wrapped

    def getEndpoint(self, tag):
        """ Get an endpoint of the user matching the given tag.

            @param tag:         Tag which is used to identify the endpoint which
                                should be returned.
            @type  tag:         str

            @return:            Endpoint which was requested.
            @rtype:             rce.core.network.Endpoint

            @raise:             rce.core.error.InvalidRequest
        """
        if tag in self.robots:
            return self.robots[tag]
        elif tag in self.containers:
            return self.containers[tag]
        else:
            raise InvalidRequest('Can not get a non existent endpoint '
                                 "'{0}'.".format(tag))

    def registerListener(self, listener):
        """ TODO: Add doc
        """
        self._listeners.add(listener)
        self._updateListener(listener)

    def unregisterListener(self, listener):
        """ TODO: Add doc
        """
        self._listeners.discard(listener)

    def _updateListener(self, listener):
        listener.feedUpdate({'robot' : [(k, v.trafficInfo) for k, v in self.robots.iteritems()],
                             'container' : [(k, v.trafficInfo, v.nodes.keys()) for k, v in self.containers.iteritems()]})

    def _publishUpdate(self):
        for listener in self._listeners:
            self._updateListener(listener)

    def _addRobot(self, tag, robot):
        robot.notifyOnDeath(self._robotDied)
        self._publishUpdate()

    def _removeRobot(self, tag, robot):
        robot.dontNotifyOnDeath(self._robotDied)
        self._publishUpdate()

    def _robotDied(self, robot):
        if self.robots:
            for uid, candidate in self.robots.iteritems():
                if candidate == robot:
                    del self.robots[uid]
                    break
            else:
                print('Received notification for non existent Robot.')
        else:
            print('Received notification for dead Robot, '
                  'but User is already destroyed.')

    def _addContainer(self, tag, container):
        container.notifyOnDeath(self._containerDied)
        self._publishUpdate()

    def _removeContainer(self, tag, container):
        container.dontNotifyOnDeath(self._containerDied)
        self._publishUpdate()

    def _containerDied(self, container):
        if self.containers:
            for uid, candidate in self.containers.iteritems():
                if candidate == container:
                    del self.containers[uid]
                    break
            else:
                print('Received notification for non existent Container.')
        else:
            print('Received notification for dead Container, '
                  'but User is already destroyed.')

    def _addConnection(self, tag, connection):
        connection.notifyOnDeath(self._connectionDied)

    def _removeConnection(self, tag, connection):
        connection.dontNotifyOnDeath(self._connectionDied)

    def _connectionDied(self, connection):
        if self.connections:
            for uid, candidate in self.connections.iteritems():
                if candidate == connection:
                    del self.connections[uid]
                    break
            else:
                print('Received notification for non existent Connection.')
        else:
            print('Received notification for dead Connection, '
                  'but User is already destroyed.')

    def destroy(self):
        """ Method should be called to destroy the user and will take care of
            destroying all objects owned by this User as well as deleting all
            circular references.
        """
        for connection in self.connections.itervalues():
            connection.dontNotifyOnDeath(self._connectionDied)

        for container in self.containers.itervalues():
            container.dontNotifyOnDeath(self._containerDied)

        for robot in self.robots.itervalues():
            robot.dontNotifyOnDeath(self._robotDied)

        for container in self.containers.itervalues():
            container.destroy()

        for robot in self.robots.itervalues():
            robot.destroy()

        self.connections = None
        self.containers = None
        self.robots = None
