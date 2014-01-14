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

        self._robots = {}
        self._containers = {}
        self._connections = {}

        # TODO: Temporary for WebUI!
        self._listeners = set()

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
            return {'console': MonitorView(), 'robot': ControlView()}

    def getRobotList(self):
        """ # TODO: Add doc
        """
        return self._robots.keys()

    def getContainerList(self):
        """ # TODO: Add doc
        """
        return self._containers.keys()

    def registerRobot(self, robot, robotID):
        """ Create a new Robot Wrapper.

            # TODO: Add description of arguments

            @raise:             rce.core.error.InvalidRequest
        """
        try:
            validateName(robotID)
        except IllegalName as e:
            raise InvalidRequest('Robot ID is invalid: {0}'.format(e))

        if self.hasEndpoint(robotID):
            raise InvalidRequest('ID is already used for a container '
                                 'or robot.')

        robot = Robot(robot)
        self._robots[robotID] = robot
        robot.notifyOnDeath(self._robotDied)

        self._updateFeed()

    def registerContainer(self, container, tag):
        """ # TODO: Add doc
        """
        self._containers[tag] = container
        container.notifyOnDeath(self._containerDied)

        self._updateFeed()

    def registerConnection(self, connection, key):
        """ # TODO: Add doc
        """
        self._connections[key] = connection
        connection.notifyOnDeath(self._connectionDied)

    def hasEndpoint(self, tag):
        """ # TODO: Add doc
        """
        return tag in self._containers or tag in self._robots

    def getEndpoint(self, tag):
        """ Get an endpoint of the user matching the given tag.

            @param tag:         Tag which is used to identify the endpoint which
                                should be returned.
            @type  tag:         str

            @return:            Endpoint which was requested.
            @rtype:             rce.core.network.Endpoint

            @raise:             KeyError
        """
        if tag in self._robots:
            return self._robots[tag]
        elif tag in self._containers:
            return self._containers[tag]
        else:
            raise KeyError(tag)

    def getRobot(self, robotID):
        """ # TODO: Add doc

            @raise:             KeyError
        """
        return self._robots[robotID]

    def getContainer(self, tag):
        """ # TODO: Add doc

            @raise:             KeyError
        """
        return self._containers[tag]

    def popContainer(self, tag):
        """ # TODO: Add doc

            @raise:             KeyError
        """
        container = self._containers.pop(tag)
        container.dontNotifyOnDeath(self._containerDied)
        return container

    def hasConnection(self, key):
        """ # TODO: Add doc
        """
        return key in self._connections

    def popConnection(self, key):
        """ # TODO: Add doc

            @raise:             KeyError
        """
        connection = self._connections.pop(key)
        connection.dontNotifyOnDeath(self._connectionDied)
        return connection

    def registerListener(self, listener):
        """ TODO: Temporary for WebUI!
        """
        self._listeners.add(listener)
        listener.feedUpdate({'robot' : self._robots.keys(),
                             'container' : self._containers.keys()})

    def unregisterListener(self, listener):
        """ TODO: Temporary for WebUI!
        """
        self._listeners.discard(listener)

    def _updateFeed(self):
        """ TODO: Temporary for WebUI!
        """
        for listener in self._listeners:
            listener.feedUpdate({'robot' : self._robots.keys(),
                                 'container' : self._containers.keys()})

    def _containerDied(self, container):
        """ Callback which is used to inform the user of the death of a
            container.
        """
        if self._containers:
            for uid, candidate in self._containers.iteritems():
                if candidate == container:
                    del self._containers[uid]
                    self._updateFeed()
                    break
        else:
            print('Received notification for dead Container, '
                  'but User is already destroyed.')

    def _robotDied(self, robot):
        """ Callback which is used to inform the user of the death of a robot.
        """
        if self._robots:
            for uid, candidate in self._robots.iteritems():
                if candidate == robot:
                    del self._robots[uid]
                    self._updateFeed()
                    break
        else:
            print('Received notification for dead Robot, '
                  'but User is already destroyed.')

    def _connectionDied(self, connection):
        """ Callback which is used to inform the user of the death of a
            connection.
        """
        if self._connections:
            for uid, candidate in self._connections.iteritems():
                if candidate == connection:
                    del self._connections[uid]
                    break
        else:
            print('Received notification for dead Connection, '
                  'but User is already destroyed.')

    def destroy(self):
        """ Method should be called to destroy the user and will take care of
            destroying all objects owned by this User as well as deleting all
            circular references.
        """
        for connection in self._connections.itervalues():
            connection.dontNotifyOnDeath(self._connectionDied)

        for container in self._containers.itervalues():
            container.dontNotifyOnDeath(self._containerDied)

        for robot in self._robots.itervalues():
            robot.dontNotifyOnDeath(self._robotDied)

        for container in self._containers.itervalues():
            container.destroy()

        for robot in self._robots.itervalues():
            robot.destroy()

        self._connections = None
        self._containers = None
        self._robots = None
