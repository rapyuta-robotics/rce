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

        self.robots = {}
        self.containers = {}
        self.connections = {}

    @property
    def realm(self):
        """ Realm to which this User belongs. """
        return self._realm

    @property
    def userID(self):
        """ User ID of this User. """
        return self._userID

    def registerRobot(self, robot, robotID):
        """ Create a new Robot Wrapper.

            # TODO: Add description of arguments

            @raise:             rce.core.error.InvalidRequest
        """
        try:
            validateName(robotID)
        except IllegalName as e:
            raise InvalidRequest('Robot ID is invalid: {0}'.format(e))

        if (robotID in self.robots or robotID in self.containers):
            raise InvalidRequest('ID is already used for a container '
                                 'or robot.')

        robot = Robot(robot)
        self.robots[robotID] = robot
        robot.notifyOnDeath(self.robotDied)

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

    def containerDied(self, container):
        """ Callback which is used to inform the user of the death of a
            container.
        """
        if self.containers:
            for uid, candidate in self.containers.iteritems():
                if candidate == container:
                    del self.containers[uid]
                    break
        else:
            print('Received notification for dead Container, '
                  'but User is already destroyed.')

    def robotDied(self, robot):
        """ Callback which is used to inform the user of the death of a robot.
        """
        if self.robots:
            for uid, candidate in self.robots.iteritems():
                if candidate == robot:
                    del self.robots[uid]
                    break
        else:
            print('Received notification for dead Robot, '
                  'but User is already destroyed.')

    def connectionDied(self, connection):
        """ Callback which is used to inform the user of the death of a
            connection.
        """
        if self.connections:
            for uid, candidate in self.connections.iteritems():
                if candidate == connection:
                    del self.connections[uid]
                    break
        else:
            print('Received notification for dead Connection, '
                  'but User is already destroyed.')

    def destroy(self):
        """ Method should be called to destroy the user and will take care of
            destroying all objects owned by this User as well as deleting all
            circular references.
        """
        for connection in self.connections.itervalues():
            connection.dontNotifyOnDeath(self.connectionDied)

        for container in self.containers.itervalues():
            container.dontNotifyOnDeath(self.containerDied)

        for robot in self.robots.itervalues():
            robot.dontNotifyOnDeath(self.robotDied)

        for container in self.containers.itervalues():
            container.destroy()

        for robot in self.robots.itervalues():
            robot.destroy()

        self.connections = None
        self.containers = None
        self.robots = None
