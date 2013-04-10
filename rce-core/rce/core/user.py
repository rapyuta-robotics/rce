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
from twisted.spread.pb import Avatar, Error

# rce specific imports
from rce.util.name import validateName
from rce.core.view import AdminConsoleView, NormalConsoleView, RobotView
from rce.core.wrapper import Robot


class InvalidRequest(Error):
    """ Exception is raised if the request can not be processed.
    """


class User(Avatar):
    """ Represents a User avatar. It has references to all objects the User is
        currently using. For this to happen, each Robot object in the robot
        processes has a remote reference to on of the User instances in the
        Master. To modify the data the Robot objects should used Views.
    """
    def __init__(self, realm, userID):
        """ Initialize the User.
            
            @param realm:       The realm of the RoboEarth Cloud Engine master.
            @type  realm:       rce.master.core.RoboEarthCloudEngine
            
            @param userID:      The user ID associated with the user.
            @type  userID:      str
        """
        self._realm = realm
        self._userID = userID
        
        self._role = None
        self._robots = {}
        self._containers = {}
        self._connections = {}
    
    @property
    def userID(self):
        """ User ID of this User. """
        return self._userID
    
    @property
    def role(self):
        """ Role of the user. """
        return self._role

    @property
    def robots(self):
        """ Robots owned by this User. """
        return self._robots

    @property
    def containers(self):
        """" Containers used by this User. """
        return self._containers

    def createRobotWrapper(self, robotNamespace, location, robotID):
        """ Create a new Robot Wrapper.
            
            # TODO: Add description of arguments
            
            @raise:             rce.util.name.IllegalName,
                                rce.core.user.InvalidRequest
        """
        validateName(robotID)
        
        if (robotID in self._robots or robotID in self._containers):
            raise InvalidRequest('ID is already used for a container '
                                 'or robot.')
        
        robot, status = location.createRobotProxy(robotID, robotNamespace)
        robot = Robot(robot)
        self._robots[robotID] = robot
        robot.notifyOnDeath(self._robotDied)
        return status
    
    def perspective_getUserView(self, console=True):
        """
        """
        if self._userID == 'admin' and not console:
            raise InvalidRequest('Administrator cannot login via robot')
        elif self._userID == 'admin':
            return AdminConsoleView()
        elif not console:
            return RobotView()
        else:
            return {'console': NormalConsoleView(), 'robot': RobotView()}

    def _getEndpoint(self, tag):
        if tag in self._robots:
            return self._robots[tag]
        elif tag in self._containers:
            return self._containers[tag]
        else:
            raise InvalidRequest('Can not get a non existent endpoint '
                                 "'{0}'.".format(tag))
    
    def _containerDied(self, container):
        if self._containers:
            for key, value in self._containers.iteritems():
                if value == container:
                    del self._containers[key]
                    break
        else:
            print('Received notification for dead Container, '
                  'but User is already destroyed.')
    
    def _robotDied(self, robot):
        if self._robots:
            for key, value in self._robots.iteritems():
                if value == robot:
                    del self._robots[key]
                    break
        else:
            print('Received notification for dead Robot, '
                  'but User is already destroyed.')
    
    def _connectionDied(self, connection):
        if self._connections:
            for key, value in self._connections.iteritems():
                if value == connection:
                    del self._connections[key]
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
