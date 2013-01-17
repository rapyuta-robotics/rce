#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     robot.py
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

# Custom imports
from rce.master.network import Endpoint, Namespace
from rce.master.base import Status


class Robot(Namespace):
    """ Representation of a namespace which has a websocket connection from a
        robot assigned and is part of the cloud engine internal communication.
    """
    def __init__(self, endpoint):
        """ Initialize the Robot.
            
            @param endpoint:    Endpoint in which the robot was created.
            @type  endpoint:    rce.master.network.Endpoint
        """
        super(Robot, self).__init__(endpoint)
    
    def getIP(self):
        """ Get the IP address of the process in which the Robot lives.
            
            @return:            IP address of the process. (type: str)
            @rtype:             twisted::Deferred
        """
        return self().addCallback(lambda remote:
                                  remote.broker.transport.getPeer().host)


class RobotEndpoint(Endpoint):
    """ Representation of an endpoint which is a process that acts as a server
        for websocket connections from robots and is part of the cloud engine
        internal communication.
    """
    def __init__(self, network, distributor):
        """ Initialize the Environment Endpoint.
            
            @param network:     Network to which the endpoint belongs.
            @type  network:     rce.master.network.Network
            
            @param distributor: Distributor which is responsible for assigning
                                new robot websocket connections to robot
                                endpoints.
            @type  container:   rce.master.robot.Distributor
        """
        super(RobotEndpoint, self).__init__(network)
        
        self._distributor = distributor
        distributor.registerRobotProcess(self)
    
    @property
    def active(self):
        """ The number of active robot websocket connections in the
            robot process.
        """
        return len(self._namespaces)
    
#    def getAddress(self):
#        """ Get the address of the robot endpoint's internal communication
#            server.
#            
#            @return:            Address of the robot endpoint's internal
#                                communication server.
#                                (type: twisted.internet.address.IPv4Address)
#            @rtype:             twisted::Deferred
#        """
#        return succeed(self.obj.broker.transport.getPeer())
    
    def createNamespace(self, user, robotID, key):
        """ Create a Namespace object in the endpoint.
            
            @param user:        User instance to which this namespace will
                                belong.
            @type  user:        rce.master.user.User
            
            @param robotID:     ID of the robot which has to be created.
            @type  robotID:     str
            
            @param uid:         Key which will be used to authenticate the
                                webscoket connection.
            @type  uid:         str
            
            @return:            New Robot instance.
            @rtype:             rce.master.robot.Robot
                                (subclass of rce.master.base.Proxy)
        """
        robot = Robot(self)
        status = Status(robot)
        self.callRemote('createNamespace', status, user, user.userID, robotID,
                        key).chainDeferred(robot)
        return robot
    
    def destroy(self):
        """ Method should be called to destroy the robot endpoint and will take
            care of destroying all objects owned by this RobotEndpoint as well
            as deleting all circular references.
        """
        if self._distributor:
            self._distributor.unregisterRobotProcess(self)
            self._distributor = None
            
            super(RobotEndpoint, self).destroy()
        else:
            print('robot.RobotEndpoint destroy() called multiple times...')
