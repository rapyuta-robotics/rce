#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     core.py
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
import sys
from uuid import uuid4

# zope specific imports
from zope.interface import implements

# twisted specific imports
from twisted.python import log
from twisted.cred.credentials import UsernamePassword
from twisted.cred.portal import IRealm, Portal
from twisted.spread.pb import IPerspective, PBServerFactory, Avatar
from twisted.web.server import Site
from twisted.internet import defer

# Custom imports
from rce.error import InternalError
from rce.client.protocol import MasterRobotAuthentication
from rce.master.machine import LoadBalancer, ContainerProcessError, \
    Distributor, RobotProcessError
from rce.master.network import Network
from rce.master.environment import EnvironmentEndpoint
from rce.master.robot import RobotEndpoint
from rce.master.user import User
from rce.util.network import getIP
from rce.master.console import Console, UserAvatar, ConsoleDummyRealm

class RoboEarthCloudEngine(object):
    """ Realm for the twisted cred system. It is responsible for storing all
        cloud engine relevant informations by having a reference to a Network,
        Load Balancer, and Distributor instance, where each is responsible for
        part of the system. Additionally, the realm keeps a list of all User
        object.
        
        There should be only one instance running in the Master process.
    """
    implements(IRealm)
    
    # CONFIG
    LOAD_BALANCER_CLS = LoadBalancer
    DISTRIBUTOR_CLS = Distributor
    CONSOLE_CLS = Console
    
    def __init__(self, reactor, checker, intIF, port):
        """ Initialize the RoboEarth Cloud Engine realm.
            
            @param reactor:     Twisted reactor used in this process.
            @type  reactor:     twisted::reactor
            
            @param checker:     Login checker which authenticates the User when
                                an initial request is received.
            @type  checker:     twisted.cred.checkers.ICredentialsChecker
            
            @param intIF:       Name of network interface used for internal
                                communication.
            @type  intIF:       str
            
            @param port:        Port where the robot process is listening for
                                connections to other endpoints.
            @type  port:        int
        """
        self._reactor = reactor
        self._checker = checker
        self._intIF = intIF
        self._port = port
        
        self._network = Network()
        self._balancer = self.LOAD_BALANCER_CLS(self)
        self._distributor = self.DISTRIBUTOR_CLS()
        self._console = self.CONSOLE_CLS(self)
        
        self._users = {}
        self._pendingContainer = {}
        
        self._intIP = None
    
    def requestAvatar(self, avatarId, mind, *interfaces):
        """ Returns Avatar for slave processes of the cloud engine.
            
            Implementation for IRealm
        """
        if IPerspective not in interfaces:
            raise NotImplementedError('RoboEarthCloudEngine only '
                                      'handles IPerspective.')
        
        # There are three possible roles (=avatarId):
        #     'container', 'robot', and 'environment'
        if avatarId == 'container':
            machine = self._balancer.createMachine(mind[0], mind[1])
            avatar = Avatar() # TODO: At the moment does nothing
            detach = lambda: self._balancer.destroyMachine(machine)
            print('Connection to Container process established.')
        elif avatarId == 'robot':
            endpoint = RobotEndpoint(self._network, self._distributor, self,
                                     self._port)
            endpoint.callback(mind)
            avatar = Avatar() # TODO: At the moment does nothing
            detach = lambda: endpoint.destroy()
            print('Connection to Robot process established.')
        elif avatarId == 'environment':
            endpoint = self._pendingContainer.pop(mind[1])
            endpoint.callback(mind[0])
            avatar = Avatar() # TODO: At the moment does nothing
            detach = lambda: endpoint.destroy()
            print('Connection to Environment process established.')
        else:
            raise InternalError('Invalid avatar ID received.')
        
        return IPerspective, avatar, detach
    
    def _getUser(self, userID):
        if userID not in self._users:
            self._users[userID] = User(self, userID)
        
        return self._users[userID]
    
    def getInternalIP(self):
        """ Get the IP address of the network interface used for the internal
            communication.
            
            @return:            IP address of the network interface.
            @rtype:             str
        """
        if self._intIP is None:
            self._intIP = getIP(self._intIF)
        
        return self._intIP
    
    def requestUser(self, userID, robotID, password):
        """ Callback for client protocol to initialize the connection by
            authenticating himself and announcing the robot's ID.
            
            @param userID:      User ID under which the robot will login.
            @type  userID:      str
            
            @param robotID:     ID of the robot which will be using the cloud
                                engine.
            @type  robotID:     str
            
            @param password:    Password matching the User ID for login.
            @type  password:    str
            
            @return:            Key which should be used to authenticate the
                                websocket connection and the IP address of
                                responsible robot process as a tuple.
            @rtype:             twisted::Deferred
        """
        # Check if the credentials are correct
        d = self._checker.requestAvatarId(UsernamePassword(userID, password))
        
        # Create the robot
        d.addCallback(lambda uID: self._getUser(uID).createRobot(robotID))
        
        return d
    
    def createRobot(self, user, robotID, uid):
        """ Callback for User instance to create a new Robot object in a
            robot process.
            
            @param user:        User instance under which the robot has logged
                                in and will run. (Owner of the robot)
            @type  user:        rce.master.user.User
                                (subclass of twisted.spread.pb.Referenceable)
            
            @param robotID:     ID of the robot which has to be created.
            @type  robotID:     str
            
            @param uid:         Key which will be used to authenticate the
                                websocket connection.
            @type  uid:         str
            
            @return:            New Robot instance.
            @rtype:             rce.master.robot.Robot
                                (subclass of rce.master.base.Proxy)
        """
        try:
            location = self._distributor.getNextLocation()
        except RobotProcessError:
            # TODO: What should we do here?
            raise InternalError('Robot can not be created.')
        
        return location.createNamespace(user, robotID, uid)
    
    def createContainer(self, userID):
        """ Callback for User instance to create a new Container object in a
            container process.
            
            @param userID:        UserID of the user who created the container.
            @type  userID:        str

            @return:            New Namespace and Container instance.
            @rtype:             (rce.master.environment.Environment,
                                 rce.master.container.Container)
                                (subclasses of rce.master.base.Proxy)
        """
        while 1:
            uid = uuid4().hex
            
            if uid not in self._pendingContainer:
                break
        
        try:
            container = self._balancer.createContainer(uid, userID)
        except ContainerProcessError:
            # TODO: What should we do here?
            raise InternalError('Container can not be created.')
        
        endpoint = EnvironmentEndpoint(self._network, container)
        self._pendingContainer[uid] = endpoint
        return endpoint.createNamespace(), container
    
    def createConnection(self, interfaceA, interfaceB):
        """ Callback for User instance to create a new connection between two
            interfaces.
            
            @param interfaceX:  Interface which should be connected.
            @type  interfaceX:  rce.master.network.Interface
            
            @return:            New Connection instance.
            @rtype:             rce.master.network.Connection
        """
        return self._network.createConnection(interfaceA, interfaceB)
    
    def preShutdown(self):
        """ Method is executed by the twisted reactor when a shutdown event
            is triggered, before the reactor is being stopped.
        """
        for user in self._users.values():
            user.destroy()
    
    def postShutdown(self):
        """ Method is executed by the twisted reactor when a shutdown event
            is triggered, after the reactor has been stopped.
        """
        self._network.cleanUp()
        self._balancer.cleanUp()
        self._distributor.cleanUp()


def main(reactor, internalCred, externalCred, internalPort, externalPort,
         intIF, commPort, consolePort):
    log.startLogging(sys.stdout)
    
    rce = RoboEarthCloudEngine(reactor, externalCred, intIF, commPort)
    consolerealm = ConsoleDummyRealm(reactor, rce)

    # Internal communication
    p = Portal(rce, (internalCred,))
    reactor.listenTCP(internalPort, PBServerFactory(p))
    
    # Client Connection
    reactor.listenTCP(externalPort, Site(MasterRobotAuthentication(rce)))
    
    pconsole = Portal(consolerealm, (externalCred,))
    
    #Console Connection
    reactor.listenTCP(consolePort, PBServerFactory(pconsole))
    reactor.addSystemEventTrigger('before', 'shutdown', rce.preShutdown)
    reactor.addSystemEventTrigger('after', 'shutdown', rce.postShutdown)
    reactor.run()
