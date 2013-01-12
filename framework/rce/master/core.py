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
from twisted.cred.checkers import InMemoryUsernamePasswordDatabaseDontUse
#from twisted.cred.checkers import FilePasswordDB
from twisted.cred.portal import IRealm, Portal
from twisted.spread.pb import IPerspective, PBServerFactory, Avatar
from twisted.web.server import Site

# Custom imports
from rce.error import InternalError
from rce.client.protocol import MasterRobotAuthentication
from rce.master.machine import LoadBalancer
from rce.master.network import Network
from rce.master.environment import EnvironmentEndpoint
from rce.master.robot import Distributor, RobotEndpoint
from rce.master.user import User

# TODO: At the moment only one user available and Password database in memory
class RoboEarthCloudEngine(object):
    """
    """
    implements(IRealm)
    
    MAX_ROBOTS = 10
    MAX_CONTAINER = 10
    LOAD_BALANCER_CLS = LoadBalancer
    DISTRIBUTOR_CLS = Distributor
    
    def __init__(self, reactor):
        """
        """
        self._reactor = reactor
        
        self._checker = InMemoryUsernamePasswordDatabaseDontUse(testUser='testUser')
        #self._checker = FilePasswordDB(userFile)
        self._users = {}
        
        self._network = Network()
        self._balancer = self.LOAD_BALANCER_CLS()
        self._distributor = self.DISTRIBUTOR_CLS()
        
        self._pendingContainer = {}
    
    def _detachRobotEndpoint(self, endpoint):
        """
        """
        self._distributor.unregisterRelay(endpoint)
        self._network.destroyEndpoint(endpoint)
    
    def requestAvatar(self, avatarId, mind, *interfaces):
        """
        """
        if IPerspective not in interfaces:
            raise NotImplementedError('RoboEarthCloudEngine only '
                                      'handles IPerspective.')
        
        # There are three possible roles (=avatarId):
        #     'container', 'robot', and 'environment'
        if avatarId == 'container':
            machine = self._balancer.createMachine(mind, self.MAX_CONTAINER)
            avatar = Avatar() # TODO: At the moment does nothing
            detach = lambda: self._balancer.destroyMachine(machine)
            print('Connection to Container process established.')
        elif avatarId == 'robot':
            endpoint = RobotEndpoint(self._network, self._distributor)
            endpoint.createRemoteObject(mind)
            avatar = Avatar() # TODO: At the moment does nothing
            detach = lambda: endpoint.destroy()
            print('Connection to Robot process established.')
        elif avatarId == 'environment':
            endpoint = self._pendingContainer.pop(mind[1])
            endpoint.createRemoteObject(mind[0])
            avatar = Avatar() # TODO: At the moment does nothing
            detach = lambda: endpoint.destroy()
            print('Connection to Environment process established.')
        else:
            raise InternalError('Invalid avatar ID received.')
        
        return IPerspective, avatar, detach
    
    def _getUser(self, userID):
        """
        """
        if userID not in self._users:
            self._users[userID] = User(self, userID)
        
        return self._users[userID]
    
    def requestUser(self, userID, robotID, password):
        """
        """
        d = self._checker.requestAvatarId(UsernamePassword(userID, password))
        d.addCallback(self._userAuthenticated, robotID)
        return d
    
    def _userAuthenticated(self, userID, robotID):
        """
        """
        return self._getUser(userID).createRobot(robotID)
    
    def createRobot(self, user, userID, robotID, uid):
        """
        """
        return self._distributor.getNextLocation(
                    ).createNamespace(user, userID, robotID, uid)
    
    def createContainer(self, tag):
        """
        """
        while 1:
            uid = uuid4().hex
            
            if uid not in self._pendingContainer:
                break
        
        container = self._balancer.createContainer(uid)
        endpoint = EnvironmentEndpoint(self._network, container)
        self._pendingContainer[uid] = endpoint
        return endpoint.createNamespace()
    
    def createConnection(self, interfaceA, interfaceB):
        """
        """
        return self._network.createConnection(interfaceA, interfaceB)
        
    
    def preShutdown(self):
        """
        """
        for user in self._users.values():
            user.destroy()
    
    def postShutdown(self):
        """
        """
        self._network.cleanUp()
        self._balancer.cleanUp()
        self._distributor.cleanUp()


def main(reactor, cred, internalPort, externalPort):
    log.startLogging(sys.stdout)
    
    rce = RoboEarthCloudEngine(reactor)
    
    p = Portal(rce, (cred,))
    #p = Portal(RoboEarthCloudEngine(), (FilePasswordDB(credFile),))
    reactor.listenTCP(internalPort, PBServerFactory(p))
    
    # Client Connection
    reactor.listenTCP(externalPort, Site(MasterRobotAuthentication(rce)))
    
    reactor.addSystemEventTrigger('before', 'shutdown', rce.preShutdown)
    reactor.addSystemEventTrigger('after', 'shutdown', rce.postShutdown)
    reactor.run()
