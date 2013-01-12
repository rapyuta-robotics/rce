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

# Python specific imports
import sys
from uuid import UUID

# zope specific imports
from zope.interface import implements

# twisted specific imports
from twisted.python import log
from twisted.python.failure import Failure
from twisted.internet.defer import fail, maybeDeferred
from twisted.cred.error import UnauthorizedLogin
from twisted.cred.checkers import ICredentialsChecker
from twisted.cred.portal import IRealm, Portal
from twisted.spread.pb import PBClientFactory, DeadReferenceError

# Autobahn specific imports
from autobahn.websocket import listenWS

# Custom imports
from rce.error import InternalError, DeadConnection
from rce.client.interfaces import IRobot, IRobotCredentials
from rce.client.protocol import RobotWebSocketProtocol, \
    CloudEngineWebSocketFactory
from rce.monitor.converter import PublisherConverter, SubscriberConverter, \
    ServiceClientConverter, ServiceProviderConverter, Forwarder
from rce.slave.endpoint import Endpoint
from rce.slave.namespace import Namespace


class Robot(Namespace):
    """
    """
    implements(IRobot)
    
    _MAP = [ServiceClientConverter, PublisherConverter,
            SubscriberConverter, ServiceProviderConverter,
            Forwarder, Forwarder, Forwarder, Forwarder]
    
    def __init__(self, user):
        """
        """
        self._user = user
        self._connection = None
        
        self._interfaces = {}
    
    def _reportError(self, failure):
        self._connection.sendErrorMessage(failure.getTraceback())
    
    def createContainer(self, tag):
        """
        """
        try:
            d = self._user.callRemote('createContainer', tag)
        except DeadReferenceError:
            raise DeadConnection()
        
        d.addErrback(self._reportError)
    
    def destroyContainer(self, tag):
        """
        """
        try:
            d = self._user.callRemote('destroyContainer', tag)
        except DeadReferenceError:
            raise DeadConnection()
        
        d.addErrback(self._reportError)
    
    def addNode(self, cTag, nTag, pkg, exe, args='', name='', namespace=''):
        """
        """
        try:
            d = self._user.callRemote('addNode', cTag, nTag, pkg, exe, args,
                                      name, namespace)
        except DeadReferenceError:
            raise DeadConnection()
        
        d.addErrback(self._reportError)
    
    def removeNode(self, cTag, nTag):
        """
        """
        try:
            d = self._user.callRemote('removeNode', cTag, nTag)
        except DeadReferenceError:
            raise DeadConnection()
        
        d.addErrback(self._reportError)
    
    def addInterface(self, eTag, iTag, iType, clsName, addr=''):
        """
        """
        try:
            d = self._user.callRemote('addInterface', eTag, iTag, iType,
                                      clsName, addr)
        except DeadReferenceError:
            raise DeadConnection()
        
        d.addErrback(self._reportError)
    
    def removeInterface(self, iTag):
        """
        """
        try:
            d = self._user.callRemote('removeInterface', iTag)
        except DeadReferenceError:
            raise DeadConnection()
        
        d.addErrback(self._reportError)
    
    def addParameter(self, cTag, name, value):
        """
        """
        try:
            d = self._user.callRemote('addParameter', cTag, name, value)
        except DeadReferenceError:
            raise DeadConnection()
        
        d.addErrback(self._reportError)
    
    def removeParameter(self, cTag, name):
        """
        """
        try:
            d = self._user.callRemote('removeParameter', cTag, name)
        except DeadReferenceError:
            raise DeadConnection()
        
        d.addErrback(self._reportError)
    
    def addConnection(self, tagA, tagB):
        """
        """
        try:
            d = self._user.callRemote('addConnection', tagA, tagB)
        except DeadReferenceError:
            raise DeadConnection()
        
        d.addErrback(self._reportError)
    
    def removeConnection(self, tagA, tagB):
        """
        """
        try:
            d = self._user.callRemote('removeConnection', tagA, tagB)
        except DeadReferenceError:
            raise DeadConnection()
        
        d.addErrback(self._reportError)
    
    def receivedFromClient(self, iTag, msgType, msgID, msg):
        """
        """
        try:
            self._interfaces[iTag].receive(msgType, msgID, msg)
        except DeadReferenceError:
            raise DeadConnection()
    
    def sendToClient(self, iTag, msgType, msgID, msg):
        """
        """
        self._connection.sendDataMessage(iTag, msgType, msgID, msg)
    
    def registerConnectionToRobot(self, connection):
        """
        """
        self._connection = connection
    
    def unregisterConnectionToRobot(self):
        """
        """
        self._connection = None
    
    def remote_createInterface(self, uid, iType, msgType, addr):
        """
        """
        return self._MAP[iType](self, UUID(bytes=uid), msgType, addr)
    
    def registerInterface(self, interface):
        # "Special" method to account for 'dict' instead of standard 'set'
        tag = interface.tag
        
        assert tag not in self._interfaces
        self._interfaces[tag] = interface
    
    def unregisterInterface(self, interface):
        # "Special" method to account for 'dict' instead of standard 'set'
        tag = interface.tag
        
        assert tag in self._interfaces
        del self._interfaces[tag]
    
    def remote_destroy(self):
        """
        """
        if self._connection:
            self._connection.dropConnection()
        
        for interface in self._interfaces.values():
            interface.remote_destroy()
        
        assert len(self._interfaces) == 0


class RobotClient(Endpoint):
    """
    """
    implements(IRealm, ICredentialsChecker)
    
    credentialInterfaces = (IRobotCredentials,)
    
    def __init__(self, reactor):
        """
        """
        Endpoint.__init__(self, reactor)
        
        self._pendingRobots = {}
    
    def requestAvatar(self, avatarId, mind, *interfaces):
        """ 
            
            From IRealm.
        """
        if IRobot not in interfaces:
            raise NotImplementedError('RobotClient only handles IRobot.')
        
        try:
            robot = self._pendingRobots.pop(avatarId)[1]
        except KeyError:
            raise InternalError('There is no avatar matching the ID.')
        
        return IRobot, robot, robot.remote_destroy
    
    def _passwordMatch(self, matched, avatarId):
        """
        """
        if matched:
            return avatarId
        else:
            return Failure(UnauthorizedLogin())

    def requestAvatarId(self, credentials):
        """ 
            
            From ICredentialsChecker.
        """
        avatarId = (credentials.userID, credentials.robotID)
        
        if avatarId in self._pendingRobots:
            d = maybeDeferred(credentials.checkKey,
                              self._pendingRobots[avatarId][0])
            return d.addCallback(self._passwordMatch, avatarId)
        else:
            return fail(UnauthorizedLogin())
    
    def remote_createNamespace(self, user, userID, robotID, key):
        """
        """
        uid = (userID, robotID)
        
        if uid in self._pendingRobots:
            raise InternalError('Can not create the same robot twice.')
        
        robot = Robot(user)
        self._pendingRobots[uid] = key, robot
        return robot


def main(reactor, cred, masterIP, masterPort, robotPort):
    log.startLogging(sys.stdout)
    
    def _err(reason):
        print(reason)
        reactor.stop()
    
    factory = PBClientFactory()
    reactor.connectTCP(masterIP, masterPort, factory)
    
    client = RobotClient(reactor)
    d = factory.login(cred, client)
    d.addCallback(lambda ref: setattr(client, '__ref', ref))
    d.addErrback(_err)
    
    portal = Portal(client, (client,))
    robot = CloudEngineWebSocketFactory(RobotWebSocketProtocol, portal,
                                        'ws://localhost:{0}'.format(robotPort))
    listenWS(robot)
    
    reactor.addSystemEventTrigger('before', 'shutdown', client.terminate)
    reactor.run()
