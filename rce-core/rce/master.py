#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-core/rce/master.py
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
from twisted.cred.portal import IRealm, Portal
from twisted.spread.pb import IPerspective, PBServerFactory
from twisted.web.server import Site

# rce specific imports
from rce.util.error import InternalError
from rce.util.cred import CredentialError
from rce.comm.interfaces import IMasterRealm
from rce.comm.server import RobotResource
from rce.core.machine import LoadBalancer, ContainerProcessError, \
    Distributor, RobotProcessError, MachineAvatar
from rce.core.network import Network
from rce.core.environment import EnvironmentEndpoint, EnvironmentEndpointAvatar
from rce.core.robot import RobotEndpoint, RobotEndpointAvatar
from rce.core.user import User


class UserRealm(object):
    """
    """
    implements(IRealm)

    def __init__(self, rce):
        self._rce = rce

    def requestAvatar(self, avatarId, mind, *interfaces):
        if IPerspective not in interfaces:
            raise NotImplementedError('RoboEarthCloudEngine only '
                                      'handles IPerspective.')

        return IPerspective, self._rce.getUser(avatarId), lambda: None


class RoboEarthCloudEngine(object):
    """ Realm for the twisted cred system. It is responsible for storing all
        cloud engine relevant informations by having a reference to a Network,
        Load Balancer, and Distributor instance, where each is responsible for
        part of the system. Additionally, the realm keeps a list of all User
        object.

        There should be only one instance running in the Master process.
    """
    implements(IRealm, IMasterRealm)

    def __init__(self, checker, port):
        """ Initialize the RoboEarth Cloud Engine realm.

            @param checker:     Login checker which authenticates the User when
                                an initial request is received.
            @type  checker:     twisted.cred.checkers.ICredentialsChecker

            @param port:        Port where the robot process is listening for
                                connections from other endpoints.
            @type  port:        int
        """
        self._checker = checker
        self._port = port

        self._network = Network()
        self._balancer = LoadBalancer()
        self._distributor = Distributor()

        self._users = {}
        self._pendingContainer = {}

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
            avatar = MachineAvatar(machine, self._balancer)
            detach = lambda: avatar.logout()
            print('Connection to Container process established.')
        elif avatarId == 'robot':
            endpoint = RobotEndpoint(self._network, self._distributor,
                                     self._port)
            endpoint.callback(mind)
            avatar = RobotEndpointAvatar(self, endpoint)
            detach = lambda: avatar.logout()
            print('Connection to Robot process established.')
        elif avatarId == 'environment':
            endpoint = self._pendingContainer.pop(mind[1])
            endpoint.callback(mind[0])
            avatar = EnvironmentEndpointAvatar(self, endpoint)
            detach = lambda: avatar.logout()
            print('Connection to Environment process established.')
        else:
            raise InternalError('Invalid avatar ID received.')

        return IPerspective, avatar, detach

    def getUser(self, userID):
        """ Get the user object matching the given user ID.

            @param userID:      user ID of the user which should be retrieved.
            @type  userID:      str

            @return:            User object matching the user ID.
            @rtype:             rce.core.user.User
        """
        if userID not in self._users:
            self._users[userID] = User(self, userID)

        return self._users[userID]

    def requestURL(self, userID):
        """ Callback for Robot resource to retrieve the location of the Robot
            process to which a WebSocket connection should be established.

            @param userID:      User ID under which the robot will login.
                                (Can be used to do optimizations in distributing
                                the load.)
            @type  userID:      str

            @return:            The IP address of Robot process to which a
                                WebSocket connection should be established.
                                (type: str)
            @rtype:             twisted.internet.defer.Deferred
        """
        try:
            location = self._distributor.getNextLocation()
        except RobotProcessError:
            # TODO: What should we do here?
            raise InternalError('Robot can not be created.')

        return location.getWebsocketAddress()

    def createContainer(self, userID, data):
        """ Callback for User instance to create a new Container object in a
            container process.

            @param userID:      UserID of the user who created the container.
            @type  userID:      str

            @param data:        Extra data which is used to configure the
                                container.
            @param data:        dict

            @return:            New Namespace and Container instance.
            @rtype:             (rce.core.environment.Environment,
                                 rce.core.container.Container)
                                 (subclasses of rce.core.base.Proxy)
        """
        while 1:
            uid = uuid4().hex

            if uid not in self._pendingContainer:
                break

        try:
            container = self._balancer.createContainer(uid, userID, data)
        except ContainerProcessError:
            # TODO: What should we do here?
            raise InternalError('Container can not be created.')

        endpoint = EnvironmentEndpoint(self._network, container)
        self._pendingContainer[uid] = endpoint
        return endpoint.createNamespace(), container

    def checkUIDValidity(self, uid):
        """Method to check if incoming environment ID is valid.

            @param uid:         UID to be tested.
            @type  uid:         str
        """
        if uid not in self._pendingContainer:
            raise CredentialError('Invalid environment ID.')

    def createConnection(self, interfaceA, interfaceB):
        """ Callback for User instance to create a new connection between two
            interfaces.

            @param interfaceX:  Interface which should be connected.
            @type  interfaceX:  rce.master.network.Interface

            @return:            New Connection instance.
            @rtype:             rce.core.network.Connection
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
         commPort, consolePort):
    log.startLogging(sys.stdout)

    # Realms
    rce = RoboEarthCloudEngine(externalCred, commPort)
    user = UserRealm(rce)

    internalCred.add_checker(rce.checkUIDValidity)

    # Portals
    rcePortal = Portal(rce, (internalCred,))
    consolePortal = Portal(user, (externalCred,))

    # Internal Communication
    reactor.listenTCP(internalPort, PBServerFactory(rcePortal))

    # Client Connections
    reactor.listenTCP(consolePort, PBServerFactory(consolePortal))
    reactor.listenTCP(externalPort, Site(RobotResource(rce)))

    reactor.addSystemEventTrigger('before', 'shutdown', rce.preShutdown)
    reactor.addSystemEventTrigger('after', 'shutdown', rce.postShutdown)

    reactor.run()
