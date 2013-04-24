#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-core/rce/slave/endpoint.py
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
from twisted.python.failure import Failure
from twisted.internet.defer import fail
from twisted.internet.protocol import ServerFactory, ClientCreator
from twisted.spread.pb import Referenceable, Error

# rce specific imports
from rce.slave.protocol import Loopback, RCEInternalProtocol


class ConnectionError(Error):
    """ Error is raised when the connection failed unexpectedly.
    """


class Endpoint(Referenceable):
    """ Abstract base class for an Endpoint in a slave process.
    """
    def __init__(self, reactor, commPort):
        """ Initialize the Endpoint.

            @param reactor:     Reference to the twisted reactor used in this
                                robot process.
            @type  reactor:     twisted::reactor

            @param commPort:    Port where the server for the cloud engine
                                internal communication will listen for incoming
                                connections.
            @type  commPort:    int
        """
        self._avatar = None
        self._reactor = reactor
        reactor.listenTCP(commPort, _RCEInternalServerFactory(self))

        self._loopback = None

        self._pendingConnections = {}
        self._protocols = set()

    def remote_createNamespace(self, *args, **kw):
        """ Remote callable method to create a namespace in this endpoint.

            Method has to be implemented!

            @return:            New Namespace instance.
            @rtype:             rce.slave.namespace.Namespace
        """
        raise NotImplementedError("Method 'remote_createNamespace' has"
                                  'to be implemented.')

    def remote_getLoopback(self):
        """ Get the loopback protocol.

            @return:            Reference to the loopback protocol.
            @rtype:             rce.slave.protocol.Loopback
        """
        if not self._loopback:
            self._loopback = Loopback(self)

        return self._loopback

    def remote_prepareConnection(self, connID, key, auth):
        """ Prepare the endpoint for the connection attempt by adding the
            necessary connection information to the remote process.

            @param connID:      Unique ID which is used to identify the
                                connection and respond with the appropriate
                                key.
            @type  connID:      str

            @param key:         Key which is sent to the other side to
                                authenticate the endpoint.
            @type  key:         str

            @param auth:        Authenticator which is used to validate the
                                key received from the other side.
            @type  auth:        twisted.spread.pb.RemoteReference
        """
        assert connID not in self._pendingConnections
        self._pendingConnections[connID] = [key, auth]

    def remote_connect(self, connID, addr):
        """ Connect to the endpoint with the given address using the
            connection information matching the received ID.

            @param connID:      Unique ID which is used to identify the tuple
                                containing the connection information.
            @type  connID:      str

            @param addr:        Address to which the endpoint should connect.
                                It consists of an IP address and a port number.
            @type  addr:        (str, int)
        """
        assert connID in self._pendingConnections

        # Retrieve the key which should be sent and replace it with None to
        # indicate that the key has already been sent
        info = self._pendingConnections[connID]
        key, auth = info
        info[0] = None

        client = ClientCreator(self._reactor, RCEInternalProtocol, self)
        d = client.connectTCP(*addr)
        d.addCallback(lambda p: p.sendInit(connID, key))
        d.addErrback(self._connectError, auth)

    def _connectError(self, failure, auth):
        failure.printTraceback()
        # TODO: Signal back the error
        # v does not work
        #auth.callRemote('verifyKey', None, failure)

    def processInit(self, protocol, connID, remoteKey):
        """ Callback for the RCE Internal Protocol which is called when the
            protocol received an init message which has to be processed.

            @param protocol:    Protocol instance which received the init
                                message.
            @type  protocol:    rce.slave.protocol.RCEInternalProtocol

            @param connID:      Unique ID which is used to identify the
                                connection.
            @type  connID:      str

            @param remoteKey:   Key which was received from the other side to
                                authenticate the endpoint.
            @type  remoteKey:   str

            @return:            True if the connection should be accepted.
            @rtype:             twisted.internet.defer.Deferred
        """
        try:
            key, auth = self._pendingConnections[connID]
        except KeyError:
            return fail(Failure(ConnectionError('Connection was not '
                                                'expected.')))

        try:
            if key:
                protocol.sendInit(connID, key)
        except Exception as e:
            failure = Failure(e)
            # TODO: Signal back the error
            # v does not work
            #auth.callRemote('verifyKey', None, failure)
            return fail(failure)
        else:
            return auth.callRemote('verifyKey', remoteKey, protocol)

        # Should never be reached...
        raise RuntimeError()

    def registerProtocol(self, protocol):
        assert protocol not in self._protocols
        self._protocols.add(protocol)

    def unregisterProtocol(self, protocol):
        assert protocol in self._protocols
        self._protocols.remove(protocol)

    def terminate(self):
        """ Method should be called to terminate the endpoint before the
            reactor is stopped.

            @return:            Deferred which fires as soon as the client is
                                ready to stop the reactor.
            @rtype:             twisted.internet.defer.Deferred
        """
        self._pendingConnections = {}

        for protocol in self._protocols.copy():
            protocol.remote_destroy()
        # Can not check here, because protocols are unregistered when the
        # connection is lost and remote_destroy only requests to lose the
        # connection
        #assert len(self._protocols) == 0

        if self._loopback:
            self._loopback.remote_destroy()
            self._loopback = None

        self._factory = None


class _RCEInternalServerFactory(ServerFactory):
    """ Server Factory for the cloud engine internal communication.
    """
    def __init__(self, endpoint):
        self._endpoint = endpoint

    def buildProtocol(self, addr):
        return RCEInternalProtocol(self._endpoint)
