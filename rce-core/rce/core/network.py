#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-core/rce/core/network.py
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
from uuid import uuid4

# twisted specific imports
from twisted.python.failure import Failure
from twisted.internet.defer import Deferred, DeferredList
from twisted.spread.pb import Referenceable, Error, PBConnectionLost, Avatar

# rce specific imports
from rce.util.error import InternalError
from rce.core.base import Proxy, AlreadyDead


class ConnectionError(Error):
    """ Error is raised when the connection failed unexpectedly.
    """


class InvalidKey(Error):
    """ this class is used to signal an invalid key during the initialization
        of the ROS message connections.
    """


class Network(object):
    """ The network is responsible for keeping track of all endpoints,
        namespaces, and interfaces in the cloud engine. Additionally, it
        provides the method to connect two interface.
    """
    def __init__(self):
        """ Initialize the Network.
        """
        self._endpoints = {}

    def registerEndpoint(self, endpoint):
        assert endpoint not in self._endpoints
        self._endpoints[endpoint] = set()

    def unregisterEndpoint(self, endpoint):
        assert endpoint in self._endpoints

        # First remove the endpoint from the dictionary
        endedConnections = self._endpoints.pop(endpoint)

        # Inform the endpoint connections that they are no longer valid
        for connection in endedConnections:
            connection.destroy()

        # Now remove all references to the ended connections
        for connections in self._endpoints.itervalues():
            connections -= endedConnections

    def _getEndpointConnection(self, epA, epB):
        """ Internally used method to get the connection between two endpoints.

            @param epX:         The endpoint which is part of the connection
                                that should be retrieved.
            @type  epX:         rce.core.network.Endpoint

            @return:            Connection between the two endpoints.
            @rtype:             rce.core.network.EndpointConnection
        """
        if epA not in self._endpoints or epB not in self._endpoints:
            raise InternalError('Endpoint is not part of this network.')

        if epA == epB:
            return epA.getLoopback()
        else:
            connectionsA = self._endpoints[epA]
            connectionsB = self._endpoints[epB]

            candidates = connectionsA.intersection(connectionsB)

            if candidates:
                if len(candidates) != 1:
                    raise InternalError('There are more than one possible '
                                        'endpoint connections.')

                return candidates.pop()
            else:
                connection = EndpointConnection(epA, epB)
                connectionsA.add(connection)
                connectionsB.add(connection)
                return connection

    def createConnection(self, interfaceA, interfaceB):
        """ Create a connection between two interfaces.

            @param interfaceX:  The interface which should be connected.
            @type  interfaceX:  rce.core.network.Interface

            @return:            Connection between the two interfaces.
            @rtype:             rce.core.network.Connection
        """
        assert interfaceA != interfaceB

        epA = interfaceA.endpoint
        epB = interfaceB.endpoint

        epA_epB = self._getEndpointConnection(epA, epB)

        pA_iA = epA.getInterfaceConnection(interfaceA,
                                           epA_epB.getProtocol(epA))
        pB_iB = epB.getInterfaceConnection(interfaceB,
                                           epA_epB.getProtocol(epB))

        return Connection(pA_iA, pB_iB)

    def cleanUp(self):
        """ Method should be called to destroy all machines and therefore all
            namespaces and interfaces.
        """
        for endpoint in self._endpoints.keys():
            endpoint.destroy()

        assert len(self._endpoints) == 0


class Endpoint(Proxy):
    """ Representation of an endpoint, which is a process which is part of the
        cloud engine internal communication.

        This class is an abstract implementation, where some methods have to
        be adapted.
    """
    def __init__(self, network):
        """ Initialize the Endpoint.

            @param network:     Network to which the endpoint belongs.
            @type  network:     rce.core.network.Network
        """
        super(Endpoint, self).__init__()

        self._network = network
        network.registerEndpoint(self)

        self._addr = None
        self._loopback = None
        self._namespaces = set()
        self._interfaces = {}
        self._protocols = {}

        self._uids = set()

    def getAddress(self):
        """ Get the address of the endpoint's internal communication server.

            @return:            Address of the endpoint's internal
                                communication server.
                                (type: twisted.internet.address.IPv4Address)
            @rtype:             twisted.internet.defer.Deferred
        """
        raise NotImplementedError('Endpoint can not be used directly.')

    def getUID(self):
        """ Get a ID which is unique within the endpoint.

            @return:            Unique ID (within the endpoint)
            @rtype:             uuid.UUID
        """
        while 1:
            uid = uuid4()

            if uid not in self._uids:
                self._uids.add(uid)
                return uid

    def returnUID(self, uid):
        """ Return a unique ID such that it could be reused again.

            @return:            Unique ID which should be returned.
            @rtype:             uuid.UUID
        """
        assert uid in self._uids
        self._uids.remove(uid)

    def createNamespace(self):
        """ Create a Namespace object in the endpoint.

            @return:            New namespace instance.
            @rtype:             rce.core.namespace.Namespace
                                (subclass of rce.core.base.Proxy)
        """
        raise NotImplementedError('Endpoint can not be used directly.')

    def getLoopback(self):
        """ Get the 'loopback' protocol, which is a special protocol that is
            used to connect two interfaces which are in the same endpoint.

            @return:            Loopback protocol.
            @rtype:             rce.core.network.LoopbackConnection
        """
        if not self._loopback:
            protocol = Protocol(self)
            self._loopback = LoopbackConnection(protocol)
            self.callRemote('getLoopback').chainDeferred(protocol)

        return self._loopback

    def prepareConnection(self, connID, key, auth):
        """ Prepare the endpoint for the connection attempt by adding the
            necessary connection information to the remote process. When the
            returned Deferred fires the endpoint is ready for the connection.

            @param connID:      Unique ID which is used to identify the
                                connection and respond with the appropriate
                                key.
            @type  connID:      str

            @param key:         Key which is used by the other side to
                                authenticate the endpoint.
            @type  key:         str

            @param auth:        Authenticator which is used to validate the
                                key from the other side.
            @type  auth:        rce.core.network._ConnectionValidator

            @return:            None. Deferred fires as soon as the endpoint is
                                ready for the connection attempt.
            @rtype:             twisted.internet.defer.Deferred
        """
        return self.callRemote('prepareConnection', connID, key, auth)

    def connect(self, connID, addr):
        """ Tell the endpoint to connect to the given address using the
            authentication details matching the given connection ID. This
            means that the connection has to be first prepared using
            'prepareConnection', before the actual command can be sent.
            Only one side should get the 'connect' command, because the
            receiver will be the client and not both sides can be the client.

            @param connID:      Unique ID which is used to identify the
                                connection.
            @type  connID:      str

            @param addr:        Address to which the endpoint should connect.
                                It consists of an IP address and a port number.
            @type  addr:        (str, int)

            @return:            None.
            @rtype:             twisted.internet.defer.Deferred
        """
        return self.callRemote('connect', connID, addr)

    def registerNamespace(self, namespace):
        assert namespace not in self._namespaces
        self._namespaces.add(namespace)

    def registerInterface(self, interface):
        assert interface not in self._interfaces
        self._interfaces[interface] = set()

    def registerProtocol(self, protocol):
        assert protocol not in self._protocols
        self._protocols[protocol] = set()

    def unregisterNamespace(self, namespace):
        assert namespace in self._namespaces
        self._namespaces.remove(namespace)

    def unregisterInterface(self, interface):
        assert interface in self._interfaces

        # First remove the interface from the dictionary
        endedConnections = self._interfaces.pop(interface)

        # Inform the interface connections that they are no longer valid
        for connection in endedConnections:
            connection.destroy()

        # Now remove all references to the ended connections
        for connections in self._protocols.itervalues():
            connections -= endedConnections

    def unregisterProtocol(self, protocol):
        assert protocol in self._protocols

        # First remove the protocol from the dictionary
        endedConnections = self._protocols.pop(protocol)

        # Inform the interface connections that they are no longer valid
        for connection in endedConnections:
            connection.destroy()

        # Now remove all references to the ended connections
        for connections in self._interfaces.itervalues():
            connections -= endedConnections

        # Handle special case where the protocol is the Loopback protocol
        if self._loopback == protocol:
            self._loopback = None

    def getInterfaceConnection(self, interface, protocol):
        """ Get the connection between an interface and a protocol.

            @param interface:   Interface which belongs to this endpoint and
                                which is on one side of the connection.
            @type  interface:   rce.core.network.Interface

            @param protocol:    Protocol which belongs to this endpoint and
                                which is on one side of the connection.
            @type  protocol:    rce.core.network.Protocol

            @return:            Connection between the interface and the
                                protocol.
            @rtype:             rce.core.network.InterfaceConnection
        """
        try:
            connectionI = self._interfaces[interface]
        except KeyError:
            raise InternalError('Interface does not belong to this endpoint.')

        try:
            connectionP = self._protocols[protocol]
        except KeyError:
            raise InternalError('Protocol does not belong to this endpoint.')

        candidates = connectionP.intersection(connectionI)

        if candidates:
            if len(candidates) != 1:
                raise InternalError('There are more than one possible '
                                    'interface-protocol connections.')

            return candidates.pop()
        else:
            connection = InterfaceConnection(interface, protocol)
            connectionI.add(connection)
            connectionP.add(connection)
            return connection

    def destroyNamespace(self, remoteNamespace):
        """ Method should be called to destroy the namespace proxy referenced by
            the remote namespace.

            @param remoteNamespace: Reference to Namespace in Remote process.
            @type  remoteNamespace: twisted.spread.pb.RemoteReference
        """
        for namespace in self._namespaces:
            if namespace.destroyExternal(remoteNamespace):
                break

    def destroyProtocol(self, remoteProtocol):
        """ Method should be called to destroy the protocol proxy referenced by
            the remote namespace.

            @param remoteProtocol:  Reference to Protocol in Remote process.
            @type  remoteProtocol:  twisted.spread.pb.RemoteReference
        """
        for protocol in self._protocols:
            if protocol.destroyExternal(remoteProtocol):
                break

    def destroyInterface(self, remoteInterface):
        """ Method should be called to destroy the interface proxy referenced by
            the remote namespace.

            @param remoteInterface: Reference to Interface in Remote process.
            @type  remoteInterface: twisted.spread.pb.RemoteReference
        """
        for interface in self._interfaces:
            if interface.destroyExternal(remoteInterface):
                break

    def destroy(self):
        """ Method should be called to destroy the endpoint and will take care
            of destroying all objects owned by this Endpoint as well as
            deleting all circular references.
        """
        # Protocols should be implicitly destroyed by the Network
        # Interfaces should be implicitly destroyed by the Namespaces

        if self._loopback:
            self._loopback.destroy()
            self._loopback = None

        for namespace in self._namespaces.copy():
            namespace.destroy()

        self._network.unregisterEndpoint(self)
        self._network = None

        assert len(self._protocols) == 0
        assert len(self._interfaces) == 0
        assert len(self._namespaces) == 0

        super(Endpoint, self).destroy()


class EndpointAvatar(Avatar):
    """ Avatar for internal PB connection from an Endpoint.
    """
    def __init__(self, realm, endpoint):
        """ Initialize the Endpoint avatar.

            @param realm:       User realm from which a user object can be
                                retrieved.
            @type  realm:       # TODO: Check this

            @param endpoint:    Representation of the Endpoint.
            @type  endpoint:    rce.core.network.Endpoint
        """
        self._realm = realm  # Required in subclass
        self._endpoint = endpoint

    def perspective_setupNamespace(self, remoteNamespace):
        """ Register a namespace with the Master process.

            @param remoteNamespace: Reference to the Namesapce in the slave
                                    process.
            @type  remoteNamespace: twisted.spread.pb.RemoteReference
        """
        raise NotImplementedError

    def perspective_interfaceDied(self, remoteInterface):
        """ Notify that a remote interface died.

            @param remoteInterface: Reference to the Interface in the slave
                                    process.
            @type  remoteInterface: twisted.spread.pb.RemoteReference
        """
        self._endpoint.destroyInterface(remoteInterface)

    def perspective_protocolDied(self, remoteProtocol):
        """ Notify that a remote protocol died.

            @param remoteProtocol:  Reference to the Protocol in the slave
                                    process.
            @type  remoteProtocol:  twisted.spread.pb.RemoteReference
        """
        self._endpoint.destroyProtocol(remoteProtocol)

    def perspective_namespaceDied(self, remoteNamespace):
        """ Notify that a remote namespace died.

            @param remoteNamespace: Reference to the Namespace in the slave
                                    process.
            @type  remoteNamespace: twisted.spread.pb.RemoteReference
        """
        self._endpoint.destroyNamespace(remoteNamespace)

    def logout(self):
        """ Callback which should be called upon disconnection of the Endpoint.
        """
        self._endpoint.destroy()


class Namespace(Proxy):
    """ Representation of a namespace, which is part of the cloud engine
        internal communication.
    """
    def __init__(self, endpoint):
        """ Initialize the Namespace.

            @param endpoint:    Endpoint in which the namespace was created.
            @type  endpoint:    rce.master.network.Endpoint
        """
        super(Namespace, self).__init__()

        self._endpoint = endpoint
        endpoint.registerNamespace(self)

        self._interfaces = set()

    def createInterface(self, iType, clsName, addr):
        """ Create an Interface object in the namespace and therefore endpoint.

            @param iType:       Type of the interface encoded as an integer.
                                Refer to rce.slave.interface.Types for more
                                information.
            @type  IType:       int

            @param clsName:     Message type/Service type consisting of the
                                package and the name of the message/service,
                                i.e. 'std_msgs/Int32'.
            @type  clsName:     str

            @return:            New Interface instance.
            @rtype:             rce.core.network.Interface
                                (subclass of rce.core.base.Proxy)
        """
        uid = self._endpoint.getUID()
        interface = Interface(self._endpoint, self, uid)
        self.callRemote('createInterface', uid.bytes, iType, clsName,
                        addr).chainDeferred(interface)
        return interface

    def registerInterface(self, interface):
        assert interface not in self._interfaces
        self._interfaces.add(interface)

    def unregisterInterface(self, interface):
        assert interface in self._interfaces
        self._interfaces.remove(interface)

    def destroy(self):
        """ Method should be called to destroy the namespace and will take care
            of destroying all objects owned by this Namespace as well as
            deleting all circular references.
        """
        for interface in self._interfaces.copy():
            interface.destroy()

        assert len(self._interfaces) == 0

        self._endpoint.unregisterNamespace(self)
        self._endpoint = None

        super(Namespace, self).destroy()


class Interface(Proxy):
    """ Representation of an interface, which is part of the cloud engine
        internal communication.
    """
    def __init__(self, endpoint, namespace, uid):
        """ Initialize the Interface.

            @param endpoint:    Endpoint in which the interface is created.
            @type  endpoint:    rce.core.network.Endpoint

            @param namespace:   Namespace in which the interface is created.
            @type  namespace:   rce.core.network.Namespace

            @param uid:         Unique ID which is used to identify the
                                interface in the internal communication.
            @type  uid:         uuid.UUID
        """
        super(Interface, self).__init__()

        self._endpoint = endpoint
        endpoint.registerInterface(self)

        self._namespace = namespace
        namespace.registerInterface(self)

        self._uid = uid

        self._connections = set()

    @property
    def UID(self):
        """ Unique ID of the interface. """
        return self._uid

    @property
    def endpoint(self):
        """ Reference to endpoint to which this interface belongs. """
        return self._endpoint

    def registerConnection(self, connection):
        assert connection not in self._connections
        self._connections.add(connection)

    def unregisterConnection(self, connection):
        assert connection in self._connections
        self._connections.remove(connection)

    def registerRemoteID(self, protocol, remoteID):
        """ Register an interface (using its unique ID) with this interface,
            which means that this and the indicated interface will be
            connected.

            @param protocol:    Protocol which provides the connection to the
                                other side, i.e. the other endpoint. (Could
                                also be the loopback protocol which would mean
                                that both interfaces are in the same endpoint.)
            @type  protocol:    rce.master.network.Protocol

            @param remoteID:    Unique ID of the interface at the other side
                                of the connection, i.e. in the other endpoint.
            @type  remoteID:    uuid.UUID

            @return:            None.
            @rtype:             twisted.internet.defer.Deferred
        """
        return protocol().addCallback(self._remoteID, remoteID, 'connect')

    def unregisterRemoteID(self, protocol, remoteID):
        """ Unregister an interface (using its unique ID) with this interface,
            which means that this and the indicated interface will no longer
            be connected.

            @param protocol:    Protocol which provides the connection to the
                                other side, i.e. the other endpoint. (Could
                                also be the loopback protocol which would mean
                                that both interfaces are in the same endpoint.)
            @type  protocol:    rce.core.network.Protocol

            @param remoteID:    Unique ID of the interface at the other side
                                of the connection, i.e. in the other endpoint.
            @type  remoteID:    uuid.UUID

            @return:            None.
            @rtype:             twisted.internet.defer.Deferred
        """
        def eb(failure):
            failure.trap(PBConnectionLost)

        d = protocol()
        d.addCallback(self._remoteID, remoteID, 'disconnect')
        d.addErrback(eb)
        return d

    def _remoteID(self, protocol, remoteID, name):
        return self.callRemote(name, protocol, remoteID.bytes)

    def destroy(self):
        """ Method should be called to destroy the interface and will take care
            of destroying all objects owned by this Interface as well as
            deleting all circular references.
        """
        self._endpoint.unregisterInterface(self)
        self._endpoint = None

        self._namespace.unregisterInterface(self)
        self._namespace = None

        # Endpoint should destroy all connections
        assert len(self._connections) == 0

        super(Interface, self).destroy()


class Protocol(Proxy):
    """ Representation of a protocol, which is part of the cloud engine
        internal communication.
    """
    def __init__(self, endpoint):
        """ Initialize the Protocol.

            @param endpoint:    Endpoint in which the protocol was created.
            @type  endpoint:    rce.core.network.Endpoint
        """
        super(Protocol, self).__init__()

        self._endpoint = endpoint
        endpoint.registerProtocol(self)

        self._connections = set()

    def registerConnection(self, connection):
        assert connection not in self._connections
        self._connections.add(connection)

    def unregisterConnection(self, connection):
        assert connection in self._connections
        self._connections.remove(connection)

    def destroy(self):
        """ Method should be called to destroy the protocol and will take care
            of destroying all objects owned by this Protocol as well as
            deleting all circular references.
        """
        # TODO: WHY ???
        if not self._endpoint:
            return

        self._endpoint.unregisterProtocol(self)
        self._endpoint = None

        # Endpoint should destroy all connections
        assert len(self._connections) == 0

        super(Protocol, self).destroy()


class _ConnectionValidator(Referenceable):
    """ Small helper to provide a callback to the endpoint to validate the
        connection and to register the protocol who is responsible for the it.
    """
    def __init__(self, key):
        """ Initialize the Connection Validator.

            @param key:         Key which is the correct.
            @type  key:         str
        """
        self._key = key
        self._authenticated = Deferred()

    @property
    def result(self):
        """ Get the result of the validation as soon as its available.

            @return:            Protocol instance which sent the validation
                                request.
                                (type: twisted.spred.pb.RemoteReference)
            @rtype:             twisted.internet.defer.Deferred
        """
        # For now only one Deferred is used for the result as there is no
        # reason to have a list here
        return self._authenticated

    def remote_verifyKey(self, key, protocol):
        """ Verify that the provided key matches the correct value. This method
            can be called only once, after which an error is raised.

            In case the key could be validated the protocol is registered with
            the connection.

            @param key:         Key which should be validated.
            @type  key:         str

            @param protocol:    Protocol who is responsible for the
                                validation request and one partner of the
                                new connection.
            @type  protocol:    twisted.spread.pb.RemoteReference

            @return:            None
        """
        if self._authenticated.called:
            return Failure(InvalidKey('Only one guess is possible.'))

        if isinstance(protocol, Failure):
            self._authenticated.errback(protocol)
        else:
            if self._key != key:
                e = Failure(InvalidKey('Wrong key supplied.'))
                self._authenticated.errback(e)
                return e

            self._authenticated.callback(protocol)


class LoopbackConnection(object):
    def __init__(self, protocol):
        """ Initialize the loopback connection for an endpoint.

            @param protocol:    Protocol instance providing loopback
                                functionality in the endpoint.
            @type  protocol:    rce.core.network.Protocol
        """
        self._protocol = protocol

    def getProtocol(self, _):
        """ Get the protocol which is part of this connection.

            @return:            Protocol which belongs to the endpoint and is
                                part of this connection.
            @rtype:             rce.core.network.Protocol
                                (subclass of rce.core.base.Proxy)
        """
        return self._protocol

    def destroy(self):
        """ Method should be called to destroy the endpoint connection and will
            take care of destroying the participating protocols as well as
            deleting all circular references.
        """
        self._protocol.destroy()

        self._protocol = None


class EndpointConnection(object):
    """ Representation of a connection between two endpoints, where the two
        endpoints are not the same.
    """
    def __init__(self, endpointA, endpointB):
        """ Initialize the connection between the two endpoints.
            The connection will be scheduled to be created here.

            @param endpointX:   Endpoint which is part of the new connection.
            @type  endpointX:   rce.core.network.Endpoint
        """
        assert endpointA != endpointB

        # Register the endpoints and create Protocol proxies
        self._serverEndpoint = endpointA
        self._clientEndpoint = endpointB

        self._serverProtocol = Protocol(endpointA)
        self._clientProtocol = Protocol(endpointB)

        # Create the keys used to validate the connection
        connectionID = uuid4().bytes
        serverKey = uuid4().bytes
        clientKey = uuid4().bytes

        # Create Validators and get result Deferred
        authServer = _ConnectionValidator(serverKey)
        authClient = _ConnectionValidator(clientKey)
        authServerResult = authServer.result
        authClientResult = authClient.result

        authenticator = DeferredList([authServerResult, authClientResult])
        authenticator.addCallback(self._validate)
        authenticator.addErrback(self._logError)

        readyServer = endpointA.prepareConnection(connectionID, serverKey,
                                                  authClient)
        readyClient = endpointB.prepareConnection(connectionID, clientKey,
                                                  authServer)
        ready = DeferredList([readyServer, readyClient])
        ready.addCallback(self._getAddress)
        ready.addCallback(self._connect, connectionID)
        ready.addErrback(self._connectPrepError, authenticator)
        ready.addErrback(self._logError)

    def _getAddress(self, result):
        """ Internally used method which is part of a callback chain.
            Its task is to verify that both endpoints are ready for the
            connection attempt. In case both signal readiness the address
            of the designated server endpoint is retrieved.

            @param result:      Response of the DeferredList containing the
                                Deferreds of the 'prepareConnection' calls.

            @return:            Address of the endpoint's internal
                                communication server.
            @rtype:             twisted.internet.address.IPv4Address
        """
        ((serverReady, _), (clientReady, _)) = result

        if not (serverReady and clientReady):
            # There was a problem in making the server/client ready for the
            # connection attempt
            # TODO: What should we do here?
            return Failure(InternalError('Server/Client could not be prepared '
                                         'for connection attempt.'))

        return self._serverEndpoint.getAddress()

    def _connect(self, addr, connID):
        """ Internally used method which is part of a callback chain.
            Its task is to send the 'connect' command to the client.

            @param addr:        Address of the endpoint's internal
                                communication server.
            @rtype:             twisted.internet.address.IPv4Address

            @param connID:      Connection ID which is used to identify the
                                appropriate authentication key.
            @type  connID:      str

            @return:            None.
            @rtype:             twisted.internet.defer.Deferred
        """
        return self._clientEndpoint.connect(connID, (addr.host, addr.port))

    def _connectPrepError(self, failure, authenticator):
        """ Internally used method which is part of an errback chain.
            Its task is to signal to the authenticator that the connection can
            not be attempted by cancelling the Deferred.
        """
        authenticator.cancel()
        return failure

    def _validate(self, result):
        """ Internally used method which is part of a callback chain.
            Its task is to verify that both sides of the connection could be
            authenticated. In case both signal success the corresponding
            protocols are registered.
        """
        ((serverAuth, clientProtocol), (clientAuth, serverProtocol)) = result

        if not (serverAuth and clientAuth):
            # There was a problem in authenticating the connection
            # TODO: What should we do here?
            return Failure(InternalError('Connection could not be '
                                         'authenticated.'))

        self._serverProtocol.callback(serverProtocol)
        self._clientProtocol.callback(clientProtocol)

    def _logError(self, failure):
        """ Internally used method to print out the errors for now...
        """
        try:
            failure.printTraceback()
        except:
            print('Could not print traceback of failure, print error '
                  'message instead:')
            print(failure.getErrorMessage())

    def getProtocol(self, endpoint):
        """ Get the protocol which is part of this connection and belongs to
            the given endpoint.

            @param endpoint:    Endpoint to which the protocol has to belong.
            @type  endpoint:    rce.core.network.Endpoint

            @return:            Protocol which belongs to the endpoint and is
                                part of this connection.
            @rtype:             rce.core.network.Protocol
                                (subclass of rce.core.base.Proxy)
        """
        if not (self._serverProtocol and self._serverEndpoint and
                self._clientProtocol and self._clientEndpoint):
            raise ConnectionError('Endpoint connection is dead.')

        if self._serverEndpoint == endpoint:
            return self._serverProtocol
        elif self._clientEndpoint == endpoint:
            return self._clientProtocol
        else:
            raise InternalError('The endpoint is not part of this connection.')

    def destroy(self):
        """ Method should be called to destroy the endpoint connection and will
            take care of destroying the participating protocols as well as
            deleting all circular references.
        """
        self._serverProtocol.destroy()
        self._clientProtocol.destroy()

        self._serverProtocol = None
        self._clientProtocol = None

        self._serverEndpoint = None
        self._clientEndpoint = None


class InterfaceConnection(object):
    """ Representation of a connection between an interface and a protocol,
        where both blong to the same endpoint.
    """
    def __init__(self, interface, protocol):
        """ Initialize the interface-protocol connection.

            @param interface:   Interface which is on one side of the
                                connection.
            @type  interface:   rce.core.network.Interface

            @param protocol:    Protocol which is on one side of the
                                connection.
            @type  protocol:    rce.core.network.Protocol
        """
        assert protocol._endpoint == interface._endpoint

        self._interface = interface
        interface.registerConnection(self)

        self._protocol = protocol
        protocol.registerConnection(self)

        self._users = set()

    def registerUser(self, connection, remoteID):
        assert connection not in self._users
        self._users.add(connection)
        self._interface.registerRemoteID(self._protocol, remoteID)

    def unregisterUser(self, connection, remoteID):
        assert connection in self._users
        self._users.remove(connection)
        self._interface.unregisterRemoteID(self._protocol, remoteID)

    def getID(self):
        """ Get the unique ID of the interface which is part of this
            connection.

            @return:            Unique ID of the interface.
            @rtype:             uuid.UUID
        """
        return self._interface.UID

    def destroy(self):
        """ Method should be called to destroy the interface-protocol
            connection and will take care of destroying all objects owned by
            this connection as well as deleting all circular references.
        """
        for user in self._users.copy():
            user.destroy()

        assert len(self._users) == 0

        self._interface.unregisterConnection(self)
        self._interface = None

        self._protocol.unregisterConnection(self)
        self._protocol = None


class Connection(object):
    """ Representation of a connection between two interfaces.
    """
    def __init__(self, connectionA, connectionB):
        """ Initialize the connection, which is represented by two
            interface-protocol connections.

            @param connectionX: Interface-Protocol connection which is part
                                of the connection.
            @type  connectionX: rce.core.network.InterfaceConnection
        """
        assert connectionA != connectionB

        self._connectionA = connectionA
        connectionA.registerUser(self, connectionB.getID())

        self._connectionB = connectionB
        connectionB.registerUser(self, connectionA.getID())

        self._cbs = set()

    def notifyOnDeath(self, cb):
        """ Method is used to  to register a callback which will be called
            when the connection died.

            @param cb:          Callback which should be registered. The
                                callback should take the died connection as
                                only argument.
            @type  cb:          callable
        """
        assert callable(cb)

        try:
            self._cbs.add(cb)
        except AttributeError:
            raise AlreadyDead('{0} is already '
                              'dead.'.format(self.__class__.__name__))

    def dontNotifyOnDeath(self, cb):
        """ Method is used to unregister a callback which should have been
            called when the connection died.

            @param cb:          Callback which should be unregistered.
            @type  cb:          callable
        """
        try:
            self._cbs.remove(cb)
        except AttributeError:
            pass

    def destroy(self):
        """ Method should be called to destroy the connection and will
            take care deleting all circular references.
        """
        if self._cbs:
            for cb in self._cbs:
                cb(self)

            self._cbs = None

        if self._connectionA:
            self._connectionA.unregisterUser(self, self._connectionB.getID())
            self._connectionB.unregisterUser(self, self._connectionA.getID())

            self._connectionA = None
            self._connectionB = None
        else:
            print('network.Connection destroy() called multiple times...')
