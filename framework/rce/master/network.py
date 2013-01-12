#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     network.py
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
from twisted.spread.pb import Referenceable, DeadReferenceError

# Custom imports
from rce.error import InternalError, ConnectionError, InvalidKey
from rce.master.base import Proxy


class Network(object):
    """
    """
    def __init__(self):
        """
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
        """
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
        """
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
        """
        """
        for endpoint in self._endpoints.keys():
            endpoint.destroy()
        
        assert len(self._endpoints) == 0


class Protocol(Proxy):
    """
    """
    def __init__(self, endpoint):
        """
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
        """
        """
        self._endpoint.unregisterProtocol(self)
        self._endpoint = None
        
        # Endpoint should destroy all connections
        assert len(self._connections) == 0
        
        super(Protocol, self).destroy()
    
    @Proxy.destroyProxy
    def _destroy(self):
        try:
            self.obj.callRemote('destroy')
        except DeadReferenceError:
            pass


class Endpoint(Proxy):
    """
    """
    def __init__(self, network):
        """
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
    
    def getUID(self):
        """
        """
        while 1:
            uid = uuid4()
            
            if uid not in self._uids:
                self._uids.add(uid)
                return uid
    
    def returnUID(self, uid):
        """
        """
        assert uid in self._uids
        self._uids.remove(uid)
    
    def createNamespace(self, *args, **kw):
        """
        """
        raise NotImplementedError()
    
    def getLoopback(self):
        """
        """
        if not self._loopback:
            self._loopback = self._getLoopback()
        
        return self._loopback
    
    @Proxy.returnDeferred
    def prepareConnection(self, connID, key, auth):
        """
        """
        return self.obj.callRemote('prepareConnection', connID, key, auth)
    
    @Proxy.returnDeferred
    def connect(self, connID, addr):
        """
        """
        return self.obj.callRemote('connect', connID, addr)
    
    @Proxy.returnProxy(Protocol)
    def _getLoopback(self):
        return self.obj.callRemote('getLoopback')
    
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
    
    def getInterfaceConnection(self, interface, protocol):
        """
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
    
    def destroy(self):
        """
        """
        # Protocols should be implicitly destroyed by the Network
        # Interfaces should be implicitly destroyed by the Namespaces
        
        for namespace in self._namespaces.copy():
            namespace.destroy()
        
        self._network.unregisterEndpoint(self)
        self._network = None
        
        assert len(self._protocols) == 0
        assert len(self._interfaces) == 0
        assert len(self._namespaces) == 0
        
        super(Endpoint, self).destroy()
    
    @Proxy.destroyProxy
    def _destroy(self):
        self.obj.broker.transport.loseConnection()


class Interface(Proxy):
    """
    """
    def __init__(self, namespace):
        """
        """
        super(Interface, self).__init__()
        
        self._endpoint = namespace.endpoint
        self._endpoint.registerInterface(self)
        
        self._namespace = namespace
        namespace.registerInterface(self)
        
        self._uid = None
        
        self._connections = set()
    
    @property
    def UID(self):
        """ ... """
        assert self._uid is not None
        return self._uid
    
    @UID.setter
    def UID(self, uid):
        assert self._uid is None
        self._uid = uid
    
    @property
    def endpoint(self):
        """ ... """
        return self._endpoint
    
    def registerConnection(self, connection):
        assert connection not in self._connections
        self._connections.add(connection)
    
    def unregisterConnection(self, connection):
        assert connection in self._connections
        self._connections.remove(connection)
    
    def registerRemoteID(self, protocol, remoteID):
        """
        """
        return self._registerRemoteID(protocol, remoteID)
    
    def unregisterRemoteID(self, protocol, remoteID):
        """
        """
        return self._unregisterRemoteID(protocol, remoteID)
    
    @Proxy.returnDeferred
    def _registerRemoteID(self, protocol, remoteID):
        def cb(protocol_, remoteID_):
            return self.obj.callRemote('connect', protocol_.obj,
                                       remoteID_.bytes)
        
        return protocol().addCallback(cb, remoteID)
    
    @Proxy.returnDeferred
    def _unregisterRemoteID(self, protocol, remoteID):
        def cb(protocol_, remoteID_):
            return self.obj.callRemote('disconnect', protocol_.obj,
                                       remoteID_.bytes)
        
        return protocol().addCallback(cb, remoteID)
    
    def destroy(self):
        """
        """
        self._endpoint.unregisterInterface(self)
        self._endpoint = None
        
        self._namespace.unregisterInterface(self)
        self._namespace = None
        
        # Endpoint should destroy all connections
        assert len(self._connections) == 0
        
        super(Interface, self).destroy()
    
    @Proxy.destroyProxy
    def _destroy(self):
        try:
            self.obj.callRemote('destroy')
        except DeadReferenceError:
            pass


class Namespace(Proxy):
    """
    """
    def __init__(self, endpoint):
        """
        """
        super(Namespace, self).__init__()
        
        self._endpoint = endpoint
        endpoint.registerNamespace(self)
        
        self._interfaces = set()
    
    @property
    def endpoint(self):
        """ ... """
        return self._endpoint
    
    def getAddress(self):
        """
        """
        raise NotImplementedError()
    
    def createInterface(self, iType, msgType, addr):
        """
        """
        uid = self._endpoint.getUID()
        interface = self._createInterface(uid.bytes, iType, msgType, addr)
        interface.UID = uid
        return interface
    
    @Proxy.returnProxy(Interface)
    def _createInterface(self, uid, iType, msgType, addr):
        return self.obj.callRemote('createInterface', uid, iType, msgType, addr)
    
    def registerInterface(self, interface):
        assert interface not in self._interfaces
        self._interfaces.add(interface)
    
    def unregisterInterface(self, interface):
        assert interface in self._interfaces
        self._interfaces.remove(interface)
    
    def destroy(self):
        """
        """
        for interface in self._interfaces.copy():
            interface.destroy()
        
        assert len(self._interfaces) == 0
        
        self._endpoint.unregisterNamespace(self)
        self._endpoint = None
        
        super(Namespace, self).destroy()
    
    @Proxy.destroyProxy
    def _destroy(self):
        try:
            self.obj.callRemote('destroy')
        except DeadReferenceError:
            pass


class _ConnectionValidator(Referenceable):
    """
    """
    def __init__(self, key):
        """
        """
        self._key = key
        self._authenticated = Deferred()
    
    @property
    def result(self):
        """ Get the deferred representing the result of the authentication. """
        return self._authenticated
    
    def remote_verifyKey(self, key, protocol):
        """
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


class EndpointConnection(object):
    """
    """
    def __init__(self, endpointA, endpointB):
        """
        """
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
        ((serverReady, _), (clientReady, _)) = result
        
        if not (serverReady and clientReady):
            # There was a problem in making the server/client ready for the
            # connection attempt
            ### TODO: What should we do here?
            return Failure(InternalError('Server/Client could not be prepared '
                                         'for connection attempt.'))
        
        return self._serverEndpoint.getAddress()
    
    def _connect(self, addr, connectionID):
        return self._clientEndpoint.connect(connectionID,
                                            (addr.host, addr.port))
    
    def _connectPrepError(self, failure, authenticator):
        authenticator.cancel()
        return failure
    
    def _validate(self, result):
        ((serverAuth, clientProtocol), (clientAuth, serverProtocol)) = result
        
        if not (serverAuth and clientAuth):
            # There was a problem in authenticating the connection
            ### TODO: What should we do here?
            return Failure(InternalError('Connection could not be '
                                         'authenticated.'))
        
        self._serverProtocol.createRemoteObject(serverProtocol)
        self._clientProtocol.createRemoteObject(clientProtocol)
    
    def _logError(self, failure):
        try:
            failure.printTraceback()
        except:
            print('Could not print traceback of failure, print error '
                  'message instead:')
            print(failure.getErrorMessage())
    
    def getProtocol(self, endpoint):
        """
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
        """
        """
        self._serverProtocol.destroy()
        self._clientProtocol.destroy()
        
        self._serverProtocol = None
        self._clientProtocol = None
        
        self._serverEndpoint = None
        self._clientEndpoint = None


class InterfaceConnection(object):
    """
    """
    def __init__(self, interface, protocol):
        """
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
        """
        """
        return self._interface.UID
    
    def destroy(self):
        """
        """
        for user in self._users.copy():
            user.destroy()
        
        assert len(self._users) == 0
        
        self._interface.unregisterConnection(self)
        self._interface = None
        
        self._protocol.unregisterConnection(self)
        self._protocol = None


class Connection(object):
    """
    """
    def __init__(self, connectionA, connectionB):
        """
        """
        super(Connection, self).__init__()
        
        self._connectionA = connectionA
        connectionA.registerUser(self, connectionB.getID())
        
        self._connectionB = connectionB
        connectionB.registerUser(self, connectionA.getID())
    
    def destroy(self):
        """
        """
        if self._connectionA:
            self._connectionA.unregisterUser(self, self._connectionB.getID())
            self._connectionB.unregisterUser(self, self._connectionA.getID())
            
            self._connectionA = None
            self._connectionB = None
        else:
            print('network.Connection destroy() called multiple times...')
