#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     endpoint.py
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
from twisted.spread.pb import Referenceable

# Custom imports
from rce.error import ConnectionError
from rce.slave.protocol import Protocol, RCEInternalProtocol
import settings


class Endpoint(Referenceable):
    """
    """
    def __init__(self, reactor):
        """
        """
        self._reactor = reactor
        reactor.listenTCP(settings.RCE_INTERNAL_PORT,
                          RCEInternalServerFactory(self))
        
        self._loopback = Protocol()
        
        self._pendingConnections = {}
        self._protocols = set()
    
    def remote_createNamespace(self, *args, **kw):
        """
        """
        raise NotImplementedError()
    
    def remote_getLoopback(self):
        """
        """
        return self._loopback
    
    def remote_prepareConnection(self, connID, key, auth):
        """
        """
        assert connID not in self._pendingConnections
        self._pendingConnections[connID] = [key, auth]
    
    def remote_connect(self, connID, addr):
        """
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
        """
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
        """
        """
        self._pendingConnections = {}
        
        for protocol in self._protocols.copy():
            protocol.remote_destroy()
        
        assert len(self._protocols) == 0
        
        self._loopback.remote_destroy()
        self._loopback = None
        
        self._factory = None


class RCEInternalServerFactory(ServerFactory):
    def __init__(self, endpoint):
        self._endpoint = endpoint
    
    def buildProtocol(self, addr):
        return RCEInternalProtocol(self._endpoint)
    