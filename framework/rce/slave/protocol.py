#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     protocol.py
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
import struct
from uuid import UUID

# twisted specific imports
from twisted.python import log
from twisted.protocols.basic import Int32StringReceiver
from twisted.spread.pb import Referenceable

# Custom imports
from rce.error import InternalError


class Protocol(Referenceable):
    """
    """
    def __init__(self):
        self._receivers = {}
    
    def sendMessage(self, interface, msg, msgID, remoteID=None):
        """
        """
        self.messageReceived(interface.UID, msg, msgID, remoteID)
    
    def messageReceived(self, remoteID, msg, msgID, destID=None):
        """
        """
        if remoteID not in self._receivers:
            log.msg('Received message dropped, because there is no interface '
                    'ready for the message.')
            return
        
        for interface in self._receivers[remoteID]:
            if destID:
                if interface.UID == destID:
                    interface.send(msg, msgID, self, remoteID)
                    break
            else:
                interface.send(msg, msgID, self, remoteID)
    
    def registerConnection(self, interface, remoteID):
        """
        """
        if remoteID not in self._receivers:
            self._receivers[remoteID] = set()
        else:
            assert interface not in self._receivers[remoteID]
        
        self._receivers[remoteID].add(interface)
    
    def unregisterConnection(self, interface, remoteID):
        """
        """
        assert remoteID in self._receivers
        receivers = self._receivers[remoteID]
        
        assert interface in receivers
        receivers.remove(interface)
        
        if not receivers:
            del self._receivers[remoteID]
    
    def remote_destroy(self):
        """
        """
        if self._receivers:
            for interface in reduce(set.union, self._receivers.itervalues()):
                interface.unregisterProtocol(self)
            
            self._receivers = None


class RCEInternalProtocol(Int32StringReceiver, Protocol):
    """
    """
    _MSG_ID_STRUCT = struct.Struct('!B')
    _TRUE = struct.pack('!?', True)
    _FALSE = struct.pack('!?', False)
    
    def __init__(self, endpoint):
        """
        """
        Protocol.__init__(self)
        
        self._endpoint = endpoint
        endpoint.registerProtocol(self)
        
        self._initialized = False
        self.stringReceived = self._initReceived
    
    def _initReceived(self, msg):
        """
        """
        if len(msg) != 32:
            log.msg('Protocol Error: iInit message has invalid format.')
            self.transport.loseConnection()
            return
        
        d = self._endpoint.processInit(self, msg[:16], msg[16:])
        d.addCallbacks(self._initSuccessful, self._initFailed)
    
    def _initSuccessful(self, _):
        self.stringReceived = self._messageReceived
        self._initialized = True
    
    def _initFailed(self, failure):
        log.msg('Protocol Error: {0}'.format(failure.getErrorMessage()))
        self.transport.loseConnection()
    
    def _messageReceived(self, msg):
        """
        """
        if len(msg) < 17:
            self.transport.loseConnection()
        
        flag = msg[:1]
        
        if flag == self._TRUE:
            destID = UUID(bytes=msg[1:17])
            offset = 17
        elif flag == self._FALSE:
            destID = None
            offset = 1
        else:
            log.msg('Protocol Error: Could not identify flag.')
            self.transport.loseConnection()
            return
        
        remoteID = UUID(bytes=msg[offset:offset+16])
        offset += 16
        
        idLen, = self._MSG_ID_STRUCT.unpack(msg[offset:offset+1])
        offset += 1
        
        msgID = msg[offset:offset+idLen]
        offset += idLen
        
        self.messageReceived(remoteID, buffer(msg, offset), msgID, destID)
    
    def sendInit(self, connID, key):
        """
        """
        assert len(connID) == 16
        assert len(key) == 16
        
        self.sendString(connID + key)
    
    def sendMessage(self, interface, msg, msgID, remoteID=None):
        """
        """
        assert self._initialized
        
        uid = interface.UID.bytes
        assert len(uid) == 16
        
        try:
            idLen = self._MSG_ID_STRUCT.pack(len(msgID))
        except struct.error:
            raise InternalError('Message ID is too long.')
        
        if remoteID:
            flag = self._TRUE
            rmtID = remoteID.bytes
            assert len(rmtID) == 16
        else:
            flag = self._FALSE
            rmtID = ''
        
        self.sendString(''.join((flag, rmtID, uid, idLen, msgID, msg)))
    
    def connectionLost(self, reason):
        """
        """
        Protocol.remote_destroy(self)
        
        if self._endpoint:
            self._endpoint.unregisterProtocol(self)
            self._endpoint = None
    
    def remote_destroy(self):
        """
        """
        self.transport.loseConnection()
    
