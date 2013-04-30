#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-core/rce/slave/protocol.py
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

# rce specific imports
from rce.util.error import InternalError


class _Protocol(Referenceable):
    """ Abstract base class for a internal Protocol which interacts with the
        Endpoint, Namespace, and Interfaces in a slave process.
    """
    def __init__(self, endpoint):
        """ Initialize the Protocol.
        """
        self._receivers = {}
        self._endpoint = endpoint
        endpoint.registerProtocol(self)

    def sendMessage(self, interface, msg, msgID, remoteID=None):
        """ Send a message received from an Interface to the other side.

            @param interface:   Interface which wants to send the message.
            @type  interface:   rce.slave.interface.Interface

            @param msg:         Message which should be sent.
            @type  msg:         str

            @param msgID:       Unique ID which can be used to find a
                                correspondence between request / response
                                message.
            @type  msgID:       uuid.UUID

            @param remoteID:    If the remote ID is supplied than only this
                                Interface will receive the message, regardless
                                of additional interfaces which might be
                                registered.
            @type  remoteID:    uuid.UUID
        """
        raise NotImplementedError("Method 'sendMessage' has to be "
                                  'implemented.')

    def messageReceived(self, remoteID, msg, msgID, destID=None):
        """ Protocol internal method used to send a received message to the
            stored receivers.

            @param remoteID:    Unique ID of the Interface on the other side
                                which sent the message.
            @type  remoteID:    uuid.UUID

            @param msg:         Message which was received.
            @type  msg:         str

            @param msgID:       Unique ID which can be used to find a
                                correspondence between request / response
                                message.
            @type  msgID:       uuid.UUID

            @param destID:      If the dest ID is supplied than only this
                                Interface will receive the message, regardless
                                of additional interfaces which might be
                                registered.
            @type  destID:      uuid.UUID
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
        """ Register the connection between the local Interface and the remote
            Interface such that the two interfaces can communicate with each
            other.

            @param interface:   Reference to the local Interface.
            @type  interface:   rce.slave.interface.Interface

            @param remoteID:    Unique ID of the remote Interface.
            @type  remoteID:    uuid.UUID
        """
        if remoteID not in self._receivers:
            self._receivers[remoteID] = set()
        else:
            assert interface not in self._receivers[remoteID]

        self._receivers[remoteID].add(interface)

    def unregisterConnection(self, interface, remoteID):
        """ Unregister the connection between the local Interface and the
            remote Interface such that the two interfaces can no longer
            communicate with each other.

            @param interface:   Reference to the local Interface.
            @type  interface:   rce.slave.interface.Interface

            @param remoteID:    Unique ID of the remote Interface.
            @type  remoteID:    uuid.UUID
        """
        assert remoteID in self._receivers
        receivers = self._receivers[remoteID]

        assert interface in receivers
        receivers.remove(interface)

        if not receivers:
            del self._receivers[remoteID]

    def remote_destroy(self):
        """ Method should be called to destroy the protocol and will take care
            of destroying all connections of this Protocol as well as
            deleting all circular references.
        """
        if self._receivers:
            for interface in reduce(set.union, self._receivers.itervalues()):
                interface.unregisterProtocol(self)

            self._receivers = None

        if self._endpoint:
            self._endpoint.unregisterProtocol(self)
            self._endpoint = None


class Loopback(_Protocol):
    """ Special Protocol 'Loopback' which can be used to connect Interfaces
        which are in the same Endpoint.
    """
    def sendMessage(self, interface, msg, msgID, remoteID=None):
        self.messageReceived(interface.UID, msg, msgID, remoteID)

    sendMessage.__doc__ = _Protocol.sendMessage.__doc__


class RCEInternalProtocol(Int32StringReceiver, _Protocol):
    """ Protocol which is used to connect Endpoints such that Interfaces in
        different Endpoint are able to communicate.
    """
    # CONFIG
    MAX_LENGTH = 30000000  # Maximal message length in bytes

    _MSG_ID_STRUCT = struct.Struct('!B')
    _TRUE = struct.pack('!?', True)
    _FALSE = struct.pack('!?', False)

    def __init__(self, endpoint):
        """ Initialize the Protocol.

            @param endpoint:    Endpoint for which this Protocol is created.
            @type  endpoint:    rce.slave.endpoint.Endpoint
        """
        _Protocol.__init__(self, endpoint)

        self._initialized = False
        self.stringReceived = self._initReceived

    def _initReceived(self, msg):
        """ Internally used method process a complete string message as long as
            the connection is not yet initialized.

            @param msg:         Message which was received.
            @type  msg:         str
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
        """ Internally used method process a complete string message after
            the connection has been initialized.

            @param msg:         Message which was received.
            @type  msg:         str
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

        remoteID = UUID(bytes=msg[offset:offset + 16])
        offset += 16

        idLen, = self._MSG_ID_STRUCT.unpack(msg[offset:offset + 1])
        offset += 1

        msgID = msg[offset:offset + idLen]
        offset += idLen

        self.messageReceived(remoteID, buffer(msg, offset), msgID, destID)

    def sendInit(self, connID, key):
        """ Send an init message to the other side.

            @param connID:      Unique ID which is used to identify the
                                connection.
            @type  connID:      str

            @param key:         Key which should be sent with the init message
                                to authenticate this endpoint.
            @type  key:         str
        """
        assert len(connID) == 16
        assert len(key) == 16

        self.sendString(connID + key)

    def sendMessage(self, interface, msg, msgID, remoteID=None):
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

    sendMessage.__doc__ = _Protocol.sendMessage.__doc__

    def connectionLost(self, reason):
        """ Method is called by the twisted framework when the connection is
            lost.
        """
        _Protocol.remote_destroy(self)

    def remote_destroy(self):
        """ Method should be called to destroy the connection and the protocol.
            It also takes care of any circular references.
        """
        self.transport.loseConnection()

    def lengthLimitExceeded(self, length):
        print('LENGTH LIMIT EXCEEDED {0}'.format(length))
        Int32StringReceiver.lengthLimitExceeded(self, length)
