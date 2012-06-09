#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       UIDClient.py
#       
#       Copyright 2012 dominique hunziker <dominique.hunziker@gmail.com>
#       
#       This program is free software; you can redistribute it and/or modify
#       it under the terms of the GNU General Public License as published by
#       the Free Software Foundation; either version 2 of the License, or
#       (at your option) any later version.
#       
#       This program is distributed in the hope that it will be useful,
#       but WITHOUT ANY WARRANTY; without even the implied warranty of
#       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#       GNU General Public License for more details.
#       
#       You should have received a copy of the GNU General Public License
#       along with this program; if not, write to the Free Software
#       Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#       MA 02110-1301, USA.
#       
#       

# twisted specific imports
from twisted.internet.error import ConnectionDone
from twisted.internet.protocol import Protocol, ClientFactory

class UIDClientProtocol(Protocol):
    """ Protocol which is used by a client to retrieve a new UID for a machine.
    """
    def __init__(self, deferred):
        """ Initialize UIDClientProtocol.
            
            @param deferred:    Deferred which should be called with the received UID.
            @type  deferred:    Deferred
        """
        self._deferred = deferred
        self._buff = ''
    
    def dataReceived(self, data):
        """ Callback which is called by twisted when new data arrived.
        """
        self._buff += data
    
    def connectionLost(self, reason):
        """Callback which is called by twisted when the connection is lost.
        """
        if reason.check(ConnectionDone):
            self._deferred.callback(self._buff)
        else:
            self._deferred.errback(reason.getErrorMessage())

class UIDClientFactory(ClientFactory):
    """ Factory which is used by a client to retrieve a new UID for a machine.
    """
    def __init__(self, deferred):
        """ Initialize UIDClientFactory.
            
            @param deferred:    Deferred which should be called with the reveived UID.
            @type  deferred:    Deferred
        """
        self._deferred = deferred
    
    def buildProtocol(self, addr):
        """ Callback which is called by twisted when a new connection should be established.
        """
        return UIDClientProtocol(self._deferred)
