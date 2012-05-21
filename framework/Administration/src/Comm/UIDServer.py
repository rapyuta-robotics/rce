#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       UIDServer.py
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
from zope.interface.verify import verifyObject
from twisted.internet.protocol import Protocol, ServerFactory

# Custom imports
from Exceptions import InternalError
from Interfaces import IUIDServer #@UnresolvedImport

class UIDServerProtocol(Protocol):
    """ Protocol which is used by the server to send a new UID for a machine.
    """
    def __init__(self, uidServer):
        """ Initialize the UIDServerProtocol.
            
            @param uidServer:   Instance which is used to provide the new UIDs.
            @type  uidServer:   IUIDServer
        """
        self._uidServer = uidServer
    
    def connectionMade(self):
        """Callback which is called by twisted when the connection has been established.
        """
        uid = self._uidServer.getUID()
        self.transport.write(uid)
        self.transport.loseConnection()

class UIDServerFactory(ServerFactory):
    """ Factory which is used on the server side to provide clients with a UID.
    """
    def __init__(self, uidServer):
        """ Initialize the UIDServerFactory.
            
            @param uidServer:   Instance which is used to provide the new UIDs.
            @type  uidServer:   IUIDServer
        """
        if not verifyObject(IUIDServer, uidServer):
            raise InternalError('UIDServer does not implement the interface "IUIDServer".')
        
        self._uidServer = uidServer
    
    def buildProtocol(self, addr):
        """ Callback which is called by twisted when a new connection should be established.
        """
        return UIDServerProtocol(self._uidServer)
