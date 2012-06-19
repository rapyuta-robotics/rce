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

# zope specific imports
from zope.interface.verify import verifyObject
from zope.interface.exceptions import Invalid

# twisted specific imports
from twisted.internet.protocol import ServerFactory
from twisted.protocols.basic import LineReceiver

# Custom imports
import settings
from Exceptions import InternalError
from Interfaces import IUIDServer

class UIDServerProtocol(LineReceiver):
    """ Protocol which is used by the server to send a new UID for a machine.
    """
    def __init__(self, uidServer):
        """ Initialize the UIDServerProtocol.
            
            @param uidServer:   Instance which is used to provide the new UIDs.
            @type  uidServer:   IUIDServer
        """
        self._uidServer = uidServer
        self._buff = []
    
    def connectionMade(self):
        """ Callback which is called by twisted when connection is established.
        """
        self.transport.write('{0}{1}'.format(self._uidServer.getUID(), self.delimiter))
        
        if not settings.USE_SSL:
            self.transport.loseConnection()
    
    def lineReceived(self, line):
        """ Callback which is called by twisted when a line has been received.
        """
        if settings.USE_SSL:
            if not line:
                self.transport.write(
                    '{0}{1}'.format(
                        self._uidServer.getCertificate(
                            '\n'.join(self._buff),
                        ),
                        self.delimiter
                    )
                )
                self.transport.loseConnection()
            else:
                self._buff.append(line)

class UIDServerFactory(ServerFactory):
    """ Factory which is used on the server side to provide clients with a UID.
    """
    def __init__(self, uidServer):
        """ Initialize the UIDServerFactory.
            
            @param uidServer:   Instance which is used to provide the new UIDs.
            @type  uidServer:   IUIDServer
        """
        try:
            verifyObject(IUIDServer, uidServer)
        except Invalid as e:
            raise InternalError(
                'Verification of the class "{0}" for the Interface "IUIDServer" failed: {1}'.format(
                    uidServer.__class__.__name__,
                    e
                )
            )
        
        self._uidServer = uidServer
    
    def buildProtocol(self, addr):
        """ Callback which is called by twisted when a new connection should be established.
        """
        return UIDServerProtocol(self._uidServer)
