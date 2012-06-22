#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       UIDServer.py
#       
#       This file is part of the RoboEarth Cloud Engine framework.
#       
#       This file was originally created for RoboEearth
#       http://www.roboearth.org/
#       
#       The research leading to these results has received funding from
#       the European Union Seventh Framework Programme FP7/2007-2013 under
#       grant agreement no248942 RoboEarth.
#       
#       Copyright 2012 RoboEarth
#       
#       Licensed under the Apache License, Version 2.0 (the "License");
#       you may not use this file except in compliance with the License.
#       You may obtain a copy of the License at
#       
#       http://www.apache.org/licenses/LICENSE-2.0
#       
#       Unless required by applicable law or agreed to in writing, software
#       distributed under the License is distributed on an "AS IS" BASIS,
#       WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#       See the License for the specific language governing permissions and
#       limitations under the License.
#       
#       \author/s: Dominique Hunziker 
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
