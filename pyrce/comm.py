#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     comm.py
#     
#     This file is part of the RoboEarth Cloud Engine pyrce client.
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

# twisted specific imports
from autobahn.websocket import WebSocketClientFactory, \
    WebSocketClientProtocol

# Custom package imports
import sys                          ### TODO:
sys.path.append('../framework')     ### TEMPORARY FIX

from rce.client.assembler import MessageAssembler


class RCERobotProtocol(WebSocketClientProtocol):
    """ WebSocket client protocol which is used to communicate with the Robot
        Manager.
    """
    def __init__(self, connection):
        """ Initialize the protocol.
            
            @param connection:      Connection instance which provides callback
                                    functions.
            @type  connection:      pyrce.connection._Connection
        """
        self._connection = connection
        self._assembler = MessageAssembler(self, 60)
        self._registered = False
    
    def onOpen(self):
        """ This method is called by twisted as soon as the websocket
            connection has been successfully established.
        """
        self._assembler.start()
        self._connection.registerConnection(self)
        self._registered = True
    
    def onMessage(self, msg, binary):
        """ This method is called by twisted when a new message has been
            received.
        """
        self._assembler.processMessage(msg, binary)
    
    def processCompleteMessage(self, msg):
        """ Callback for MessageAssembler which will be called as soon as a
            message has been completed and is ready for processing.
        """
        self._connection.receivedMessage(msg)
    
    def onClose(self, *a):
        """ This method is called by twisted when the connection has been
            closed.
        """
        if self._registered:
            self._connection.unregisterConnection(self)
            self._assembler.stop()
            self._registered = False
    
    def failHandshake(self, reason):
        """ This method is called by twisted when the connection could not be
            initialized.
        """
        print(reason)
        WebSocketClientProtocol.failHandshake(self, reason)


class RCERobotFactory(WebSocketClientFactory):
    """ WebSocket protocol factory which is used for the communication with the
        Robot Manager.
    """
    def __init__(self, url, connection):
        """ Initialize the factory.
            
            @param url:             URL of the Robot Manager.
            @type  url:             str
            
            @param connection:      Connection instance which provides callback
                                    functions.
            @type  connection:      pyrce.connection._Connection
        """
        WebSocketClientFactory.__init__(self, url)
        self._connection = connection
    
    def buildProtocol(self, addr):
        """ This method is called by twisted when a new connection should be
            made.
        """
        p = RCERobotProtocol(self._connection)
        p.factory = self
        return p
