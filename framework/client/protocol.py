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
import json

try:
    from cStringIO import StringIO, InputType, OutputType
    from StringIO import StringIO as pyStringIO
    
    def _checkIsStringIO(obj):
        return isinstance(obj, (InputType, OutputType, pyStringIO))
except ImportError:
    from StringIO import StringIO
    
    def _checkIsStringIO(obj):
        return isinstance(obj, StringIO)

# twisted specific imports
from twisted.python import log
from autobahn.websocket import WebSocketServerFactory, WebSocketServerProtocol

# Custom imports
from errors import InvalidRequest, AuthenticationError
from client import types
from client.interfaces import IClientMsgHandler
from client.assembler import recursiveBinarySearch, MessageAssembler
from client.handler import CreateContainerHandler, DestroyContainerHandler, \
    ConfigureContainerHandler, ConnectInterfacesHandler, DataMessageHandler
from util.interfaces import verifyObject
from comm import definition


class MasterWebSocketProtocol(WebSocketServerProtocol):
    """ Protocol which is used for the connections from the robots to the
        master manager.
    """
    def __init__(self, manager):
        """ Initialize the Protocol.
        """
        self._manager = manager
    
    def _handleMessage(self, msg):
        """ Internally used method to handle a message.
        """
        try:
            msg = json.loads(msg)
        except ValueError:
            raise InvalidRequest('Message is not in valid JSON format.')
        
        try:
            msgType = msg['type']
            data = msg['data']
        except KeyError as e:
            raise InvalidRequest('Message is missing key: {0}'.format(e))
        
        if msgType != types.INIT:
            raise InvalidRequest('Message type is not INIT.')
        
        try:
            userID = data['userID']
            robotID = data['robotID']
        except KeyError as e:
            raise InvalidRequest('Data is missing key: {0}'.format(e))
        
        response = self._manager.newConnection(userID, robotID)
        
        if not response:
            raise AuthenticationError('Could not authenticate user.')
        
        key, ip = response
        return {'key' : key, 'url' : 'ws://{0}:9010/'.format(ip)}
    
    def onMessage(self, msg, binary):
        """ Method is called by the Autobahn engine when a message has been '
            received from the client.
        """
        log.msg('WebSocket: Received new message from client. '
                '(binary={0})'.format(binary))
        
        if binary:
            resp = 'Error: No binary messages allowed.'
            msgType = types.ERROR
        else:
            try:
                resp = self._handleMessage(msg)
                msgType = types.INIT
            except InvalidRequest as e:
                resp = 'Invalid Request: {0}'.format(e)
                msgType = types.ERROR
            except AuthenticationError as e:
                resp = 'Authentication Error: {0}'.format(e)
                msgType = types.ERROR
            except Exception as e:   # TODO: Refine Error handling
                #import sys, traceback
                #resp = '\n'.join(traceback.format_exception_only(type(e), e)))
                
                # Full debug message
                import traceback
                resp = traceback.format_exc()
                msgType = types.ERROR
        
        self.sendMessage(json.dumps({'data' : resp, 'type' : msgType}))
        self.dropConnection()


class RobotWebSocketProtocol(WebSocketServerProtocol):
    """ Protocol which is used for the connections from the robots to the
        robot manager.
    """
    def __init__(self, manager):
        """ Initialize the Protocol.
        """
        self._manager = manager
        self._userID = None
        self._robotID = None
        self._msgHandler = {}
        self._assembler = MessageAssembler(self, definition.MSG_QUEUE_TIMEOUT)
    
    def _initConnection(self, msg):
        """ Internally used method to setup the necessary context to use this
            connection.
            
            @param msg:     Incoming message.
            @type  msg:     dict
        """
        try:
            msgType = msg['type']
            data = msg['data']
        except KeyError as e:
            raise InvalidRequest('Message is missing key: {0}'.format(e))
            
        if msgType != types.INIT:
            raise InvalidRequest('First message has to be an INIT message.')
        
        try:
            userID = str(data['userID'])
            robotID = str(data['robotID'])
            key = str(data['key'])
        except KeyError as e:
            raise InvalidRequest('INIT Message is missing key: {0}'.format(e))
        
        self._manager.robotConnected(userID, robotID, key, self)
        self._robotID = robotID
        self._userID = userID
        
        # TODO: List should probably not be hard coded here,
        #       but given as an argument...
        for handler in [ CreateContainerHandler(self._manager, userID),
                         DestroyContainerHandler(self._manager, userID),
                         ConfigureContainerHandler(self._manager, userID),
                         ConnectInterfacesHandler(self._manager, userID),
                         DataMessageHandler(self._manager, userID) ]:
            verifyObject(IClientMsgHandler, handler)
            self._msgHandler[handler.TYPE] = handler
        
        self._assembler.start()
        return 'Connection successfully initialized.'
    
    def processCompleteMessage(self, msg):
        """ Process complete messages by calling the appropriate handler for
            the manager. (Called by client.protocol.BinaryAssembler)
        """
        try:
            msgType = msg['type']
            data = msg['data']
        except KeyError as e:
            raise InvalidRequest('Message is missing key: {0}'.format(e))
        
        try:
            self._msgHandler[msgType].handle(data)
        except KeyError as e:
            if msgType == types.INIT:
                raise InvalidRequest('Connection is already initialized.')
            
            raise InvalidRequest('This message type is not supported.')
    
    def onMessage(self, msg, binary):
        """ Method is called by the Autobahn engine when a message has been '
            received from the client.
        """
        log.msg('WebSocket: Received new message from client. '
                '(binary={0})'.format(binary))
        
        try:
            if not self._robotID or not self._userID:
                if binary:
                    raise InvalidRequest('First message has to be an '
                                         'INIT message.')
                
                resp = self._initConnection(json.loads(msg))
                msgType = types.STATUS
            else:
                self._assembler.processMessage(msg, binary)
                resp = None
        except InvalidRequest as e:
            #resp = 'Invalid Request: {0}'.format(e)
            import traceback
            resp = traceback.format_exc()
            msgType = types.ERROR
        except AuthenticationError as e:
            #resp =  'Authentication Error: {0}'.format(e)
            import traceback
            resp = traceback.format_exc()
            msgType = types.ERROR
        except Exception as e:   # TODO: Refine Error handling
            #import sys, traceback
            #WebSocketServerProtocol.sendMessage(self, '\n'.join(
            #    traceback.format_exception_only(type(e), e)))
            
            # Full debug message
            import traceback
            resp = traceback.format_exc()
            msgType = types.ERROR
        
        if resp:
            self.sendMessage({'data' : resp, 'type' : msgType})
    
    def sendMessage(self, msg):
        """ This method is called by the User instance to send a message to the
            robot.
            
            @param msg:     Message which should be sent
        """
        uriBinary, msgURI = recursiveBinarySearch(msg)
        
        WebSocketServerProtocol.sendMessage(self, json.dumps(msgURI))
        
        for binData in uriBinary:
            WebSocketServerProtocol.sendMessage(self,
                binData[0] + binData[1].getvalue(), binary=True)
    
    def onClose(self, wasClean, code, reason):
        """ Method is called by the Autobahn engine when the connection has
            been lost.
        """
        if self._userID and self._robotID:
            self._manager.robotClosed(self._userID, self._robotID)
        
        self._assembler.stop()
        self._assembler = None


class CloudEngineWebSocketFactory(WebSocketServerFactory):
    """ Factory which is used for the connections from the robots to the
        RoboEarth Cloud Engine.
    """
    def __init__(self, protocolCls, manager, url):
        """ Initialize the Factory.
        """
        WebSocketServerFactory.__init__(self, url)
        
        self._protocolCls = protocolCls
        self._manager = manager
    
    def buildProtocol(self, addr):
        """ Method is called by the twisted reactor when a new connection
            attempt is made.
        """
        p = self._protocolCls(self._manager)
        p.factory = self
        return p
