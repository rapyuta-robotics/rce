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
import uuid
from datetime import datetime, timedelta

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
from twisted.internet.task import LoopingCall
from autobahn.websocket import WebSocketServerFactory, WebSocketServerProtocol

# Custom imports
from errors import InvalidRequest, AuthenticationError
from client import types
from client.interfaces import IClientMsgHandler
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
        msg = json.loads(msg)
        
        try:
            userID = msg['userID']
            robotID = msg['robotID']
        except KeyError as e:
            raise InvalidRequest('Error: Missing key: {0}'.format(e))
        
        response = self._manager.newConnection(userID, robotID)
        
        if not response:
            raise AuthenticationError('Error: Could not authenticate user.')
        
        self.sendMessage(json.dumps(response))
        self.dropConnection()
    
    def onMessage(self, msg, binary):
        """ Method is called by the Autobahn engine when a message has been '
            received from the client.
        """
        log.msg('WebSocket: Received new message from client. '
                '(binary={0})'.format(binary))
        
        if binary:
            self.sendMessage('Error: No binary messages allowed.')
            self.dropConnection()
            return
        
        try:
            self._handleMessage(msg)
        except InvalidRequest as e:
            self.sendMessage('Invalid Request: {0}'.format(e))
            self.dropConnection()
        except AuthenticationError as e:
            self.sendMessage('Authentication Error: {0}'.format(e))
            self.dropConnection()
        except Exception:   # TODO: Refine Error handling
            #import sys, traceback
            #etype, value, _ = sys.exc_info()
            #self.sendMessage('\n'.join(traceback.format_exception_only(etype,
            #                                                           value)))
            
            # Full debug message
            import traceback
            self.sendMessage(traceback.format_exc())
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
        
        # List of tuples (incompleteMsg, incompleteURIList, arrivalTime)
        self._incompleteMsgs = []
        
        # Setup repeated calling of the clean up method
        self._cleaner = LoopingCall(self._cleanUp)
        self._cleaner.start(definition.MSG_QUEUE_TIMEOUT/4)
    
    def _initConnection(self, msg):
        """ Internally used method to setup the necessary context to use this
            connection.
            
            @param msg:     Incoming message.
            @type  msg:     dict
        """
        if msg['type'] == types.INIT:
            data = msg['data']
            userID = str(data['userID'])
            robotID = str(data['robotID'])
            key = str(data['key'])
            
            self._manager.robotConnected(userID, robotID, key, self)
            self._robotID = robotID
            self._userID = userID
        else:
            raise InvalidRequest('First message has to be an INIT message.')
        
        # TODO: List should probably not be hard coded here,
        #       but given as an argument...
        for handler in [ CreateContainerHandler(self._manager, userID),
                         DestroyContainerHandler(self._manager, userID),
                         ConfigureContainerHandler(self._manager, userID),
                         ConnectInterfacesHandler(self._manager, userID),
                         DataMessageHandler(self._manager, userID) ]:
            verifyObject(IClientMsgHandler, handler)
            self._msgHandler[handler.TYPE] = handler
        
        WebSocketServerProtocol.sendMessage(self, 'Connection successfully '
                                                  'initialized.')
    
    def _recursiveURISearch(self, multidict):
        """ Internally used method to find binary data in incoming messages.
        """
        valueList = []
        
        for k,v in multidict.iteritems():
            if isinstance(v, dict):
                valueList += self._recursiveURISearch(v)
            elif k[-1] == '*':
                valueList.append((v, multidict, k))
        
        return valueList
    
    def _handleBinary(self, msg):
        """ Process a received binary message, i.e. store the binary data
            with the matching incomplete the message and forward the message
            if the received binary data completes the cached message.
            
            @param msg:     Received binary message.
            @type  msg:     str
        """
        uri = msg[:32]
        binaryData = StringIO()
        binaryData.write(msg[32:])
        
        for incompleteMsg in self._incompleteMsgs:
            msg, incompleteURIList, _ = incompleteMsg
            
            for incompleteUri in incompleteURIList:
                missingUri, msgDict, key = incompleteUri
                
                if uri == missingUri:
                    del msgDict[key]
                    msgDict[key[:-1]] = binaryData
                    incompleteURIList.remove(incompleteUri)
                    
                    if not incompleteURIList:
                        self._incompleteMsgs.remove(incompleteMsg)
                        self._processReceivedMessage(msg)
                    
                    return
        else:
            raise InvalidRequest('There was no message waiting for this '
                                 'binary message.')
    
    def _processReceivedMessage(self, msg):
        """ Internally used method to process complete messages by calling the
            appropriate handler.
        """
        try:
            self._msgHandler[msg['type']].handle(msg['data'])
        except KeyError:
            if msg['type'] == types.INIT:
                raise InvalidRequest('Connection is already initialized.')
            
            raise InvalidRequest('This message type is not supported.')
    
    def onMessage(self, msg, binary):
        """ Method is called by the Autobahn engine when a message has been '
            received from the client.
        """
        log.msg('WebSocket: Received new message from client. '
                '(binary={0})'.format(binary))
        
        try:
            if binary:
                if not (self._robotID and self._userID):
                    raise InvalidRequest('First message has to be an '
                                         'INIT message.')
                
                self._handleBinary(msg)
            else:
                msg = json.loads(msg)
                
                # Check if the connection has be initialized
                if not (self._robotID and self._userID):
                    self._initConnection(msg)
                    return
                
                # Check if there is a reference to a binary part
                uris = self._recursiveURISearch(msg)
                
                if uris:
                    # The message is incomplete, i.e. missing binary part
                    self._incompleteMsgs.append((msg, uris, datetime.now()))
                else:
                    self._processReceivedMessage(msg)
        except InvalidRequest as e:
            WebSocketServerProtocol.sendMessage(self, 'Invalid Request: '
                                                      '{0}'.format(e))
        except AuthenticationError as e:
            WebSocketServerProtocol.sendMessage(self, 'Authentication Error: '
                                                      '{0}'.format(e))
        except:   # TODO: Refine Error handling
            #import sys, traceback
            #etype, value, _ = sys.exc_info()
            #WebSocketServerProtocol.sendMessage(self, '\n'.join(
            #    traceback.format_exception_only(etype, value)))
            
            # Full debug message
            import traceback
            WebSocketServerProtocol.sendMessage(self, traceback.format_exc())
    
    def _recursiveBinarySearch(self, multidict):
        """ Internally used method to find binary data in outgoing messages.
        """
        uriBinary = []
        keys = []
        
        for k,v in multidict.iteritems():
            if isinstance(v, dict):
                uriBinaryPart, multidictPart = self._recursiveBinarySearch(v)
                uriBinary += uriBinaryPart
                multidict[k] = multidictPart 
            elif _checkIsStringIO(v):
                keys.append(k)
        
        for k in keys:
            tmpURI = uuid.uuid4().hex
            uriBinary.append((tmpURI, multidict[k]))
            del multidict[k]
            multidict['{0}*'.format(k)] = tmpURI
        
        return uriBinary, multidict
    
    def sendMessage(self, msg):
        """ This method is called by the User instance to send a message to the
            robot.
            
            @param msg:     Message which should be sent
        """
        URIBinary, msgURI = self._recursiveBinarySearch(msg)
        
        WebSocketServerProtocol.sendMessage(self, json.dumps(msgURI))
        
        for binData in URIBinary:
            WebSocketServerProtocol.sendMessage(self,
                binData[0] + binData[1].getvalue(), binary=True)
    
    def onClose(self, wasClean, code, reason):
        """ Method is called by the Autobahn engine when the connection has
            been lost.
        """
        if self._userID and self._robotID:
            self._manager.robotClosed(self._userID, self._robotID)
        
        self._cleaner.stop()
    
    def _cleanUp(self):
        """ Internally used method to remove old incomplete messages.
        """
        limit = datetime.now()-timedelta(seconds=definition.MSG_QUEUE_TIMEOUT)
        
        for i, incompleteMsg in enumerate(self._incompleteMsgs):
            if incompleteMsg[2] > limit:
                if i:
                    self._incompleteMsgs = self._incompleteMsgs[i:]
                    log.msg('{0} Incomplete message(s) have been dropped '
                            'from cache.'.format(i))
                
                break


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
