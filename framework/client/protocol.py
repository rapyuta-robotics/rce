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
from twisted.web.resource import Resource
from autobahn import httpstatus
from autobahn.websocket import HttpException, \
    WebSocketServerFactory, WebSocketServerProtocol

# Custom imports
from errors import InvalidRequest, InternalError, AuthenticationError
from client import types
from client.interfaces import IClientMsgHandler
from client.assembler import recursiveBinarySearch, MessageAssembler
from client.handler import CreateContainerHandler, DestroyContainerHandler, \
    ConfigureContainerHandler, ConnectInterfacesHandler, DataMessageHandler
from util.interfaces import verifyObject
from comm import definition


_MIN_VERSION = '20121031'  # Minimal required version of the client
_CUR_VERSION = '20121113'  # Current version of the client


class MasterRobotAuthentication(Resource):
    """ Twisted web.Resource which is used in the Master to authenticate new
        robots connections.
    """
    isLeaf = True
    
    def __init__(self, handler):
        """ Initialize the authentication resource.
            
            @param handler:     Handler which is used to process the requests.
            @type  handler:     client.handler.AuthenticationHandler
        """
        self._handler = handler
    
    def _processGET(self, args):
        """ Internally used method to process a GET request.
        """
        try:
            try:
                version = args['version']
            except KeyError:
                raise InvalidRequest('Request is missing parameter: version')
            
            if len(version) != 1:
                raise InvalidRequest("Parameter 'version' has to be unique in "
                                     'request.')
            
            version = version[0]
            
            if version < _MIN_VERSION:
                return (httpstatus.HTTP_STATUS_CODE_GONE[0],
                        'text/plain; charset=utf-8',
                        'Client version is out-dated. Minimal version is '
                        "'{0}'.".format(_MIN_VERSION))
            
            msg = self._handler.handle(args)
        
            if version != _CUR_VERSION:
                msg['current'] = _CUR_VERSION
        except InvalidRequest as e:
            return (httpstatus.HTTP_STATUS_CODE_BAD_REQUEST[0],
                    'text/plain; charset=utf-8', str(e))
        except AuthenticationError as e:
            return (httpstatus.HTTP_STATUS_CODE_FORBIDDEN[0],
                    'text/plain; charset=utf-8', str(e))
        except Exception as e:
            return (httpstatus.HTTP_STATUS_CODE_INTERNAL_SERVER_ERROR[0],
                    'text/plain; charset=utf-8', str(e))
        
        return (httpstatus.HTTP_STATUS_CODE_OK[0],
                'application/json; charset=utf-8', json.dumps(msg))
    
    def render_GET(self, request):
        """ This method is called by the twisted framework when a GET request
            was received.
        """
        response = self._processGET(request.args)
        
        request.setResponseCode(response[0])
        request.setHeader('content-type', response[1])
        return response[2]


#class MasterWebSocketProtocol(WebSocketServerProtocol):
#    """ Protocol which is used for the connections from the robots to the
#        master manager.
#    """
#    def __init__(self, manager):
#        """ Initialize the Protocol.
#        """
#        self._manager = manager
#    
#    def onConnect(self, req):
#        """ Method is called by the Autobahn engine when a request to establish
#            a connection has been received.
#            
#            @param req:     Connection Request object.
#            @type  req:     autobahn.websocket.ConnectionRequest
#            
#            @raise:         autobahn.websocket.HttpException
#        """
#        params = req.params
#        
#        try:
#            userID = params['userID']
#            robotID = params['robotID']
#        except KeyError as e:
#            raise HttpException(httpstatus.HTTP_STATUS_CODE_BAD_REQUEST[0],
#                                'Request is missing parameter: {0}'.format(e))
#        
#        if len(userID) != 1:
#            raise HttpException(httpstatus.HTTP_STATUS_CODE_BAD_REQUEST[0],
#                                "Parameter 'userID' has to be unique in "
#                                'request.')
#        else:
#            userID = userID[0]
#        
#        if len(robotID) != 1:
#            raise HttpException(httpstatus.HTTP_STATUS_CODE_BAD_REQUEST[0],
#                                "Parameter 'robotID' has to be unique in "
#                                'request.')
#        else:
#            robotID = robotID[0]
#        
#        try:
#            response = self._manager.newConnection(userID, robotID)
#        except InternalError as e:
#            raise HttpException(
#                httpstatus.HTTP_STATUS_CODE_INTERNAL_SERVER_ERROR[0], str(e)
#            )
#        except AuthenticationError as e:
#            raise HttpException(httpstatus.HTTP_STATUS_CODE_FORBIDDEN[0],
#                                str(e))
#            
#        if not response:
#            raise HttpException(httpstatus.HTTP_STATUS_CODE_FORBIDDEN[0],
#                                'Could not authenticate user.')
#        
#        key, ip = response
#        
#        return {'key' : key, 'url' : 'ws://{0}:9010/'.format(ip)}
#    
#    def onMessage(self, msg, binary):
#        """ Method is called by the Autobahn engine when a message has been '
#            received from the client.
#        """
#        log.msg('WebSocket: Received new message from client. '
#                '(binary={0})'.format(binary))
#        
#        if binary:
#            resp = 'Error: No binary messages allowed.'
#            msgType = types.ERROR
#        else:
#            try:
#                resp = self._handleMessage(msg)
#                msgType = types.INIT
#            except InvalidRequest as e:
#                resp = 'Invalid Request: {0}'.format(e)
#                msgType = types.ERROR
#            except AuthenticationError as e:
#                resp = 'Authentication Error: {0}'.format(e)
#                msgType = types.ERROR
#            except Exception as e:   # TODO: Refine Error handling
#                #import sys, traceback
#                #resp = '\n'.join(traceback.format_exception_only(type(e), e)))
#                
#                # Full debug message
#                import traceback
#                resp = traceback.format_exc()
#                msgType = types.ERROR
#        
#        self.sendMessage(json.dumps({'data' : resp, 'type' : msgType}))
#        self.dropConnection()


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
    
    def onConnect(self, req):
        """ Method is called by the Autobahn engine when a request to establish
            a connection has been received.
            
            @param req:     Connection Request object.
            @type  req:     autobahn.websocket.ConnectionRequest
            
            @raise:         autobahn.websocket.HttpException
        """
        params = req.params
        
        try:
            userID = params['userID']
            robotID = params['robotID']
            key = params['key']
        except KeyError as e:
            raise HttpException(httpstatus.HTTP_STATUS_CODE_BAD_REQUEST[0],
                                'Request is missing parameter: {0}'.format(e))
        
        for name, param in [('userID', userID),
                            ('robotID', robotID),
                            ('key', key)]:
            if len(param) != 1:
                raise HttpException(httpstatus.HTTP_STATUS_CODE_BAD_REQUEST[0],
                                    "Parameter '{0}' has to be unique in "
                                    'request.'.format(name))
        
        userID = userID[0]
        robotID = robotID[0]
        key = key[0]
        
        try:
            self._manager.robotConnected(userID, robotID, key, self)
        except AuthenticationError as e:
            raise HttpException(httpstatus.HTTP_STATUS_CODE_FORBIDDEN[0],
                                str(e))
        except Exception as e:
            import traceback
            raise HttpException(
                httpstatus.HTTP_STATUS_CODE_INTERNAL_SERVER_ERROR[0],
                traceback.format_exc()
            )
            
        self._robotID = robotID
        self._userID = userID
        uid = (userID, robotID)
        
        # TODO: List should probably not be hard coded here,
        #       but given as an argument...
        for handler in [ CreateContainerHandler(self._manager, uid),
                         DestroyContainerHandler(self._manager, uid),
                         ConfigureContainerHandler(self._manager, uid),
                         ConnectInterfacesHandler(self._manager, uid),
                         DataMessageHandler(self._manager, uid) ]:
            verifyObject(IClientMsgHandler, handler)
            self._msgHandler[handler.TYPE] = handler
        
        self._assembler.start()
        return None
    
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
            raise InvalidRequest('This message type is not supported.')
    
    def onMessage(self, msg, binary):
        """ Method is called by the Autobahn engine when a message has been
            received from the client.
        """
        log.msg('WebSocket: Received new message from client. '
                '(binary={0})'.format(binary))
        
        resp = None
        
        try:
            self._assembler.processMessage(msg, binary)
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
