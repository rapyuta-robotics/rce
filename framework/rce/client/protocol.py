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
    from cStringIO import StringIO
except ImportError:
    from StringIO import StringIO #@UnusedImport

# twisted specific imports
from twisted.python import log
from twisted.python.failure import Failure
from twisted.internet.defer import fail
from twisted.cred.error import UnauthorizedLogin
from twisted.web.resource import Resource
from twisted.web.server import NOT_DONE_YET

# Autobahn specific imports
from autobahn import httpstatus
from autobahn.websocket import HttpException, \
    WebSocketServerFactory, WebSocketServerProtocol

# Custom imports
from rce.error import InvalidRequest, InternalError, DeadConnection
from rce.robot import RobotFacade
from rce.client import types
from rce.client.interfaces import IRobot
from rce.client.assembler import recursiveBinarySearch, MessageAssembler
from rce.client.cred import RobotCredentials
import settings


_MIN_VERSION = '20130210'  # Minimal required version of the client
_CUR_VERSION = '20130210'  # Current version of the client


class _VersionError(Exception):
    """ Error is raised during Master-Robot authentication in case the used
        client version is insufficient.
    """


class MasterRobotAuthentication(Resource):
    """ Twisted web.Resource which is used in the Master to authenticate new
        robots connections.
    """
    isLeaf = True
    
    def __init__(self, realm):
        """ Initialize the authentication resource.
            
            @param realm:       Realm of the cloud engine which provides the
                                method to login a new user/robot.
            @type  realm:       rce.master.core.RoboEarthCloudEngine
        """
        self._realm = realm
    
    def _processGETReq(self, args):
        """ Internally used method to process a GET request.
        """
        # First check if the version is ok
        try:
            version = args['version']
        except KeyError as e:
            return fail(InvalidRequest('Request is missing parameter: '
                                       '{0}'.format(e)))
        
        if len(version) != 1:
            return fail(InvalidRequest("Parameter 'version' has to be unique "
                                       'in request.'))
        
        version = version[0]
        
        if version < _MIN_VERSION:
            return fail(_VersionError('Client version is insufficient. '
                                      'Minimal version is '
                                      "'{0}'.".format(_MIN_VERSION)))
        
        # Version is ok, now the GET request can be processed
        try:
            userID = args['userID']
        except KeyError as e:
            return fail(InvalidRequest('Request is missing parameter: '
                                       '{0}'.format(e)))
        
        for name, param in [('userID', userID)]:
            if len(param) != 1:
                return fail(InvalidRequest("Parameter '{0}' has to be unique "
                                           'in request.'.format(name)))
        
        d = self._realm.requestURL(userID[0])
        d.addCallback(lambda result: (result, version))
        return d
    
    def _processGETResp(self, (addr, version), request):
        """ Internally used method to process a response to a GET request from
            the realm.
        """
        msg = {'url' : 'ws://{0}/'.format(addr)}
        
        if version != _CUR_VERSION:
            msg['current'] = _CUR_VERSION
        
        self._render_GET(request, httpstatus.HTTP_STATUS_CODE_OK[0],
                         'application/json; charset=utf-8', json.dumps(msg))
    
    def _processGETErr(self, e, request):
        """ Internally used method to process an error to a GET request from
            the realm.
        """
        if e.check(InvalidRequest):
            msg = e.getErrorMessage()
            code = httpstatus.HTTP_STATUS_CODE_BAD_REQUEST[0]
        elif e.check(UnauthorizedLogin):
            msg = e.getErrorMessage()
            code = httpstatus.HTTP_STATUS_CODE_UNAUTHORIZED[0]
        elif e.check(_VersionError):
            msg = e.getErrorMessage()
            code = httpstatus.HTTP_STATUS_CODE_GONE[0]
        elif e.check(InternalError):
            e.printTraceback()
            msg = 'Internal Error'
            code = httpstatus.HTTP_STATUS_CODE_INTERNAL_SERVER_ERROR[0]
        else:
            e.printTraceback()
            msg = 'Fatal Error'
            code = httpstatus.HTTP_STATUS_CODE_INTERNAL_SERVER_ERROR[0]
        
        self._render_GET(request, code, 'text/plain; charset=utf-8', msg)
    
    def _render_GET(self, request, code, ctype, msg):
        """ Internally used method to render the response to a GET request.
        """
        request.setResponseCode(code)
        request.setHeader('content-type', ctype)
        request.write(msg)
        request.finish()
    
    def render_GET(self, request):
        """ This method is called by the twisted framework when a GET request
            was received.
        """
        d = self._processGETReq(request.args)
        d.addCallback(self._processGETResp, request)
        d.addErrback(self._processGETErr, request)
        
        return NOT_DONE_YET


class RobotWebSocketProtocol(WebSocketServerProtocol):
    """ Protocol which is used for the connections from the robots to the
        robot manager.
    """
    # CONFIG
    MSG_QUEUE_TIMEOUT = 60
    
    def __init__(self, portal, reactor, masterIP):
        """ Initialize the Protocol.
            
            @param portal:      Portal which is responsible for the
                                authentication of the websocket connection and
                                which will provide an Avatar for the robot in
                                the cloud engine.
            @type  portal:      twisted.cred.portal.Portal
        """
        self._portal = portal
        self._reactor = reactor
        self._assembler = MessageAssembler(self, self.MSG_QUEUE_TIMEOUT)
        self._avatar = None
        self._logout = None
        self._masterIP = masterIP
    
    def onConnect(self, req):
        """ Method is called by the Autobahn engine when a request to establish
            a connection has been received.
            
            @param req:     Connection Request object.
            @type  req:     autobahn.websocket.ConnectionRequest
            
            @return:        Deferred which fires callback with None or errback
                            with autobahn.websocket.HttpException
        """
        params = req.params
        
        try:
            userID = params['userID']
            robotID = params['robotID']
            password = params['password']
        except KeyError as e:
            raise HttpException(httpstatus.HTTP_STATUS_CODE_BAD_REQUEST[0],
                                'Request is missing parameter: {0}'.format(e))
        
        for name, param in [('userID', userID), ('robotID', robotID),
                            ('password', password)]:
            if len(param) != 1:
                raise HttpException(httpstatus.HTTP_STATUS_CODE_BAD_REQUEST[0],
                                    "Parameter '{0}' has to be unique in "
                                    'request.'.format(name))
        
        deferred = self._portal.requestAvatar(userID[0], robotID[0], 
                                              password[0], self, self._masterIP,
                                              settings.RCE_CONSOLE_PORT)

        deferred.addCallback(self._authenticate_success)
        deferred.addErrback(self._authenticate_failed)
        return deferred
    
    def _authenticate_success(self, (interface, avatar, logout)):
        """ Method is called by deferred when the connection has been
            successfully authenticated while being in 'onConnect'.
        """
        if interface != IRobot:
            return Failure(InternalError("Avatar has to implement 'IRobot'."))
        
        assert callable(logout)
        
        avatar.registerConnectionToRobot(self)
        self._avatar = avatar
        self._logout = logout
        self._assembler.start()
    
    def _authenticate_failed(self, e):
        """ Method is called by deferred when the connection could not been
            authenticated while being in 'onConnect'.
        """
        if e.check(InvalidRequest):
            code = httpstatus.HTTP_STATUS_CODE_BAD_REQUEST[0]
            msg = e.getErrorMessage()
        elif e.check(UnauthorizedLogin):
            code = httpstatus.HTTP_STATUS_CODE_UNAUTHORIZED[0]
            msg = httpstatus.HTTP_STATUS_CODE_UNAUTHORIZED[1]
        elif e.check(InternalError):
            e.printTraceback()
            
            code = httpstatus.HTTP_STATUS_CODE_INTERNAL_SERVER_ERROR[0]
            msg = 'Internal Error'
        else:
            e.printTraceback()
            
            code = httpstatus.HTTP_STATUS_CODE_INTERNAL_SERVER_ERROR[0]
            msg = 'Fatal Error'
        
        return Failure(HttpException(code, msg))
    
    def processCompleteMessage(self, msg):
        """ Process complete messages by calling the appropriate handler for
            the manager. (Called by client.protocol.MessageAssembler)
        """
        try:
            msgType = msg['type']
            data = msg['data']
        except KeyError as e:
            raise InvalidRequest('Message is missing key: {0}'.format(e))
        
        if msgType == types.DATA_MESSAGE:
            self._process_DataMessage(data)
        elif msgType == types.CONFIGURE_COMPONENT:
            self._process_configureComponent(data)
        elif msgType == types.CONFIGURE_CONNECTION:
            self._process_configureConnection(data)
        elif msgType == types.CREATE_CONTAINER:
            self._process_createContainer(data)
        elif msgType == types.DESTROY_CONTAINER:
            self._process_destroyContainer(data)
        else:
            raise InvalidRequest('This message type is not supported.')
    
    def _process_createContainer(self, data):
        """ Internally used method to process a request to create a container.
        """
        try:
            self._viewProxy.createContainer(data['containerTag'])
        except KeyError as e:
            raise InvalidRequest("Can not process 'CreateContainer' request. "
                                 'Missing key: {0}'.format(e))
    
    def _process_destroyContainer(self, data):
        """ Internally used method to process a request to destroy a container.
        """
        try:
            self._viewProxy.destroyContainer(data['containerTag'])
        except KeyError as e:
            raise InvalidRequest("Can not process 'DestroyContainer' request. "
                                 'Missing key: {0}'.format(e))
    
    def _process_configureComponent(self, data):
        """ Internally used method to process a request to configure
            components.
        """
        for node in data.pop('addNodes', []):
            try:
                self._viewProxy.addNode(node['containerTag'],
                                     node['nodeTag'],
                                     node['pkg'],
                                     node['exe'],
                                     node.get('args', ''),
                                     node.get('name', ''),
                                     node.get('namespace', ''))
            except KeyError as e:
                raise InvalidRequest("Can not process 'ConfigureComponent' "
                                     "request. 'addNodes' is missing key: "
                                     '{0}'.format(e))
        
        for node in data.pop('removeNodes', []):
            try:
                self._viewProxy.removeNode(node['containerTag'],
                                        node['nodeTag'])
            except KeyError as e:
                raise InvalidRequest("Can not process 'ConfigureComponent' "
                                     "request. 'removeNodes' is missing key: "
                                     '{0}'.format(e))
        
        for conf in data.pop('addInterfaces', []):
            try:
                self._viewProxy.addInterface(conf['endpointTag'],
                                          conf['interfaceTag'],
                                          conf['interfaceType'],
                                          conf['className'],
                                          conf.get('addr', ''))
            except KeyError as e:
                raise InvalidRequest("Can not process 'ConfigureComponent' "
                                     "request. 'addInterfaces' is missing "
                                     'key: {0}'.format(e))
        
        for conf in data.pop('removeInterfaces', []):
            try:
                self._viewProxy.removeInterface(conf['endpointTag'],
                                             conf['interfaceTag'])
            except KeyError as e:
                raise InvalidRequest("Can not process 'ConfigureComponent' "
                                     "request. 'removeInterfaces' is missing "
                                     'key: {0}'.format(e))
        
        for param in data.pop('setParam', []):
            try:
                self._viewProxy.addParameter(param['containerTag'],
                                          param['name'],
                                          param['value'])
            except KeyError as e:
                raise InvalidRequest("Can not process 'ConfigureComponent' "
                                     "request. 'setParam' is missing key: "
                                     '{0}'.format(e))
        
        for param in data.pop('deleteParam', []):
            try:
                self._viewProxy.removeParameter(param['containerTag'],
                                             param['name'])
            except KeyError as e:
                raise InvalidRequest("Can not process 'ConfigureComponent' "
                                     "request. 'deleteParam' is missing key: "
                                     '{0}'.format(e))
    
    def _process_configureConnection(self, data):
        """ Internally used method to process a request to configure
            connections.
        """
        for conf in data.pop('connect', []):
            try:
                self._viewProxy.addConnection(conf['tagA'], conf['tagB'])
            except KeyError as e:
                raise InvalidRequest("Can not process 'ConfigureComponent' "
                                     "request. 'connect' is missing key: "
                                     '{0}'.format(e))
        
        for conf in data.pop('disconnect', []):
            try:
                self._viewProxy.removeConnection(conf['tagA'], conf['tagB'])
            except KeyError as e:
                raise InvalidRequest("Can not process 'ConfigureComponent' "
                                     "request. 'disconnect' is missing key: "
                                     '{0}'.format(e))
    
    def _process_DataMessage(self, data):
        """ Internally used method to process a data message.
        """
        try:
            iTag = str(data['iTag'])
            mType = str(data['type'])
            msgID = str(data['msgID'])
            msg = data['msg']
        except KeyError as e:
            raise InvalidRequest("Can not process 'DataMessage' request. "
                                 'Missing key: {0}'.format(e))
        
        if len(msgID) > 255:
            raise InvalidRequest("Can not process 'DataMessage' request. "
                                 'Message ID can not be longer than 255.')
        
        self._avatar.receivedFromClient(iTag, mType, msgID, msg)
    
    def onMessage(self, msg, binary):
        """ Method is called by the Autobahn engine when a message has been
            received from the client.
            
            @param msg:         Message which was received as a string.
            @type  msg:         str
            
            @param binary:      Flag which is True if the message has binary
                                format and False otherwise.
            @type  binary:      bool
        """
        log.msg('WebSocket: Received new message from client. '
                '(binary={0})'.format(binary))
        
        try:
            self._assembler.processMessage(msg, binary)
        except InvalidRequest as e:
            #msg = 'Invalid Request: {0}'.format(e)
            import traceback
            self.sendErrorMessage(traceback.format_exc())
        except DeadConnection:
            self.dropConnection()
        except Exception as e:   # TODO: Refine Error handling
            #import sys, traceback
            #msg = '\n'.join(traceback.format_exception_only(type(e), e)))
            import traceback
            self.sendErrorMessage(traceback.format_exc())
    
    def sendMessage(self, msg):
        """ Internally used method to send a message to the robot.
            
            Should not be used from outside the Protocol; instead use the
            methods 'sendDataMessage' or 'sendErrorMessage'.
            
            (Overwrites method from autobahn.websocket.WebSocketServerProtocol)
            
            @param msg:     Message which should be sent.
        """
        uriBinary, msgURI = recursiveBinarySearch(msg)
        
        WebSocketServerProtocol.sendMessage(self, json.dumps(msgURI))
        
        for binData in uriBinary:
            WebSocketServerProtocol.sendMessage(self,
                binData[0] + binData[1].getvalue(), binary=True)
    
    def sendDataMessage(self, iTag, clsName, msgID, msg):
        """ Callback for IRobot Avatar to send a data message to the robot
            using this websocket connection.
            
            @param iTag:        Tag which is used to identify the interface
                                from the message is sent.
            @type  iTag:        str
            
            @param clsName:     Message type/Service type consisting of the
                                package and the name of the message/service,
                                i.e. 'std_msgs/Int32'.
            @type  clsName:     str
            
            @param msgID:       Message ID which can be used to get a
                                correspondence between request and response
                                message for a service call.
            @type  msgID:       str
            
            @param msg:         Message which should be sent. It has to be a
                                JSON compatible dictionary where part or the
                                complete message can be replaced by a StringIO
                                instance which is interpreted as binary data.
            @type  msg:         {str : {} / base_types / StringIO} / StringIO
        """
        self.sendMessage({'type' : types.DATA_MESSAGE,
                          'data' : {'iTag' : iTag, 'type' : clsName,
                                    'msgID' : msgID, 'msg' : msg}})
    
    def sendErrorMessage(self, msg):
        """ Callback for IRobot Avatar to send an error message to the robot
            using this websocket connection.
            
            @param msg:         Message which should be sent to the robot.
            @type  msg:         str
        """
        self.sendMessage({'data' : msg, 'type' : types.ERROR})
    
    def onClose(self, wasClean, code, reason):
        """ Method is called by the Autobahn engine when the connection has
            been lost.
        """
        if self._avatar:
            self._avatar.unregisterConnectionToRobot()
        
        self._assembler.stop()
        
        self._avatar = None
        self._assembler = None


class CloudEngineWebSocketFactory(WebSocketServerFactory):
    """ Factory which is used for the connections from the robots to the
        RoboEarth Cloud Engine.
    """
    def __init__(self, realm, reactor, masterIP, url, **kw):
        """ Initialize the Factory.
            
            @param portal:      Portal which is responsible for the
                                authentication of the websocket connection and
                                which will provide an Avatar for the robot in
                                the cloud engine.
            @type  portal:      twisted.cred.portal.Portal
            
            @param url:         URL where the websocket server factory will
                                listen for connections. For more information
                                refer to the base class:
                                    autobahn.websocket.WebSocketServerFactory
            @type  url:         str
            
            @param kw:          Additional keyworded arguments will be passed
                                to the __init__ of the base class.
        """
        WebSocketServerFactory.__init__(self, url, **kw)
        self._reactor = reactor
        self._realm = realm
        self._masterIP = masterIP
    
    def buildProtocol(self, addr):
        """ Method is called by the twisted reactor when a new connection
            attempt is made.
        """
        p = RobotWebSocketProtocol(self._realm, self._reactor, self._masterIP)
        p.factory = self
        return p
