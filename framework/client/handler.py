#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     handler.py
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

# zope specific imports
from zope.interface import implements

# Custom imports
from errors import InvalidRequest, AuthenticationError
from client.interfaces import IClientMsgHandler
from client import types
from core.types import req


class AuthenticationHandler(object):
    """ Handler for Authentication Requests in Master.
    """
    def __init__(self, manager):
        """ Initialize the handler.
            
            @param manager:     Master Manager which is used to handle the
                                request.
            @type  manager:     core.manager.MasterManager
        """
        self._manager = manager
    
    def handle(self, args):
        """ Handle the authentication request.
            
            @param args:    Arguments which are passed with the request.
            @type  args:    { str : [str] }
        """
        try:
            userID = args['userID']
            robotID = args['robotID']
        except KeyError as e:
            raise InvalidRequest('Request is missing parameter: {0}'.format(e))
        
        if len(userID) != 1:
            raise InvalidRequest("Parameter 'userID' has to be unique in "
                                 'request.')
        else:
            userID = userID[0]
        
        if len(robotID) != 1:
            raise InvalidRequest("Parameter 'robotID' has to be unique in "
                                 'request.')
        else:
            robotID = robotID[0]
        
        response = self._manager.newConnection(userID, robotID)
            
        if not response:
            raise AuthenticationError('Could not authenticate user.')
        
        key, ip = response
        
        return {'key' : key, 'url' : 'ws://{0}:9010/'.format(ip)}


class _ClientHandlerBase(object):
    """ Base class for all client message handler.
    """
    implements(IClientMsgHandler)
    
    def __init__(self, manager, uid):
        """ Initialize the handler.
            
            @param manager:     Manager which is used to handle client
                                requests.
            @type  manager:     core.manager.RobotManager
            
            @param uid:         Identifier for the robot: (userID, robotID)
            @type  uid:         (str, str)
        """
        self._manager = manager
        self._uid = uid


class CreateContainerHandler(_ClientHandlerBase):
    """ Handler for the message type 'CreateContainer'.
    """
    TYPE = types.CREATE_CONTAINER
    
    def handle(self, msg):
        try:
            self._manager.sendRequest(self._uid, req.CREATE_CONTAINER,
                                      (msg['containerTag'],))
        except KeyError as e:
            raise InvalidRequest('Can not process "CreateContainer" request. '
                                 'Missing key: {0}'.format(e))


class DestroyContainerHandler(_ClientHandlerBase):
    """ Handler for the message type 'DestroyContainer'.
    """
    TYPE = types.DESTROY_CONTAINER
    
    def handle(self, msg):
        self._manager.sendRequest(self._uid, req.DESTROY_CONTAINER,
                                  (msg['containerTag'],))


class ConfigureContainerHandler(_ClientHandlerBase):
    """ Handler for the message type 'ConfigureContainer'.
    """
    TYPE = types.CONFIGURE_COMPONENT
    
    def handle(self, msg):
        if 'addNodes' in msg:
            for node in msg['addNodes']:
                self._manager.sendRequest(self._uid, req.ADD_NODE,
                                          (node['containerTag'],
                                           node['nodeTag'],
                                           node['pkg'],
                                           node['exe'],
                                           node.get('args', ''),
                                           node.get('name', ''),
                                           node.get('namespace', '')))
        
        if 'removeNodes' in msg:
            for node in msg['removeNodes']:
                self._manager.sendRequest(self._uid, req.REMOVE_NODE,
                                          (node['containerTag'],
                                           node['nodeTag']))
        
        if 'addInterfaces' in msg:
            for conf in msg['addInterfaces']:
                self._manager.sendRequest(self._uid, req.ADD_INTERFACE,
                                          (conf['interfaceType'],
                                           conf['endpointTag'],
                                           conf['interfaceTag'],
                                           conf['className'],
                                           conf.get('addr', '')))
        
        if 'removeInterfaces' in msg:
            for interfaceTag in msg['removeInterfaces']:
                self._manager.sendRequest(self._uid, req.REMOVE_INTERFACE,
                                          (interfaceTag,))
        
        if 'setParam' in msg:
            for param in  msg['setParam']:
                self._manager.sendRequest(self._uid, req.ADD_PARAMETER,
                                          (param['containerTag'],
                                           param['name'],
                                           json.dumps(param['value']),
                                           param['paramType']))
        
        if 'deleteParam' in msg:
            for param in msg['deleteParam']:
                self._manager.sendRequest(self._uid, req.REMOVE_NODE,
                                          (param['containerTag'],
                                           param['name']))


class ConnectInterfacesHandler(_ClientHandlerBase):
    """ Handler for the message type 'ConnectInterfaces'.
    """
    TYPE = types.CONFIGURE_CONNECTION
    
    def handle(self, msg):
        if 'connect' in msg:
            for conn in msg['connect']:
                self._manager.sendRequest(self._uid, req.ADD_CONNECTION,
                                          (conn['tagA'], conn['tagB']))
        
        if 'disconnect' in msg:
            for conn in msg['disconnect']:
                self._manager.sendRequest(self._uid, req.REMOVE_CONNECTION,
                                          (conn['tagA'], conn['tagB']))


class DataMessageHandler(_ClientHandlerBase):
    """ Handler for the message type 'DataMessage'.
    """
    TYPE = types.DATA_MESSAGE
    
    def handle(self, msg):
        self._manager.receivedFromClient(self._uid, msg['orig'],
                                         msg['dest'], msg['type'],
                                         msg['msgID'], msg['msg'])
