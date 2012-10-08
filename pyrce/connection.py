#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     connection.py
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

# Python specific imports
import json
import weakref

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
from twisted.python.threadable import isInIOThread
from twisted.internet.defer import Deferred
from autobahn.websocket import connectWS

# Custom package imports
import sys                          ### TODO:
sys.path.append('../framework')     ### TEMPORARY FIX

from client import types
from client.assembler import recursiveBinarySearch

# Custom local imports
from comm import RCEMasterFactory, RCERobotFactory
import interface

# Custom package imports
if interface.HAS_ROS:
    from util.loader import Loader


class ConnectionError(Exception):
    """ Error is raised when there is no connection or the connection is
        not valid.
    """


class _Connection(object):
    """ Abstract implementation of a Connection.
    """
    INTERFACE_MAP = {}
    
    def __init__(self, userID, robotID, reactor):
        """ Initialize the Connection.
        """
        self._init = False
        self._userID = userID
        self._robotID = robotID
        self._reactor = reactor
        
        self._conn = None
        self._connectedDeferred = None
        
        self._interfaces = {}
    
    @property
    def reactor(self):
        """ Reference to twisted::reactor.
        """
        return self._reactor
    
    def registerConnection(self, conn):
        """ Callback for RCERobotFactory.
        """
        if self._conn:
            raise ConnectionError('There is already a connection registered.')
        
        self._conn = conn
        self._sendMessage({'type':types.INIT,
                           'data':{'userID':self._userID,
                                   'robotID':self._robotID,
                                   'key':self._key}})
    
    def unregisterConnection(self, conn):
        """ Callback for RCERobotFactory.
        """
        self._init = False
        
        if not self._conn:
            raise ConnectionError('There is no connection registered.')
        
        if self._conn != conn:
            raise ConnectionError('This connection is not registered.')
        
        self._conn = None
    
    def connect(self, masterUrl, connectedDeferred):
        """ Connect to RCE.
        """
        self._connectedDeferred = connectedDeferred
        deferred = Deferred()
        
        def authenticate(resp):
            self._key, url = resp
            
            lb = 2+url.find('//')
            rb = url.rfind(':')
            
            if url[lb:rb] == '127.0.0.1':
                mlb = 2+masterUrl.find('//')
                mrb = masterUrl.rfind(':')
                url = '{0}{1}{2}'.format(url[:lb], masterUrl[mlb:mrb],
                                         url[rb:])
            
            factory = RCERobotFactory(url, self)
            self._reactor.callLater(1, connectWS, factory)
        
        deferred.addCallbacks(authenticate)
        
        factory = RCEMasterFactory(masterUrl, self._userID, self._robotID,
                                   deferred)
        connectWS(factory)
    
    def close(self):
        """ Disconnect from RCE.
        """
        if self._conn:
            self._conn.dropConnection()
        
        self._subscribers = {}
    
    def sendMessage(self, dest, msgType, msg, msgID):
        """ Callback for Interfaces.
        """
        self._sendMessage({'type':types.DATA_MESSAGE,
                           'data':{'dest':dest,
                                   'orig':self._robotID,
                                   'type':msgType,
                                   'msgID':msgID,
                                   'msg':msg}})
    
    def _sendMessage(self, msg):
        """ Internally used method to send messages via RCERobotProtocol.
        """
        if isInIOThread():
            self._sendMessageSynced(msg)
        else:
            self._reactor.callFromThread(self._sendMessageSynced, msg)
    
    def _sendMessageSynced(self, msg):
        """ Internally used method to send messages via RCERobotProtocol.
        """
        if not self._conn:
            raise ConnectionError('No connection registered.')
        
        uriBinary, msgURI = recursiveBinarySearch(msg)
        
        self._conn.sendMessage(json.dumps(msgURI))
        
        for data in uriBinary:
            self._conn.sendMessage(data[0] + data[1].getvalue(), binary=True)
    
    def createContainer(self, cTag):
        """ Create a container.
        """
        self._sendMessage({'type':types.CREATE_CONTAINER,
                           'data':{'containerTag':cTag}})
    
    def destroyContainer(self, cTag):
        """ Destroy a container.
        """
        self._sendMessage({'type':types.DESTROY_CONTAINER,
                           'data':{'containerTag':cTag}})
    
    def addNode(self, cTag, nTag, pkg, exe, namespace):
        """ Add a node.
        """
        self._sendMessage({'type':types.CONFIGURE_COMPONENT,
                           'data':{'addNodes':[{'containerTag':cTag,
                                                'nodeTag':nTag,
                                                'pkg':pkg,
                                                'exe':exe,
                                                'namespace':namespace}]}})
    
    def removeNode(self, cTag, nTag):
        """ Remove a node.
        """
        self._sendMessage({'type':types.CONFIGURE_COMPONENT,
                           'data':{'removeNodes':[{'containerTag':cTag,
                                                   'nodeTag':nTag}]}})
    
    def addParameter(self, cTag, name, paramType, value):
        """ Add a parameter.
        """
        if paramType not in ['int', 'str', 'float', 'bool', 'file']:
            raise TypeError('Parameter type is not valid.')
        
        self._sendMessage({'type':types.CONFIGURE_COMPONENT,
                           'data':{'setParam':[{'containerTag':cTag,
                                                'name':name,
                                                'value':value,
                                                'paramType':paramType}]}})
    
    def removeParameter(self, cTag, name):
        """ Remove a parameter.
        """
        self._sendMessage({'type':types.CONFIGURE_COMPONENT,
                           'data':{'deleteParam':[{'containerTag':cTag,
                                                   'name':name}]}})
    
    def addInterface(self, eTag, iTag, iType, iCls, addr):
        """ Add an interface.
        """
        if iType not in ['ServiceInterface', 'ServiceProviderInterface',
                         'PublisherInterface', 'SubscriberInterface',
                         'ServiceConverter', 'ServiceProviderConverter',
                         'PublisherConverter', 'SubscriberConverter']:
            raise TypeError('Interface type is not valid.')
        
        iType = self.INTERFACE_MAP.get(iType, iType)
        
        self._sendMessage({'type':types.CONFIGURE_COMPONENT,
                           'data':{'addInterfaces':[{'endpointTag':eTag,
                                                     'interfaceTag':iTag,
                                                     'interfaceType':iType,
                                                     'className':iCls,
                                                     'addr':addr}]}})
    
    def removeInterface(self, iTag):
        """ Remove an interface.
        """
        self._sendMessage({'type':types.CONFIGURE_COMPONENT,
                           'data':{'removeInterfaces':[iTag]}})
    
    def addConnection(self, iTag1, iTag2):
        """ Create a connection.
        """
        self._sendMessage({'type':types.CONFIGURE_CONNECTION,
                           'data':{'connect':[{'tagA':iTag1,
                                               'tagB':iTag2}]}})
    
    def removeConnection(self, iTag1, iTag2):
        """ Destroy a connection.
        """
        self._sendMessage({'type':types.CONFIGURE_CONNECTION,
                           'data':{'disconnect':[{'tagA':iTag1,
                                                  'tagB':iTag2}]}})
    
    def _registerInterface(self, iTag, interface, unique):
        """ Internally used method to register an Interface.
            
            @param unique:  Flag to indicate whether Interface should be unique
                            for its tag or not.
            @type  unique:  bool
        """
        if iTag not in self._interfaces:
            self._interfaces[iTag] = weakref.WeakSet()
        elif unique:
            ValueError('Can not have multiple interfaces with the same tag.')
        
        self._interfaces[iTag].add(interface)
        
    def unregisterInterface(self, iTag, interface):
        """ Callback for Interfaces.
        """
        if iTag not in self._interfaces:
            raise ValueError('No Interface register with tag '
                             '"{0}".'.format(iTag))
        
        interfaces = self._interfaces[iTag]
        interfaces.discard(interface)
        
        if not interfaces:
            del self._interfaces[iTag]
    
    def receivedMessage(self, msg):
        """ Callback from RCERobotProtocol.
        """
        try:
            msgType = msg['type']
            data = msg['data']
        except KeyError as e:
            raise ValueError('Received message from robot manager is missing '
                             'the key {0}.'.format(e))
        
        if msgType == types.ERROR:
            print('Received error message: {0}'.format(data))
        elif msgType == types.STATUS:
            print('Received status message: {0}'.format(data))
            self._init = True
            
            if self._connectedDeferred:
                self._connectedDeferred.callback(self)
        elif self._init and msgType == types.DATA_MESSAGE:
            self._processDataMessage(data)
    
    def _processDataMessage(self, dataMsg):
        """ Internally used message to process a Data Message.
        """
        msgType = dataMsg['type']
        msg = dataMsg['msg']
        msgID = dataMsg['msgID']
        
        if dataMsg['dest'] != self._robotID:
            print('Received message which was not for this connection.')
            return
        
        try:
            interfaces = self._interfaces[dataMsg['orig']].copy()
        except (KeyError, weakref.ReferenceError):
            interfaces = []
        
        for interface in interfaces:
            interface.callback(msgType, msg, msgID)


class Connection(_Connection):
    """ Connection which should be used for JSON based messages.
    """
    def publisher(self, iTag, msgType):
        """ Create a Publisher.
        """
        return interface.Publisher(self, iTag, msgType)
    
    def subscriber(self, iTag, msgType, cb):
        """ Create a Subscriber.
        """
        if not callable(cb):
            raise TypeError('Callback has to be callable.')
        
        subscriber = interface.Subscriber(self, iTag, msgType, cb)
        self._registerInterface(iTag, subscriber, False)
        return subscriber
    
    def service(self, iTag, srvType, cb=None):
        """ Create a Service.
            
            @param cb:  Can be used to specify a default callback for received
                        Service responses.
            @type  cb:  Callbable
        """
        if cb and not callable(cb):
            raise TypeError('Callback has to be callable.')
        
        service = interface.Service(self, iTag, srvType, cb)
        self._registerInterface(iTag, service, False)
        return service


if interface.HAS_ROS:
    class ROSConnection(_Connection):
        """ Connection which should be used for ROS based messages.
        """
        INTERFACE_MAP = {
            'ServiceConverter' : 'ServiceForwarder',
            'ServiceProviderConverter' : 'ServiceProviderForwarder',
            'PublisherConverter' : 'PublisherForwarder',
            'SubscriberConverter' : 'SubscriberForwarder'
        }
        
        _LOADER = Loader()
        
        @property
        def loader(self):
            """ Reference to Loader.
            """
            return self._LOADER
        
        def publisher(self, iTag, msgType, addr):
            """ Create a Publisher using ROS.
            """
            return interface.ROSPublisher(self, iTag, msgType, addr)
        
        def subscriber(self, iTag, msgType, addr):
            """ Create a Subscriber using ROS.
            """
            subscriber = interface.ROSSubscriber(self, iTag, msgType, addr)
            self._registerInterface(iTag, subscriber, False)
            return subscriber
        
        def service(self, iTag, srvType, addr):
            """ Create a Service using ROS.
            """
            service = interface.ROSService(self, iTag, srvType, addr)
            self._registerInterface(iTag, service, False)
            return service
