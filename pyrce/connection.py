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
from urllib import urlencode
from urllib2 import urlopen, HTTPError
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
from autobahn.websocket import connectWS

# Custom package imports
import sys                          ### TODO:
sys.path.append('../framework')     ### TEMPORARY FIX

from client import types
from client.assembler import recursiveBinarySearch

# Custom local imports
from comm import RCERobotFactory
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
        
        if self._connectedDeferred:
            self._connectedDeferred.callback(self)
    
    def unregisterConnection(self, conn):
        """ Callback for RCERobotFactory.
        """
        if not self._conn:
            raise ConnectionError('There is no connection registered.')
        
        if self._conn != conn:
            raise ConnectionError('This connection is not registered.')
        
        self._conn = None
    
    def connect(self, masterUrl, deferred):
        """ Connect to RCE.
        """
        # First make a HTTP request to the Master to get a temporary key
        argList = [('userID', self._userID), ('robotID', self._robotID)]
        url = '{0}?{1}'.format(masterUrl, urlencode(argList))
        
        try:
            f = urlopen(url)
        except HTTPError as e:
            raise ConnectionError('HTTP Error {0}: {1} - '
                                  '{2}'.format(e.getcode(), e.msg, e.read()))
        
        # Read the response
        resp = json.loads(f.read())
        url = resp['url']
        argList.append(('key', resp['key']))
        
        # TODO: Hack for localhost IP address still necessary?
        lb = 2+url.find('//')
        rb = url.rfind(':')
        
        if url[lb:rb] == '127.0.0.1':
            print('Warning: Received localhost IP address!')
            mlb = 2+masterUrl.find('//')
            mrb = masterUrl.rfind(':')
            url = '{0}{1}{2}'.format(url[:lb], masterUrl[mlb:mrb], url[rb:])
        
        # Make websocket connection to Robot Manager
        url = '{0}?{1}'.format(url, urlencode(argList))
        factory = RCERobotFactory(url, self)
        # TODO: # What should this (v) be for ?!?
        #self._reactor.callLater(1, connectWS, factory)
        
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
    
    def addNode(self, cTag, nTag, pkg, exe, args='', name='', namespace=''):
        """ Add a node.
        """
        node = {'containerTag':cTag, 'nodeTag':nTag, 'pkg':pkg, 'exe':exe}
        
        if args:
            node['args'] = args
        
        if name:
            node['name'] = name
        
        if namespace:
            node['namespace'] = namespace
        
        self._sendMessage({'type':types.CONFIGURE_COMPONENT,
                           'data':{'addNodes':[node]}})
    
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
    
    def addInterface(self, eTag, iTag, iType, iCls, addr=''):
        """ Add an interface.
        """
        if iType not in ['ServiceInterface', 'ServiceProviderInterface',
                         'PublisherInterface', 'SubscriberInterface',
                         'ServiceConverter', 'ServiceProviderConverter',
                         'PublisherConverter', 'SubscriberConverter']:
            raise TypeError('Interface type is not valid.')
        
        iType = self.INTERFACE_MAP.get(iType, iType)
        
        interface = {'endpointTag':eTag, 'interfaceTag':iTag,
                     'interfaceType':iType, 'className':iCls}
        
        if addr:
            interface['addr'] = addr
        
        self._sendMessage({'type':types.CONFIGURE_COMPONENT,
                           'data':{'addInterfaces':[interface]}})
    
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
    
    def registerInterface(self, iTag, interface, unique):
        """ Callback for Interface.
            
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
        elif msgType == types.DATA_MESSAGE:
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
        
        return interface.Subscriber(self, iTag, msgType, cb)
    
    def service(self, iTag, srvType, cb=None):
        """ Create a Service.
            
            @param cb:  Can be used to specify a default callback for received
                        Service responses.
            @type  cb:  Callbable
        """
        if cb and not callable(cb):
            raise TypeError('Callback has to be callable.')
        
        return interface.Service(self, iTag, srvType, cb)


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
            return interface.ROSSubscriber(self, iTag, msgType, addr)
        
        def service(self, iTag, srvType, addr):
            """ Create a Service using ROS.
            """
            return interface.ROSService(self, iTag, srvType, addr)
