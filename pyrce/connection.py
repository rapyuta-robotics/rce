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
from hashlib import sha256

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
from twisted.internet.threads import deferToThread
from autobahn.websocket import connectWS

import sys                          
sys.path.append('../framework')

# Custom imports
from rce.client import types
from rce.client.assembler import recursiveBinarySearch

from comm import RCERobotFactory
import interface

if interface.HAS_ROS:
    from rce.util.loader import Loader


_VERSION = '20130210'  # Client version


class ConnectionError(Exception):
    """ Error is raised when there is no connection or the connection is
        not valid.
    """


class _Connection(object):
    """ Abstract implementation of a Connection.
    """
    INTERFACE_MAP = {}
    
    def __init__(self, userID, robotID, password, reactor):
        """ Initialize the Connection.
            
            @param userID:      User ID which will be used to authenticate the
                                connection.
            @type  userID:      str
            
            @param robotID:     Robot ID which will be used to authenticate the
                                connection.
            @type  robotID:     str
            
            @param password:    Password which will be used to authenticate the
                                connection.
            @type  password:    str
            
            @param reactor:     Reference to reactor which is used for this
                                connection.
            @type  reactor:     twisted::reactor
        """
        self._userID = userID
        self._robotID = robotID
        self._password = sha256(password).digest()
        self._reactor = reactor
        
        self._argList = [('userID', self._userID)]
        
        self._conn = None
        self._connectedDeferred = None
        
        self._interfaces = {}
    
    @property
    def reactor(self):
        """ Reference to twisted::reactor. """
        return self._reactor
    
    def registerConnection(self, conn):
        """ Callback for RCERobotFactory.
            
            @param conn:        Connection which should be registered.
            @type  conn:        pyrce.comm.RCERobotProtocol
        """
        if self._conn:
            raise ConnectionError('There is already a connection registered.')
        
        self._conn = conn
        
        print('Connection to RCE established.')
        
        if self._connectedDeferred:
            self._connectedDeferred.callback(self)
    
    def unregisterConnection(self, conn):
        """ Callback for RCERobotFactory.
            
            @param conn:        Connection which should be unregistered.
            @type  conn:        pyrce.comm.RCERobotProtocol
        """
        if not self._conn:
            raise ConnectionError('There is no connection registered.')
        
        if self._conn != conn:
            raise ConnectionError('This connection is not registered.')
        
        self._conn = None
        print('Connection closed.')
        
        self._interfaces = {}
    
    def _masterConnect(self, masterUrl):
        """ Internally used method to connect to the Master manager.
        """
        print('Connect to Master Manager on: {0}'.format(masterUrl))
        
        args = self._argList+[('version', _VERSION)]
        
        try:
            f = urlopen('{0}?{1}'.format(masterUrl, urlencode(args)))
        except HTTPError as e:
            msg = e.read()
            
            if msg:
                msg = ' - {0}'.format(msg)
            
            raise ConnectionError('HTTP Error {0}: '
                                  '{1}{2}'.format(e.getcode(), e.msg, msg))
        
        return json.loads(f.read())
    
    def _robotConnect(self, resp):
        """ Internally used method to connect to the Robot manager.
        """
        # Read the response
        url = resp['url']
        current = resp.get('current', None)
        
        if current:
            print("Warning: There is a newer client (version: '{0}') "
                  'available.'.format(current))
        
        print('Connect to Robot Manager on: {0}'.format(url))
        
        # Make websocket connection to Robot Manager
        args = urlencode(self._argList+[('robotID', self._robotID), 
                                        ('password', self._password)])
        factory = RCERobotFactory('{0}?{1}'.format(url, args), self)
        connectWS(factory)
    
    def connect(self, masterUrl, deferred):
        """ Connect to RCE.
            
            @param masterUrl:   URL of Authentication Handler of Master Manager
            @type  masterUrl:   str
            
            @param deferred:    Deferred which is called as soon as the
                                connection was successfully established.
            @type  deferred:    twisted::Deferred
            
            @raise:             ConnectionError, if no connection could be
                                established.
        """
        self._connectedDeferred = deferred
        
        def eb(e):
            print(e.getErrorMessage())
        
        connection = deferToThread(self._masterConnect, masterUrl)
        connection.addCallbacks(self._robotConnect, eb)
    
    def close(self):
        """ Disconnect from RCE.
        """
        if self._conn:
            self._conn.dropConnection()
    
    def sendMessage(self, dest, msgType, msg, msgID):
        """ Callback for Interfaces.
            
            @param dest:        Interface tag of message destination.
            @type  dest:        str
            
            @param msgType:     ROS Message type in format "pkg/msg", e.g.
                                    'std_msgs/String'
            @type  msgType:     str
            
            @param msg:         Message which should be sent in form of a
                                dictionary matching the structure of the ROS
                                message and using StringIO instances for binary
                                message parts.
            @type msg:          { str : {} / base_types / StringIO }
            
            @param msgID:       Message ID which is used to match request and
                                response message.
            @type  msgID:       str
        """
        self._sendMessage({'type':types.DATA_MESSAGE,
                           'data':{'iTag':dest,
                                   'type':msgType,
                                   'msgID':msgID,
                                   'msg':msg}})
    
    def _sendMessage(self, msg):
        """ Internally used method to send messages via RCERobotProtocol.
            
            @param msg:         Message which should be sent.
        """
        if isInIOThread():
            self._sendMessageSynced(msg)
        else:
            self._reactor.callFromThread(self._sendMessageSynced, msg)
    
    def _sendMessageSynced(self, msg):
        """ Internally used method to send messages via RCERobotProtocol.
            
            @param msg:         Message which should be sent.
        """
        if not self._conn:
            raise ConnectionError('No connection registered.')
        
        uriBinary, msgURI = recursiveBinarySearch(msg)
        
        self._conn.sendMessage(json.dumps(msgURI))
        
        for data in uriBinary:
            self._conn.sendMessage(data[0] + data[1].getvalue(), binary=True)
    
    def createContainer(self, cTag):
        """ Create a container.
            
            @param cTag:        Unique tag which will be used to identify the
                                container to create.
            @type  cTag:        str
        """
        print('Request creation of container "{0}".'.format(cTag))
        self._sendMessage({'type':types.CREATE_CONTAINER,
                           'data':{'containerTag':cTag}})
    
    def destroyContainer(self, cTag):
        """ Destroy a container.
            
            @param cTag:        Tag of the container to remove.
            @type  cTag:        str
        """
        print('Request destruction of container "{0}".'.format(cTag))
        self._sendMessage({'type':types.DESTROY_CONTAINER,
                           'data':{'containerTag':cTag}})
    
    def addNode(self, cTag, nTag, pkg, exe, args='', name='', namespace=''):
        """ Add a node.
            
            @param cTag:        Tag of container in which the node should be
                                added.
            @type  cTag:        str
            
            @param nTag:        Unique tag within a container which will be
                                used to identify the node to add.
            @type  nTag:        str
            
            @param pkg:         Name of ROS package where the executable can be
                                found.
            @type  pkg:         str
            
            @param exe:         Name of the executable which should be
                                executed.
            @type  exe:         str
            
            @param args:        Optional arguments which should be passed to
                                the executable as a single string. Can contain
                                the directives $(find PKG) or $(env VAR). Other
                                special characters as '$' or ';' are not
                                allowed.
            @type  args:        str
            
            @param name:        Optional argument which defines the name used
                                in the ROS environment.
            @type  name:        str
            
            @param namespace:   Optional argument which defines the namespace
                                used in the ROS environment.
            @type  namespace:   str
        """
        print('Request addition of node "{0}" to container "{1}" '
              '[pkg: {2}; exe: {3}].'.format(nTag, cTag, pkg, exe))
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
            
            @param cTag:        Tag of container in which the node should be
                                removed.
            @type  cTag:        str
            
            @param nTag:        Tag the node to remove.
            @type  nTag:        str
        """
        print('Request removal of node "{0}" from container '
              '"{1}".'.format(nTag, cTag))
        self._sendMessage({'type':types.CONFIGURE_COMPONENT,
                           'data':{'removeNodes':[{'containerTag':cTag,
                                                   'nodeTag':nTag}]}})
    
    def addParameter(self, cTag, name, value):
        """ Add a parameter.
            
            @param cTag:        Tag of container in which the parameter should
                                be added.
            @type  cTag:        str
            
            @param name:        Name of the parameter which is used to identify
                                the parameter and which is used inside the
                                ROS environment.
            @type  name:        str
            
            @param value:       Value which should be added. String values can
                                contain the directives $(find PKG) or
                                $(env VAR).
            @type  value:       int/float/bool/str/[]
        """
        print('Request addition of parameter "{0}" to container '
              '"{1}".'.format(name, cTag))
        
        self._sendMessage({'type':types.CONFIGURE_COMPONENT,
                           'data':{'setParam':[{'containerTag':cTag,
                                                'name':name, 'value':value}]}})
    
    def removeParameter(self, cTag, name):
        """ Remove a parameter.
            
            @param cTag:        Tag of container in which the parameter should
                                be removed.
            @type  cTag:        str
            
            @param name:        Name of the parameter to remove.
            @type  name:        str
        """
        print('Request removal of parameter "{0}" from container '
              '"{1}".'.format(name, cTag))
        self._sendMessage({'type':types.CONFIGURE_COMPONENT,
                           'data':{'deleteParam':[{'containerTag':cTag,
                                                   'name':name}]}})
    
    def addInterface(self, eTag, iTag, iType, iCls, addr=''):
        """ Add an interface.
            
            @param eTag:        Tag of endpoint to which the interface should
                                be added. (Either a container tag or robot ID)
            @type  eTag:        str
            
            @param iTag:        Unique tag which will be used to identify the
                                interface to add.
            @type  iTag:        str
            
            @param iType:       Type of the interface, which needs one of the
                                following prefix:
                                    Service, ServiceProvider,
                                    Publisher, Subscriber
                                and on of the following suffix:
                                    Interface, Converter
            @type  iType:       str
            
            @param iCls:        ROS Message/Service type in format "pkg/msg",
                                e.g. 'std_msgs/String'
            @type  iCls:        str
            
            @param addr:        Optional argument which is used for ROS
                                Interfaces where the argument will provide
                                the name under which the interface will be
                                available in the local ROS environment.
            @type  addr:        str
        """
        print('Request addition of interface "{0}" of type "{1}" to endpoint '
              '"{2}".'.format(iTag, iType, eTag))
        if iType not in ['ServiceClientInterface', 'ServiceProviderInterface',
                         'PublisherInterface', 'SubscriberInterface',
                         'ServiceClientConverter', 'ServiceProviderConverter',
                         'PublisherConverter', 'SubscriberConverter']:
            raise TypeError('Interface type is not valid.')
        
        iType = self.INTERFACE_MAP.get(iType, iType)
        
        interface = {'endpointTag':eTag, 'interfaceTag':iTag,
                     'interfaceType':iType, 'className':iCls}
        
        if addr:
            interface['addr'] = addr
        
        self._sendMessage({'type':types.CONFIGURE_COMPONENT,
                           'data':{'addInterfaces':[interface]}})
    
    def removeInterface(self, eTag, iTag):
        """ Remove an interface.
            
            @param eTag:        Tag of endpoint from which the interface should
                                be removed. (Either a container tag or
                                robot ID)
            @type  eTag:        str
            
            @param iTag:        Tag of interface to remove.
            @type  iTag:        str
        """
        print('Request removal of interface "{0}".'.format(iTag))
        interface = {'endpointTag':eTag, 'interfaceTag':iTag}
        self._sendMessage({'type':types.CONFIGURE_COMPONENT,
                           'data':{'removeInterfaces':[interface]}})
    
    def addConnection(self, tag1, tag2):
        """ Create a connection.
            
            @param tagX:        One of the interfaces which should be
                                connected. The tag has to be of the form
                                    [endpoint tag]/[interface tag]
            @type  tagX:        str
        """
        print('Request creation of connection between interface "{0}" and '
              '"{1}".'.format(tag1, tag2))
        self._sendMessage({'type':types.CONFIGURE_CONNECTION,
                           'data':{'connect':[{'tagA':tag1,
                                               'tagB':tag2}]}})
    
    def removeConnection(self, tag1, tag2):
        """ Destroy a connection.
            
            @param tagX:        One of the interfaces which should be
                                disconnected. The tag has to be of the form
                                    [endpoint tag]/[interface tag]
            @type  tagX:        str
        """
        print('Request destruction of connection between interface "{0}" and '
              '"{1}".'.format(tag1, tag2))
        self._sendMessage({'type':types.CONFIGURE_CONNECTION,
                           'data':{'disconnect':[{'tagA':tag1,
                                                  'tagB':tag2}]}})
    
    def registerInterface(self, iTag, iface, unique):
        """ Callback for Interface.
            
            @param iTag:        Tag of interface which should be registered.
            @type  iTag:        str
            
            @param iface:       Interface instance which should be registered.
            @type  iface:       pyrce.interface._Subscriber/_Publisher/_Service
            
            @param unique:      Flag to indicate whether Interface should be
                                unique for its tag or not.
            @type  unique:      bool
        """
        if iTag not in self._interfaces:
            self._interfaces[iTag] = weakref.WeakSet()
        elif unique:
            ValueError('Can not have multiple interfaces with the same tag.')
        
        self._interfaces[iTag].add(iface)
        
    def unregisterInterface(self, iTag, iface):
        """ Callback for Interfaces.
            
            @param iTag:        Tag of interface which should be unregistered.
            @type  iTag:        str
            
            @param iface:       Interface instance which should be
                                unregistered.
            @type  iface:       pyrce.interface._Subscriber/_Publisher/_Service
        """
        if iTag not in self._interfaces:
            raise ValueError('No Interface register with tag '
                             '"{0}".'.format(iTag))
        
        interfaces = self._interfaces[iTag]
        interfaces.discard(iface)
        
        if not interfaces:
            del self._interfaces[iTag]
    
    def receivedMessage(self, msg):
        """ Callback from RCERobotProtocol.
            
            @param msg:         Message which has been received.
            @type  msg:         { str : {} / base_types / StringIO }
        """
        try:
            msgType = msg['type']
            data = msg['data']
        except KeyError as e:
            raise ValueError('Received message from robot manager is missing '
                             'the key {0}.'.format(e))
        
        if msgType == types.ERROR:
            print('Received error message: {0}'.format(data))
#        elif msgType == types.STATUS:
#            print('Received status message: {0}'.format(data))
        elif msgType == types.DATA_MESSAGE:
            self._processDataMessage(data)
        else:
            print('Received message with unknown message type: '
                  '{0}'.format(msgType))
    
    def _processDataMessage(self, dataMsg):
        """ Internally used message to process a Data Message.
            
            @param dataMsg:     Data part of data message which has been
                                received.
            @type  dataMsg:     {str : {} / base_types / StringIO} / StringIO
        """
        msgType = dataMsg['type']
        msg = dataMsg['msg']
        msgID = dataMsg['msgID']
        
        try:
            interfaces = self._interfaces[dataMsg['iTag']].copy()
        except (KeyError, weakref.ReferenceError):
            interfaces = []
        
        for interface in interfaces:
            interface.callback(msgType, msg, msgID)


class Connection(_Connection):
    """ Connection which should be used for JSON based messages.
    """
    def publisher(self, iTag, msgType):
        """ Create a Publisher.
            
            @param iTag:        Unique tag which will be used to identify the
                                publisher.
            @type  iTag:        str
            
            @param msgType:     ROS message which will be published, e.g.
                                    'std_msgs/String'
            @type  msgType:     str
        """
        return interface.Publisher(self, iTag, msgType)
    
    def subscriber(self, iTag, msgType, cb):
        """ Create a Subscriber.
            
            @param iTag:        Unique tag which will be used to identify the
                                subscriber.
            @type  iTag:        str
            
            @param msgType:     ROS message to which will be subscribed, e.g.
                                    'std_msgs/String'
            @type  msgType:     str
            
            @param cb:          Callback which will takes as single argument
                                the received message.
            @type  cb:          callable
        """
        if not callable(cb):
            raise TypeError('Callback has to be callable.')
        
        return interface.Subscriber(self, iTag, msgType, cb)
    
    def service(self, iTag, srvType, cb=None):
        """ Create a Service.
            
            @param iTag:        Unique tag which will be used to identify the
                                service.
            @type  iTag:        str
            
            @param srvType:     ROS Service which will used.
            @type  srvType:     str
            
            @param cb:          Can be used to specify a default callback for
                                received service responses; it should take the
                                repsonse as the single argument.
            @type  cb:          callbable
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
            """ Reference to Loader. """
            return self._LOADER
        
        def publisher(self, iTag, msgType, addr):
            """ Create a Publisher using ROS.
            
            @param iTag:        Unique tag which will be used to identify the
                                publisher.
            @type  iTag:        str
            
            @param msgType:     ROS message which will be published, e.g.
                                    'std_msgs/String'
            @type  msgType:     str
            
            @param addr:        Topic where the publisher will listen for
                                messages which should be published to the RCE.
            @type  addr:        str
            """
            return interface.ROSPublisher(self, iTag, msgType, addr)
        
        def subscriber(self, iTag, msgType, addr):
            """ Create a Subscriber using ROS.
            
            @param iTag:        Unique tag which will be used to identify the
                                subscriber.
            @type  iTag:        str
            
            @param msgType:     ROS message to which will be subscribed, e.g.
                                    'std_msgs/String'
            @type  msgType:     str
            
            @param addr:        Topic where the subscriber will publish
                                received messages from to the RCE.
            @type  addr:        str
            """
            return interface.ROSSubscriber(self, iTag, msgType, addr)
        
        def service(self, iTag, srvType, addr):
            """ Create a Service using ROS.
            
            @param iTag:        Unique tag which will be used to identify the
                                service.
            @type  iTag:        str
            
            @param srvType:     ROS Service which will used.
            @type  srvType:     str
            
            @param addr:        Address where the service will be available.
            """
            return interface.ROSService(self, iTag, srvType, addr)
