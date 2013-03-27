#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     rce-client/rce/client/connection.py
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
import weakref

# zope specific imports
from zope.interface import implements

# Custom imports
from rce.comm.interfaces import IRobot, IMessageReceiver
from rce.comm.client import RCE
from rce.client.interface import HAS_ROS
from rce.client.interface import Publisher, Subscriber, Service

if HAS_ROS:
    from rce.client.interface import ROSPublisher, ROSSubscriber, ROSService
    from rce.util.loader import Loader


class ConnectionError(Exception):
    """ Error is raised when there is no connection or the connection is
        not valid.
    """


class _Connection(object):
    """ Abstract implementation of a Connection.
    """
    implements(IRobot, IMessageReceiver)
    
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
        self._password = password
        self._reactor = reactor
        
        self._rce = None
        self._interfaces = {}
        
        self.disconnect()
    
    def __del__(self):
        self.disconnect()
    
    @property
    def reactor(self):
        """ Reference to twisted::reactor. """
        return self._reactor
    
    def _notConnected(self, *args, **kw):
        """ Method can not be used as long as there is no connection to the RCE.
        """
        raise ConnectionError('No connection to RCE.')
    
    def _addInterface(self, eTag, iTag, iType, iCls, addr=''):
        iType = self.INTERFACE_MAP.get(iType, iType)
        self._rce.addInterface(eTag, iTag, iType, iCls, addr)
    
    _addInterface.__doc__ = RCE.addInterface.__doc__ #@UndefinedVariable
    
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
        if self._rce:
            raise ConnectionError('There is already a connection registered.')
        
        self._rce = RCE(self, self._userID, self._robotID, self._password)
        
        # Setup forwarding
        self.sendMessage = self._rce.sendMessage
        self.createContainer = self._rce.createContainer
        self.destroyContainer = self._rce.destroyContainer
        self.addNode = self._rce.addNode
        self.removeNode = self._rce.removeNode
        self.addParameter = self._rce.addParameter
        self.removeParameter = self._rce.removeParameter
        self.addParameter = self._rce.addParameter
        self.addInterface = self._addInterface
        self.removeInterface = self._rce.removeInterface
        self.addConnection = self._rce.addConnection
        self.removeConnection = self._rce.removeConnection
    
    def disconnect(self):
        """ Disconnect from RCE.
        """
        if self._rce:
            self._rce.close()
        
        self._rce = None
        
        self.sendMessage = self._notConnected
        self.createContainer = self._notConnected
        self.destroyContainer = self._notConnected
        self.addNode = self._notConnected
        self.removeNode = self._notConnected
        self.addParameter = self._notConnected
        self.removeParameter = self._notConnected
        self.addParameter = self._notConnected
        self.addInterface = self._notConnected
        self.removeInterface = self._notConnected
        self.addConnection = self._notConnected
        self.removeConnection = self._notConnected
    
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
    
    def processReceivedMessage(self, iTag, clsName, msgID, msg):
        try:
            interfaces = self._interfaces[iTag].copy()
        except (KeyError, weakref.ReferenceError):
            interfaces = []
        
        for interface in interfaces:
            interface.callback(clsName, msg, msgID)
    
    processReceivedMessage.__doc__ = \
        IMessageReceiver.get('processReceivedMessage').getDoc()


class Connection(_Connection):
    """ Connection which should be used for JSON based messages.
    """
    INTERFACE_MAP = {
        'ServiceClientForwarder'   : 'ServiceClientConverter',
        'ServiceProviderForwarder' : 'ServiceProviderConverter',
        'PublisherForwarder'       : 'PublisherConverter',
        'SubscriberForwarder'      : 'SubscriberConverter'
    }
    
    def publisher(self, iTag, msgType):
        """ Create a Publisher.
            
            @param iTag:        Unique tag which will be used to identify the
                                publisher.
            @type  iTag:        str
            
            @param msgType:     ROS message which will be published, e.g.
                                    'std_msgs/String'
            @type  msgType:     str
        """
        return Publisher(self, iTag, msgType)
    
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
        
        return Subscriber(self, iTag, msgType, cb)
    
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
        
        return Service(self, iTag, srvType, cb)


if HAS_ROS:
    class ROSConnection(_Connection):
        """ Connection which should be used for ROS based messages.
        """
        INTERFACE_MAP = {
            'ServiceClientConverter'   : 'ServiceClientForwarder',
            'ServiceProviderConverter' : 'ServiceProviderForwarder',
            'PublisherConverter'       : 'PublisherForwarder',
            'SubscriberConverter'      : 'SubscriberForwarder'
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
            return ROSPublisher(self, iTag, msgType, addr)
        
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
            return ROSSubscriber(self, iTag, msgType, addr)
        
        def service(self, iTag, srvType, addr):
            """ Create a Service using ROS.
            
            @param iTag:        Unique tag which will be used to identify the
                                service.
            @type  iTag:        str
            
            @param srvType:     ROS Service which will used.
            @type  srvType:     str
            
            @param addr:        Address where the service will be available.
            """
            return ROSService(self, iTag, srvType, addr)
