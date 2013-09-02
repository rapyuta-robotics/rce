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
import re

# zope specific imports
from zope.interface import implements

# rce specific imports
from rce.comm.interfaces import IRobot, IClient
from rce.comm.client import RCE, ConnectionError
from rce.client.interface import HAS_ROS
from rce.client.interface import Publisher, Subscriber, \
    ServiceClient, ServiceProvider

if HAS_ROS:
    from rce.client.interface import ROSPublisher, ROSSubscriber, \
        ROSServiceClient, ROSServiceProvider
    from rce.util.loader import Loader


_IP_V4_REGEX = re.compile('^(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.)'
                          '{3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$')


class _Connection(object):
    """ Abstract implementation of a Connection.
    """
    implements(IRobot, IClient)

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

    def connect(self, masterUrl, deferred):
        """ Connect to RCE.

            @param masterUrl:   URL of Authentication Handler of Master Manager
            @type  masterUrl:   str

            @param deferred:    Deferred which is called as soon as the
                                connection was successfully established.
            @type  deferred:    twisted.internet.defer.Deferred

            @raise:             ConnectionError, if no connection could be
                                established.
        """
        if self._rce:
            raise ConnectionError('There is already a connection registered.')

        self._rce = RCE(self, self._userID, self._robotID, self._password,
                        self._reactor)

        # Connect
        self._rce.connect(masterUrl, deferred)

    def disconnect(self):
        """ Disconnect from RCE.
        """
        if self._rce:
            self._rce.close()

        self._rce = None

    # Callback Interface objects

    def registerInterface(self, iTag, interface):
        """ Callback for Interface.

            @param iTag:        Tag of interface which should be registered.
            @type  iTag:        str

            @param interface:   Interface instance which should be registered.
            @type  interface:   rce.client.interface.*
        """
        if iTag not in self._interfaces:
            self._interfaces[iTag] = weakref.WeakSet()
        elif interface.UNIQUE:
            raise ValueError('Can not have multiple interfaces with the same '
                             'tag.')

        self._interfaces[iTag].add(interface)

    def unregisterInterface(self, iTag, interface):
        """ Callback for Interfaces.

            @param iTag:        Tag of interface which should be unregistered.
            @type  iTag:        str

            @param interface:   Interface instance which should be
                                unregistered.
            @type  interface:   rce.client.interface.*
        """
        if iTag not in self._interfaces:
            raise ValueError('No Interface registered with tag '
                             "'{0}'.".format(iTag))

        interfaces = self._interfaces[iTag]
        interfaces.discard(interface)

        if not interfaces:
            del self._interfaces[iTag]

    # Callback Client Protocol

    def processReceivedMessage(self, iTag, clsName, msgID, msg):
        try:
            interfaces = self._interfaces[iTag].copy()
        except (KeyError, weakref.ReferenceError):
            interfaces = []

        for interface in interfaces:
            if interface.CALLABLE:
                interface.callback(clsName, msg, msgID)

    processReceivedMessage.__doc__ = \
        IClient.get('processReceivedMessage').getDoc()

    def processInterfaceStatusUpdate(self, iTag, status):
        try:
            interfaces = self._interfaces[iTag].copy()
        except (KeyError, weakref.ReferenceError):
            interfaces = []

        for interface in interfaces:
            interface.setEnabled(status)

    processInterfaceStatusUpdate.__doc__ = \
        IClient.get('processInterfaceStatusUpdate').getDoc()

    # Forwarding

    def sendMessage(self, dest, msgType, msg, msgID):
        if not self._rce:
            raise ConnectionError('No connection to RCE.')

        self._rce.sendMessage(dest, msgType, msg, msgID)

    sendMessage.__doc__ = RCE.sendMessage.__doc__  #@UndefinedVariable

    def createContainer(self, cTag, group='', groupIp='', size=1, cpu=0,
                        memory=0, bandwidth=0, specialFeatures=[]):
        if not self._rce:
            raise ConnectionError('No connection to RCE.')

        # ensure all whitespace characters around group are stripped
        group = group.strip()

        if groupIp and not _IP_V4_REGEX.match(groupIp):
            raise ValueError('Invalid IPv4 address')

        self._rce.createContainer(cTag, group, groupIp, size, cpu, memory,
                                  bandwidth, specialFeatures)

    createContainer.__doc__ = RCE.createContainer.__doc__  #@UndefinedVariable

    def destroyContainer(self, cTag):
        if not self._rce:
            raise ConnectionError('No connection to RCE.')

        self._rce.destroyContainer(cTag)

    destroyContainer.__doc__ = RCE.destroyContainer.__doc__  #@UndefinedVariable

    def addNode(self, cTag, nTag, pkg, exe, args='', name='', namespace=''):
        if not self._rce:
            raise ConnectionError('No connection to RCE.')

        self._rce.addNode(cTag, nTag, pkg, exe, args, name, namespace)

    addNode.__doc__ = RCE.addNode.__doc__  #@UndefinedVariable

    def removeNode(self, cTag, nTag):
        if not self._rce:
            raise ConnectionError('No connection to RCE.')

        self._rce.removeNode(cTag, nTag)

    removeNode.__doc__ = RCE.removeNode.__doc__  #@UndefinedVariable

    def addParameter(self, cTag, name, value):
        if not self._rce:
            raise ConnectionError('No connection to RCE.')

        self._rce.addParameter(cTag, name, value)

    addParameter.__doc__ = RCE.addParameter.__doc__  #@UndefinedVariable

    def removeParameter(self, cTag, name):
        if not self._rce:
            raise ConnectionError('No connection to RCE.')

        self._rce.removeParameter(cTag, name)

    removeParameter.__doc__ = RCE.removeParameter.__doc__  #@UndefinedVariable

    def addInterface(self, eTag, iTag, iType, iCls, addr=''):
        if not self._rce:
            raise ConnectionError('No connection to RCE.')

        iType = self.INTERFACE_MAP.get(iType, iType)
        self._rce.addInterface(eTag, iTag, iType, iCls, addr)

    addInterface.__doc__ = RCE.addInterface.__doc__  #@UndefinedVariable

    def removeInterface(self, eTag, iTag):
        if not self._rce:
            raise ConnectionError('No connection to RCE.')

        self._rce.removeInterface(eTag, iTag)

    removeInterface.__doc__ = RCE.removeInterface.__doc__  #@UndefinedVariable

    def addConnection(self, tagA, tagB):
        if not self._rce:
            raise ConnectionError('No connection to RCE.')

        self._rce.addConnection(tagA, tagB)

    addConnection.__doc__ = RCE.addConnection.__doc__  #@UndefinedVariable

    def removeConnection(self, tagA, tagB):
        if not self._rce:
            raise ConnectionError('No connection to RCE.')

        self._rce.removeConnection(tagA, tagB)

    removeConnection.__doc__ = RCE.removeConnection.__doc__  #@UndefinedVariable


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

            @return:            New Publisher instance.
            @rtype:             rce.client.interface.Publisher
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

            @return:            New Subscriber instance.
            @rtype:             rce.client.interface.Subscriber
        """
        if not callable(cb):
            raise TypeError('Callback has to be callable.')

        return Subscriber(self, iTag, msgType, cb)

    def serviceClient(self, iTag, srvType, cb=None):
        """ Create a Service Client.

            @param iTag:        Unique tag which will be used to identify the
                                service.
            @type  iTag:        str

            @param srvType:     ROS Service which will used.
            @type  srvType:     str

            @param cb:          Can be used to specify a default callback for
                                received service responses; it should take the
                                response as the single argument.
            @type  cb:          callable

            @return:            New Service Client instance.
            @rtype:             rce.client.interface.ServiceClient
        """
        if cb and not callable(cb):
            raise TypeError('Callback has to be callable.')

        return ServiceClient(self, iTag, srvType, cb)

    def serviceProvider(self, iTag, srvType, cb, *args):
        """ Create a Service Provider.

            @param iTag:        Unique tag which will be used to identify the
                                service.
            @type  iTag:        str

            @param srvType:     ROS Service which will be provided.
            @type  srvType:     str

            @param cb:          Callback which will be called when a request has
                                been received. The callback will receive the
                                request as first argument and all additional
                                arguments. The callback should return the
                                response message if the request was successful
                                and None otherwise.
            @type  cb:          callable

            @param *args:       All additional arguments are passed to the
                                callback.

            @return:            New Service Provider instance.
            @rtype:             rce.client.interface.ServiceProvider
        """
        if cb and not callable(cb):
            raise TypeError('Callback has to be callable.')

        return ServiceProvider(self, iTag, srvType, cb, args)


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

                @param iTag:        Unique tag which will be used to identify
                                    the publisher.
                @type  iTag:        str

                @param msgType:     ROS message which will be published, e.g.
                                    'std_msgs/String'
                @type  msgType:     str

                @param addr:        Topic where the publisher will listen for
                                    messages which should be published to the
                                    cloud engine.
                @type  addr:        str

                @return:            New Publisher instance.
                @rtype:             rce.client.interface.ROSPublisher
            """
            return ROSPublisher(self, iTag, msgType, addr)

        def subscriber(self, iTag, msgType, addr):
            """ Create a Subscriber using ROS.

                @param iTag:        Unique tag which will be used to identify
                                    the subscriber.
                @type  iTag:        str

                @param msgType:     ROS message to which will be subscribed,
                                    e.g. 'std_msgs/String'
                @type  msgType:     str

                @param addr:        Topic where the subscriber will publish
                                    received messages from to the RCE.
                @type  addr:        str

                @return:            New Subscriber instance.
                @rtype:             rce.client.interface.ROSSubscriber
            """
            return ROSSubscriber(self, iTag, msgType, addr)

        def serviceClient(self, iTag, srvType, addr):
            """ Create a Service Client using ROS.

                @param iTag:        Unique tag which will be used to identify
                                    the service.
                @type  iTag:        str

                @param srvType:     ROS Service which will used.
                @type  srvType:     str

                @param addr:        Address where the service will be available.

                @return:            New Service Client instance.
                @rtype:             rce.client.interface.ROSServiceClient
            """
            return ROSServiceClient(self, iTag, srvType, addr)

        def serviceProvider(self, iTag, srvType, addr):
            """ Create a Service Provider using ROS.

                @param iTag:        Unique tag which will be used to identify
                                    the service.
                @type  iTag:        str

                @param srvType:     ROS Service which will be provided.
                @type  srvType:     str

                @param addr:        Address where the service will be available.

                @return:            New Service Provider instance.
                @rtype:             rce.client.interface.ROSServiceProvider
            """
            return ROSServiceProvider(self, iTag, srvType, addr)
