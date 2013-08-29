#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-comm/rce/comm/client.py
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
import itertools
from urllib import urlencode
from urllib2 import urlopen, HTTPError
from hashlib import sha256

# zope specific imports
from zope.interface import implements

# twisted specific imports
from twisted.python.threadable import isInIOThread
from twisted.internet.threads import deferToThreadPool

# Autobahn specific imports
from autobahn.websocket import connectWS, WebSocketClientFactory, \
    WebSocketClientProtocol

# rce specific imports
from rce.comm import types
from rce.comm._version import CURRENT_VERSION
from rce.comm.interfaces import IRobot, IClient
from rce.comm.assembler import recursiveBinarySearch, MessageAssembler
from rce.util.interface import verifyObject


class RCERobotProtocol(WebSocketClientProtocol):
    """ WebSocket client protocol which is used to communicate with the Robot
        Manager.
    """
    def __init__(self, conn):
        """ Initialize the protocol.

            @param conn:        Connection instance which provides callback
                                functions.
            @type  conn:        rce.comm.client.RCE
        """
        self._connection = conn
        self._assembler = MessageAssembler(self, 60)
        self._registered = False

    def onOpen(self):
        """ This method is called by twisted as soon as the WebSocket
            connection has been successfully established.
        """
        self._assembler.start()
        self._connection.registerConnection(self)
        self._registered = True

    def onMessage(self, msg, binary):
        """ This method is called by twisted when a new message has been
            received.
        """
        self._assembler.processMessage(msg, binary)

    def processCompleteMessage(self, msg):
        """ Callback for MessageAssembler which will be called as soon as a
            message has been completed and is ready for processing.
        """
        self._connection.receivedMessage(msg)

    def sendMessage(self, msg):
        """ Internally used method to send messages via WebSocket connection.
            Thread-safe implementation.

            @param msg:         Message which should be sent.
        """
        binaries, msg = recursiveBinarySearch(msg)
        msg = json.dumps(msg)

        if isInIOThread():
            self._send(msg, binaries)
        else:
            self._connection.reactor.callFromThread(self._send, msg, binaries)

    def _send(self, msg, binaries):
        """ Internally used method to send messages via WebSocket connection.
            Handles the actual sending of the message. (Not thread-safe; use
            sendMessage instead.)
        """
        WebSocketClientProtocol.sendMessage(self, msg)

        for data in binaries:
            binMsg = data[0] + data[1].getvalue()
            WebSocketClientProtocol.sendMessage(self, binMsg, binary=True)

    def onClose(self, *args):
        """ This method is called by twisted when the connection has been
            closed.
        """
        if self._registered:
            self._connection.unregisterConnection(self)
            self._assembler.stop()
            self._registered = False

    def failHandshake(self, reason):
        """ This method is called by twisted when the connection could not be
            initialized.
        """
        print(reason)
        WebSocketClientProtocol.failHandshake(self, reason)


class RCERobotFactory(WebSocketClientFactory):
    """ WebSocket protocol factory which is used for the communication with the
        Robot Manager.
    """
    def __init__(self, url, conn):
        """ Initialize the factory.

            @param url:         URL of the Robot process.
            @type  url:         str

            @param conn:        Connection instance which provides callback
                                functions.
            @type  conn:        rce.comm.client.RCE
        """
        WebSocketClientFactory.__init__(self, url)
        self._connection = conn

    def buildProtocol(self, addr):
        """ This method is called by twisted when a new connection should be
            made.
        """
        p = RCERobotProtocol(self._connection)
        p.factory = self
        return p


class ConnectionError(Exception):
    """ Error is raised when there is no connection or the connection is
        not valid.
    """


class RCE(object):
    """ Class represents a connection to the RoboEarth Cloud Engine.
    """
    implements(IRobot)

    _PREFIXES = ['ServiceClient', 'ServiceProvider', 'Publisher', 'Subscriber']
    _SUFFIXES = ['Interface', 'Converter', 'Forwarder']
    _INTERFACES = [''.join(t) for t in itertools.product(_PREFIXES, _SUFFIXES)]

    def __init__(self, receiver, userID, robotID, password, reactor):
        """ Initialize the Connection.

            @param receiver:    Object which is responsible for the processing
                                of received data messages.
            @type  receiver:    rce.comm.interfaces.IMessageReceiver

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
        verifyObject(IClient, receiver)

        self._receiver = receiver
        self._userID = userID
        self._robotID = robotID
        self._password = sha256(password).hexdigest()
        self._reactor = reactor
        self._conn = None
        self._connectedDeferred = None

    @property
    def reactor(self):
        """ Reference to twisted::reactor. """
        return self._reactor

    def registerConnection(self, conn):
        """ Callback for RCERobotProtocol.

            @param conn:        Connection which should be registered.
            @type  conn:        rce.comm.client.RCERobotProtocol
        """
        if self._conn:
            raise ConnectionError('There is already a connection registered.')

        self._conn = conn
        print('Connection to RCE established.')

        if self._connectedDeferred:
            self._connectedDeferred.callback(self)
            self._connectedDeferred = None

    def unregisterConnection(self, conn):
        """ Callback for RCERobotProtocol.

            @param conn:        Connection which should be unregistered.
            @type  conn:        rce.comm.client.RCERobotProtocol
        """
        if not self._conn:
            raise ConnectionError('There is no connection registered.')

        if self._conn != conn:
            raise ConnectionError('This connection is not registered.')

        self._conn = None
        print('Connection closed.')

    def _getRobotURL(self, masterUrl):
        """ Internally used method to connect to the Master process to get
            a URL of a Robot process.
        """
        print('Connect to Master Process on: {0}'.format(masterUrl))

        args = urlencode((('userID', self._userID),
                          ('version', CURRENT_VERSION)))

        try:
            f = urlopen('{0}?{1}'.format(masterUrl, args))
        except HTTPError as e:
            msg = e.read()

            if msg:
                msg = ' - {0}'.format(msg)

            raise ConnectionError('HTTP Error {0}: '
                                  '{1}{2}'.format(e.getcode(), e.msg, msg))

        return json.loads(f.read())

    def _robotConnect(self, resp):
        """ Internally used method to connect to the Robot process.
        """
        # Read the response
        url = resp['url']
        current = resp.get('current', None)

        if current:
            print("Warning: There is a newer client (version: '{0}') "
                  'available.'.format(current))

        print('Connect to Robot Process on: {0}'.format(url))

        # Make WebSocket connection to Robot Manager
        args = urlencode((('userID', self._userID), ('robotID', self._robotID),
                          ('password', self._password)))
        factory = RCERobotFactory('{0}?{1}'.format(url, args), self)
        connectWS(factory)

    def connect(self, masterUrl, deferred):
        """ Connect to RCE.

            @param masterUrl:   URL of Master process.
            @type  masterUrl:   str

            @param deferred:    Deferred which is called as soon as the
                                connection was successfully established.
            @type  deferred:    twisted.internet.defer.Deferred

            @raise:             ConnectionError, if no connection could be
                                established.
        """
        self._connectedDeferred = deferred

        def eb(e):
            print(e.getErrorMessage())

            if self._connectedDeferred:
                self._connectedDeferred.errback(e)
                self._connectedDeferred = None

        d = deferToThreadPool(self._reactor, self._reactor.getThreadPool(),
                              self._getRobotURL, masterUrl)
        d.addCallback(self._robotConnect)
        d.addErrback(eb)

    def close(self):
        """ Disconnect from RCE.
        """
        if self._conn:
            self._conn.dropConnection()

    def _sendMessage(self, msgType, msgData):
        """ Internally used method to send messages via RCERobotProtocol.

            @param msgType:     String describing the type of the message.
            @type  msgType:     str

            @param msgData:     Message which should be sent.
        """
        if not self._conn:
            raise ConnectionError('No connection registered.')

        self._conn.sendMessage({'type':msgType, 'data':msgData})

    def sendMessage(self, dest, msgType, msg, msgID):
        """ Send a data message to the cloud engine.

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
        self._sendMessage(types.DATA_MESSAGE, {'iTag':dest, 'type':msgType,
                                               'msgID':msgID, 'msg':msg})

    def createContainer(self, cTag, group='', groupIp='', size=1, cpu=0,
                        memory=0, bandwidth=0, specialFeatures=[]):
        """ Create a container.

            @param cTag:                Unique tag which will be used to
                                        identify the container to create.
            @type  cTag:                str

            @param group:               The container group to which the
                                        container will be added.
            @type  group:               str

            @param groupIp:             The static IPv4 address which will be
                                        assigned to the container inside the
                                        group.
            @type  groupIp:             str

            @param size:                The container instance size.
            @type  size:                int

            @param cpu:                 CPU Allocation.
            @type  cpu:                 int

            @param memory:              Memory Allocation.
            @type  memory:              int

            @param bandwidth:           Bandwidth allocation.
            @type  bandwidth:           int

            @param specialFeatures:     Special features required,
                                        e.g. ['gpu','hadoop','avxii'].
            @type  specialFeatures:     list
        """
        print("Request creation of container '{0}'.".format(cTag))
        data = {}

        if group:
            data['group'] = group

        if groupIp:
            data['groupIp'] = groupIp

        if size:
            data['size'] = size

        if cpu:
            data['cpu'] = cpu

        if memory:
            data['memory'] = memory

        if bandwidth:
            data['bandwidth'] = bandwidth

        if specialFeatures:
            data['specialFeatures'] = specialFeatures

        container = {'containerTag':cTag}

        if data:
            container['containerData'] = data

        self._sendMessage(types.CREATE_CONTAINER, container)

    def destroyContainer(self, cTag):
        """ Destroy a container.

            @param cTag:        Tag of the container to remove.
            @type  cTag:        str
        """
        print("Request destruction of container '{0}'.".format(cTag))
        self._sendMessage(types.DESTROY_CONTAINER, {'containerTag':cTag})

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
        print("Request addition of node '{0}' to container '{1}' "
              '[pkg: {2}; exe: {3}].'.format(nTag, cTag, pkg, exe))
        node = {'containerTag':cTag, 'nodeTag':nTag, 'pkg':pkg, 'exe':exe}

        if args:
            node['args'] = args

        if name:
            node['name'] = name

        if namespace:
            node['namespace'] = namespace

        self._sendMessage(types.CONFIGURE_COMPONENT, {'addNodes':[node]})

    def removeNode(self, cTag, nTag):
        """ Remove a node.

            @param cTag:        Tag of container in which the node should be
                                removed.
            @type  cTag:        str

            @param nTag:        Tag the node to remove.
            @type  nTag:        str
        """
        print("Request removal of node '{0}' from container "
              "'{1}'.".format(nTag, cTag))
        node = {'containerTag':cTag, 'nodeTag':nTag}
        self._sendMessage(types.CONFIGURE_COMPONENT, {'removeNodes':[node]})

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
        print("Request addition of parameter '{0}' to container "
              "'{1}'.".format(name, cTag))
        param = {'containerTag':cTag, 'name':name, 'value':value}
        self._sendMessage(types.CONFIGURE_COMPONENT, {'setParam':[param]})

    def removeParameter(self, cTag, name):
        """ Remove a parameter.

            @param cTag:        Tag of container in which the parameter should
                                be removed.
            @type  cTag:        str

            @param name:        Name of the parameter to remove.
            @type  name:        str
        """
        print("Request removal of parameter '{0}' from container "
              "'{1}'.".format(name, cTag))
        param = {'containerTag':cTag, 'name':name}
        self._sendMessage(types.CONFIGURE_COMPONENT, {'deleteParam':[param]})

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
        print("Request addition of interface '{0}' of type '{1}' to endpoint "
              "'{2}'.".format(iTag, iType, eTag))
        if iType not in self._INTERFACES:
            raise TypeError('Interface type is not valid.')

        iface = {'endpointTag':eTag, 'interfaceTag':iTag,
                 'interfaceType':iType, 'className':iCls}

        if addr:
            iface['addr'] = addr

        self._sendMessage(types.CONFIGURE_COMPONENT, {'addInterfaces':[iface]})

    def removeInterface(self, eTag, iTag):
        """ Remove an interface.

            @param eTag:        Tag of endpoint from which the interface should
                                be removed. (Either a container tag or
                                robot ID)
            @type  eTag:        str

            @param iTag:        Tag of interface to remove.
            @type  iTag:        str
        """
        print("Request removal of interface '{0}'.".format(iTag))
        iface = {'endpointTag':eTag, 'interfaceTag':iTag}
        self._sendMessage(types.CONFIGURE_COMPONENT,
                          {'removeInterfaces':[iface]})

    def addConnection(self, tagA, tagB):
        """ Create a connection.

            @param tagX:        One of the interfaces which should be
                                connected. The tag has to be of the form
                                    [endpoint tag]/[interface tag]
            @type  tagX:        str
        """
        print("Request creation of connection between interface '{0}' and "
              "'{1}'.".format(tagA, tagB))
        conn = {'tagA':tagA, 'tagB':tagB}
        self._sendMessage(types.CONFIGURE_CONNECTION, {'connect':[conn]})

    def removeConnection(self, tagA, tagB):
        """ Destroy a connection.

            @param tagX:        One of the interfaces which should be
                                disconnected. The tag has to be of the form
                                    [endpoint tag]/[interface tag]
            @type  tagX:        str
        """
        print("Request destruction of connection between interface '{0}' and "
              "'{1}'.".format(tagA, tagB))
        conn = {'tagA':tagA, 'tagB':tagB}
        self._sendMessage(types.CONFIGURE_CONNECTION, {'disconnect':[conn]})

    def receivedMessage(self, msg):
        """ Callback from RCERobotProtocol.

            @param msg:         Message which has been received.
            @type  msg:         { str : {} / base_types / StringIO }
        """
        try:
            msgType = msg['type']
            data = msg['data']
        except KeyError as e:
            raise ValueError('Received message from robot process is missing '
                             'the key {0}.'.format(e))

        if msgType == types.ERROR:
            print('Received ERROR message: {0}'.format(data))
        elif msgType == types.STATUS:
            try:
                topic = data['topic']
            except KeyError as e:
                raise ValueError('Received STATUS message from robot process '
                                 'is missing the key {0}.'.format(e))

            if topic == types.STATUS_INTERFACE:
                try:
                    iTag = data['iTag']
                    status = data['status']
                except KeyError as e:
                    raise ValueError('Received STATUS message (Interface '
                                     'Status Update) from robot process is '
                                     'missing the key {0}.'.format(e))

                self._receiver.processInterfaceStatusUpdate(iTag, status)
            else:
                print('Received STATUS message with unknown content type: '
                      '{0}'.format(topic))
        elif msgType == types.DATA_MESSAGE:
            try:
                iTag = data['iTag']
                clsName = data['type']
                rosMsg = data['msg']
                msgID = data['msgID']
            except KeyError as e:
                raise ValueError('Received DATA message from robot process '
                                 'is missing the key {0}.'.format(e))

            self._receiver.processReceivedMessage(iTag, clsName, msgID, rosMsg)
        else:
            print('Received message with unknown message type: '
                  '{0}'.format(msgType))
