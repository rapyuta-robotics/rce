#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-core/rce/robot.py
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
#     Copyright 2013 RoboEarth
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
import sys

# ROS specific imports
from rospkg.environment import get_ros_paths

# zope specific imports
from zope.interface import implements

# twisted specific imports
from twisted.python import log
from twisted.cred.credentials import UsernamePassword
from twisted.spread.pb import PBClientFactory, \
    DeadReferenceError, PBConnectionLost

# Autobahn specific imports
from autobahn.websocket import listenWS

# rce specific imports
from rce.util.converter import Converter
from rce.util.loader import Loader
from rce.util.interface import verifyObject
from rce.comm.error import DeadConnection
from rce.comm.interfaces import IRobotRealm, IProtocol, \
    IRobot, IMessageReceiver
from rce.comm.server import CloudEngineWebSocketFactory
from rce.monitor.interface.robot import PublisherConverter, \
    SubscriberConverter, ServiceClientConverter, ServiceProviderConverter, \
    PublisherForwarder, SubscriberForwarder, \
    ServiceClientForwarder, ServiceProviderForwarder
from rce.slave.endpoint import Endpoint
from rce.slave.namespace import Namespace
from rce.slave.interface import Types


class ForwardingError(Exception):
    """ Exception is raised if the call could not be forwarded.
    """


class Connection(object):
    """ Representation of a connection to a robot client.
    """
    implements(IRobot, IMessageReceiver)

    def __init__(self, client, userID, robotID):
        """ Initialize the representation of a connection to a robot client.

            @param client:      Client which is responsible for managing the
                                Robot process.
            @type  client:      rce.robot.RobotClient

            @param userID:      User ID of the robot owner
            @type  userID:      str

            @param robotID:     Unique ID which is used to identify the robot.
            @type  robotID:     str
        """
        client.registerConnection(self)
        self._client = client
        self._userID = userID
        self._robotID = robotID
        self._avatar = None
        self._view = None
        self._namespace = None
        self._protocol = None

    @property
    def userID(self):
        """ User ID of the user owing this connection. """
        return self._userID

    @property
    def robotID(self):
        """ Robot ID used to identify the connected robot. """
        return self._robotID

    def destroy(self):
        """ # TODO: Add doc
        """
        if self._protocol:
            self._protocol.dropConnection()

        if self._client:
            self._client.unregisterConnection(self)

        if self._view:
            self._view.destroy()

        if self._namespace:
            self._namespace.destroy()

        self._client = None
        self._namespace = None
        self._view = None
        self._avatar = None
        self._protocol = None

    # Callbacks for RobotClient

    def registerAvatar(self, avatar):
        """ Register User Avatar.

            # TODO: Add description
        """
        assert self._avatar is None
        self._avatar = avatar

    def registerView(self, view):
        """ # TODO: Add doc
        """
        assert self._view is None
        self._view = view

    def registerNamespace(self, namespace):
        """ # TODO: Add doc
        """
        assert self._namespace is None
        self._namespace = namespace

    def registerProtocol(self, protocol):
        """ Register the client protocol.

            @param protocol:    Protocol which should be registered.
            @type  protocol:    rce.comm.interfaces.IServersideProtocol
        """
        assert self._protocol is None
        verifyObject(IProtocol, protocol)
        self._protocol = protocol

    def unregisterProtocol(self, protocol):
        """ Unregister the client protocol.

            @param protocol:    Protocol which should be unregistered.
            @type  protocol:    rce.comm.interfaces.IServersideProtocol
        """
        self._protocol = None

    # Callbacks for View & Namespace

    def reportError(self, msg):
        self._protocol.sendErrorMessage(msg)

    reportError.__doc__ = IProtocol.get('sendErrorMessage').getDoc()

    def sendMessage(self, iTag, clsName, msgID, msg):
        if not self._protocol:
            # TODO: What should we do here?
            #       One solution would be to queue the messages here for some
            #       time...
            return

        self._protocol.sendDataMessage(iTag, clsName, msgID, msg)

    sendMessage.__doc__ = IProtocol.get('sendDataMessage').getDoc()

    def sendInterfaceStatusUpdate(self, iTag, status):
        if not self._protocol:
            # TODO: What should we do here?
            #       One solution would be to queue the messages here for some
            #       time...
            return

        self._protocol.sendInterfaceStatusUpdateMessage(iTag, status)

    sendInterfaceStatusUpdate.__doc__ = \
        IProtocol.get('sendInterfaceStatusUpdateMessage').getDoc()

    # Forwarding to View

    def createContainer(self, tag, data={}):
        if not self._view:
            raise ForwardingError('Reference of the view is missing.')

        self._view.createContainer(tag, data)

    createContainer.__doc__ = IRobot.get('createContainer').getDoc()

    def destroyContainer(self, tag):
        if not self._view:
            raise ForwardingError('Reference of the view is missing.')

        self._view.destroyContainer(tag)

    destroyContainer.__doc__ = IRobot.get('destroyContainer').getDoc()

    def addNode(self, cTag, nTag, pkg, exe, args='', name='', namespace=''):
        if not self._view:
            raise ForwardingError('Reference of the view is missing.')

        self._view.addNode(cTag, nTag, pkg, exe, args, name, namespace)

    addNode.__doc__ = IRobot.get('addNode').getDoc()

    def removeNode(self, cTag, nTag):
        if not self._view:
            raise ForwardingError('Reference of the view is missing.')

        self._view.removeNode(cTag, nTag)

    removeNode.__doc__ = IRobot.get('removeNode').getDoc()

    def addInterface(self, eTag, iTag, iType, clsName, addr=''):
        if not self._view:
            raise ForwardingError('Reference of the view is missing.')

        self._view.addInterface(eTag, iTag, iType, clsName, addr)

    addInterface.__doc__ = IRobot.get('addInterface').getDoc()

    def removeInterface(self, eTag, iTag):
        if not self._view:
            raise ForwardingError('Reference of the view is missing.')

        self._view.removeInterface(eTag, iTag)

    removeInterface.__doc__ = IRobot.get('removeInterface').getDoc()

    def addParameter(self, cTag, name, value):
        if not self._view:
            raise ForwardingError('Reference of the view is missing.')

        self._view.addParameter(cTag, name, value)

    addParameter.__doc__ = IRobot.get('addParameter').getDoc()

    def removeParameter(self, cTag, name):
        if not self._view:
            raise ForwardingError('Reference of the view is missing.')

        self._view.removeParameter(cTag, name)

    removeParameter.__doc__ = IRobot.get('removeParameter').getDoc()

    def addConnection(self, tagA, tagB):
        if not self._view:
            raise ForwardingError('Reference of the view is missing.')

        self._view.addConnection(tagA, tagB)

    addConnection.__doc__ = IRobot.get('addConnection').getDoc()

    def removeConnection(self, tagA, tagB):
        if not self._view:
            raise ForwardingError('Reference of the view is missing.')

        self._view.removeConnection(tagA, tagB)

    removeConnection.__doc__ = IRobot.get('removeConnection').getDoc()

    # Forwarding to Namespace

    def processReceivedMessage(self, iTag, clsName, msgID, msg):
        if not self._namespace:
            raise ForwardingError('Reference of the namespace is missing.')

        self._namespace.receivedFromClient(iTag, clsName, msgID, msg)

    processReceivedMessage.__doc__ = \
        IMessageReceiver.get('processReceivedMessage').getDoc()


class RobotView(object):
    """ Wrapper for a RemoteReference of type RobotView.
    """
    implements(IRobot)

    def __init__(self, view, connection):
        """ Initialize the wrapper.

            @param view:        Remote reference referencing the RobotView
                                object in the Master process.
            @type  view:        twisted.spread.pb.RemoteReference

            @param connection:  Representation of the connection to a robot
                                client for which the wrapped RobotView was
                                retrieved from the Master process.
            @type  connection:  rce.robot.Connection
        """
        self._view = view
        self._connection = connection

    def _reportError(self, failure):
        """ Method is used internally as an errback to send an error message to
            the robot client.
        """
        self._connection.reportError(failure.getErrorMessage())

    def createContainer(self, tag, data={}):
        try:
            d = self._view.callRemote('createContainer', tag, data)
        except (DeadReferenceError, PBConnectionLost):
            raise DeadConnection

        d.addErrback(self._reportError)

    createContainer.__doc__ = IRobot.get('createContainer').getDoc()

    def destroyContainer(self, tag):
        try:
            d = self._view.callRemote('destroyContainer', tag)
        except (DeadReferenceError, PBConnectionLost):
            raise DeadConnection

        d.addErrback(self._reportError)

    destroyContainer.__doc__ = IRobot.get('destroyContainer').getDoc()

    def addNode(self, cTag, nTag, pkg, exe, args='', name='', namespace=''):
        try:
            d = self._view.callRemote('addNode', cTag, nTag, pkg, exe, args,
                                      name, namespace)
        except (DeadReferenceError, PBConnectionLost):
            raise DeadConnection

        d.addErrback(self._reportError)

    addNode.__doc__ = IRobot.get('addNode').getDoc()

    def removeNode(self, cTag, nTag):
        try:
            d = self._view.callRemote('removeNode', cTag, nTag)
        except (DeadReferenceError, PBConnectionLost):
            raise DeadConnection

        d.addErrback(self._reportError)

    removeNode.__doc__ = IRobot.get('removeNode').getDoc()

    def addInterface(self, eTag, iTag, iType, clsName, addr=''):
        try:
            d = self._view.callRemote('addInterface', eTag, iTag, iType,
                                      clsName, addr)
        except (DeadReferenceError, PBConnectionLost):
            raise DeadConnection

        d.addErrback(self._reportError)

    addInterface.__doc__ = IRobot.get('addInterface').getDoc()

    def removeInterface(self, eTag, iTag):
        try:
            d = self._view.callRemote('removeInterface', eTag, iTag)
        except (DeadReferenceError, PBConnectionLost):
            raise DeadConnection

        d.addErrback(self._reportError)

    removeInterface.__doc__ = IRobot.get('removeInterface').getDoc()

    def addParameter(self, cTag, name, value):
        try:
            d = self._view.callRemote('addParameter', cTag, name, value)
        except (DeadReferenceError, PBConnectionLost):
            raise DeadConnection

        d.addErrback(self._reportError)

    addParameter.__doc__ = IRobot.get('addParameter').getDoc()

    def removeParameter(self, cTag, name):
        try:
            d = self._view.callRemote('removeParameter', cTag, name)
        except (DeadReferenceError, PBConnectionLost):
            raise DeadConnection

        d.addErrback(self._reportError)

    removeParameter.__doc__ = IRobot.get('removeParameter').getDoc()

    def addConnection(self, tagA, tagB):
        try:
            d = self._view.callRemote('addConnection', tagA, tagB)
        except (DeadReferenceError, PBConnectionLost):
            raise DeadConnection

        d.addErrback(self._reportError)

    addConnection.__doc__ = IRobot.get('addConnection').getDoc()

    def removeConnection(self, tagA, tagB):
        try:
            d = self._view.callRemote('removeConnection', tagA, tagB)
        except (DeadReferenceError, PBConnectionLost):
            raise DeadConnection

        d.addErrback(self._reportError)

    removeConnection.__doc__ = IRobot.get('removeConnection').getDoc()

    def destroy(self):
        """ # TODO: Add doc
        """
        self._connection = None
        self._view = None


class Robot(Namespace):
    """ Representation of a namespace in the robot process, which is part of
        the cloud engine internal communication.
    """
    def __init__(self, endpoint, connection):
        """ Initialize the Robot.

            @param endpoint:    Robot Client which is responsible for
                                monitoring the robots in this process.
            @type  endpoint:    rce.robot.RobotClient

            @param connection:  The connection manager for robot namespaces.
            @type  connection:  rce.robot.Connection
        """
        Namespace.__init__(self, endpoint)

        interface_map = {
            Types.encode('PublisherConverter') : PublisherConverter,
            Types.encode('SubscriberConverter') : SubscriberConverter,
            Types.encode('ServiceClientConverter') : ServiceClientConverter,
            Types.encode('ServiceProviderConverter') : ServiceProviderConverter,
            Types.encode('PublisherForwarder') : PublisherForwarder,
            Types.encode('SubscriberForwarder') : SubscriberForwarder,
            Types.encode('ServiceClientForwarder') : ServiceClientForwarder,
            Types.encode('ServiceProviderForwarder') : ServiceProviderForwarder
        }
        self._map.update(interface_map)

        self._connection = connection

    @property
    def converter(self):
        """ Reference to the message converter used by the Converter
            interfaces.
        """
        return self._endpoint.converter

    def receivedFromClient(self, iTag, clsName, msgID, msg):
        """ Process a data message which has been received from the robot
            client and send the message to the appropriate interface.

            @param iTag:        Tag which is used to identify the interface to
                                which this message should be sent.
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
        # TODO: What should we do, if the interface exists, but there are no
        #       connections?
        #       For now the message is just dropped, which is fatal if it is a
        #       service call, i.e. the caller will wait forever for a response
        try:
            self._interfaces[iTag].receive(clsName, msgID, msg)
        except (DeadReferenceError, PBConnectionLost):
            raise DeadConnection

    def sendToClient(self, iTag, msgType, msgID, msg):
        """ Process a data message which has been received from an interface
            send the message to the registered connection.

            @param iTag:        Tag which is used to identify the interface
                                from which this message was sent.
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
        if not self._connection:
            # It is possible that the connection is already lost when a data
            # message is sent, e.g. when the client just disconnects without
            # properly removing the interfaces, then the interfaces are only
            # removed after given TIMEOUT, but the connection will already be
            # gone
            # TODO: What is the proper reaction to a missing connection, i.e.
            #       once reconnecting clients are available... ?
            return

        self._connection.sendMessage(iTag, msgType, msgID, msg)

    def sendToClientInterfaceStatusUpdate(self, iTag, status):
        """ Send a status change which should be used to start or stop the
            corresponding interface on the client-side to the registered
            connection.

            @param iTag:        Tag which is used to identify the interface
                                which changed its status.
            @type  iTag:        str

            @param status:      Boolean indicating whether the interface should
                                be active or not.
            @type  status:      bool
        """
        if not self._connection:
            # It is possible that the connection is already lost when a status
            # message is sent, e.g. when the client just disconnects without
            # properly removing the interfaces, then the interfaces are only
            # removed after given TIMEOUT, but the connection will already be
            # gone
            # TODO: What is the proper reaction to a missing connection, i.e.
            #       once reconnecting clients are available... ?
            return

        self._connection.sendInterfaceStatusUpdate(iTag, status)

    def destroy(self):
        """ # TODO: Add doc
        """
        self._connection = None
        Namespace.remote_destroy(self)

    def remote_destroy(self):
        """ Method should be called to destroy the robot and will take care
            of destroying all objects owned by this robot as well as
            deleting all circular references.
        """
        if self._connection:
            self._connection.destroy()

        self.destroy()


class RobotClient(Endpoint):
    """ Realm for the connections to the robots. It is responsible for storing
        all connections.
    """
    implements(IRobotRealm)

    # CONFIG
    CONNECT_TIMEOUT = 30
    RECONNECT_TIMEOUT = 10

    def __init__(self, reactor, masterIP, masterPort, commPort, extIP, extPort,
                 loader, converter):
        """ Initialize the Robot Client.

            @param reactor:     Reference to the twisted reactor used in this
                                robot process.
            @type  reactor:     twisted::reactor

            @param masterIP:    IP address of the Master process.
            @type  masterIP:    str

            @param masterPort:  Port which should be used to authenticate the
                                user with the Master process.
            @type  masterPort:  int

            @param commPort:    Port where the server for the cloud engine
                                internal communication is listening for
                                incoming connections.
            @type  commPort:    int

            @param extIP:       IP address of network interface used for the
                                external communication.
            @type  extIP:       str

            @param extPort:     Port where the server for the external
                                communication is listening for WebSocket
                                connections.
            @type  extPort:     int

            @param loader:      Object which is used to load Python modules
                                from ROS packages.
            @type  loader:      rce.util.loader.Loader

            @param converter:   Converter which takes care of converting the
                                messages from JSON to ROS message and vice
                                versa.
            @type  converter:   rce.util.converter.Converter
        """
        Endpoint.__init__(self, reactor, loader, commPort)

        self._masterIP = masterIP
        self._masterPort = masterPort
        self._extAddress = '{0}:{1}'.format(extIP, extPort)
        self._loader = loader
        self._converter = converter

        self._connections = set()
        self._deathCandidates = {}

    @property
    def converter(self):
        """ Reference to the message converter used by the Converter
            interfaces.
        """
        return self._converter

    def registerConnection(self, connection):
        assert connection not in self._connections
        self._connections.add(connection)

        # Add the connection also to the death candidates
        assert connection not in self._deathCandidates
        deathCall = self._reactor.callLater(self.CONNECT_TIMEOUT,
                                            self._killConnection, connection)
        self._deathCandidates[connection] = deathCall


    def unregisterConnection(self, connection):
        assert connection in self._connections

        # First remove the connection from the death candidates if necessary
        deathCall = self._deathCandidates.pop(connection, None)
        if deathCall and deathCall.active():
            deathCall.cancel()

        # Unregister the candidates
        self._connections.remove(connection)

    def _killConnection(self, connection):
        """ Internally used method to destroy a connection whose reconnect
            timeout was reached or which never as successfully connected.

            @param connection:  Connection which should be destroyed.
            @type  connection:  rce.robot.Connection
        """
        assert connection in self._connections
        assert connection in self._deathCandidates

        deathCall = self._deathCandidates.pop(connection)
        if deathCall.active():
            deathCall.cancel()

        connection.destroy()

    def _cbAuthenticated(self, avatar, connection):
        """ Method is used internally as a callback which is called when the
            user of a newly connected robot has been successfully authenticated
            by the Master process.

            @param avatar:      User avatar returned by the Master process upon
                                successful login and authentication.
            @type  avatar:      twisted.spread.pb.RemoteReference

            @param connection:  Representation of the connection to the robot
                                which is used in the Robot process.
            @type  connection:  rce.robot.Connection
        """
        connection.registerAvatar(avatar)
        return avatar.callRemote('getUserView', False)

    def _cbConnected(self, view, connection):
        """ Method is used internally as a callback which is called when the
            Robot view has been successfully retrieved from the Master process
            for the user of a newly connected robot.

            @param view:        Robot view returned by the Master process.
            @type  view:        twisted.spread.pb.RemoteReference

            @param connection:  Representation of the connection to the robot
                                which is used in the Robot process.
            @type  connection:  rce.robot.Connection
        """
        if not self._avatar:  # This is RobotEndpointAvatar and not User Avatar.
            raise ForwardingError('Avatar reference is missing.')

        view = RobotView(view, connection)
        namespace = Robot(self, connection)
        connection.registerView(view)
        connection.registerNamespace(namespace)
        return self._avatar.callRemote('setupNamespace', namespace,
                                       connection.userID, connection.robotID)

    def login(self, userID, robotID, password):
        """ Callback for Robot connection to login and authenticate.

            @param userID:      User ID under which the robot is logging in.
            @type  userID:      str

            @param robotID:     Unique ID of the robot in the namespace of the
                                user under which the robot is logging in.
            @type  robotID:     str

            @param password:    Hashed password as hex-encoded string which is
                                used to authenticate the user.
            @type  password:    str

            @return:            Representation of the connection to the robot
                                which is used in the Robot process.
                                (type: rce.robot.Connection)
            @rtype:             twisted.internet.defer.Deferred
        """
        conn = Connection(self, userID, robotID)

        factory = PBClientFactory()
        self._reactor.connectTCP(self._masterIP, self._masterPort, factory)

        d = factory.login(UsernamePassword(userID, password))
        d.addCallback(self._cbAuthenticated, conn)
        d.addCallback(self._cbConnected, conn)
        d.addCallback(lambda _: conn)
        return d

    def registerWebsocketProtocol(self, connection, protocol):
        """ Register the client protocol with a Connection object.

            @param connection:  Connection where the protocol should be
                                registered.
            @type  connection:  rce.robot.Connection

            @param protocol:    Protocol which should be registered.
            @type  protocol:    rce.comm.interfaces.IServersideProtocol
        """
        assert connection in self._deathCandidates
        connection.registerProtocol(protocol)
        self._deathCandidates.pop(connection).cancel()

    def unregisterWebsocketProtocol(self, connection, protocol):
        """ Unregister the client protocol from a Connection object.

            @param connection:  Connection where the protocol should be
                                unregistered.
            @type  connection:  rce.robot.Connection

            @param protocol:    Protocol which should be unregistered.
            @type  protocol:    rce.comm.interfaces.IServersideProtocol
        """
        assert connection not in self._deathCandidates
        deathCall = self._reactor.callLater(self.RECONNECT_TIMEOUT,
                                            self._killConnection, connection)
        self._deathCandidates[connection] = deathCall

        connection.unregisterProtocol(protocol)

    def remote_getWebsocketAddress(self):
        """ Get the address of the WebSocket server running in this process.

            @return:            Address which can be used to connect to the
                                cloud engine using a WebSocket connection. The
                                address has the form [IP]:[port]
            @rtype:             str
        """
        return self._extAddress

    def terminate(self):
        """ Method should be called to terminate the client before the reactor
            is stopped.

            @return:            Deferred which fires as soon as the client is
                                ready to stop the reactor.
            @rtype:             twisted.internet.defer.Deferred
        """
        for call in self._deathCandidates.itervalues():
            call.cancel()

        self._deathCandidates = {}

        for connection in self._connections.copy():
            connection.destroy()
        assert len(self._connections) == 0

        Endpoint.terminate(self)


def main(reactor, cred, masterIP, masterPort, consolePort,
                extIP, extPort, commPort, pkgPath, customConverters):
    log.startLogging(sys.stdout)

    def _err(reason):
        print(reason)
        reactor.stop()

    factory = PBClientFactory()
    reactor.connectTCP(masterIP, masterPort, factory)

    rosPath = []
    for path in get_ros_paths() + [p for p, _ in pkgPath]:
        if path not in rosPath:
            rosPath.append(path)

    loader = Loader(rosPath)
    converter = Converter(loader)

    for customConverter in customConverters:
        # Get correct path/name of the converter
        module, className = customConverter.rsplit('.', 1)

        # Load the converter
        mod = __import__(module, fromlist=[className])
        converter.addCustomConverter(getattr(mod, className))

    client = RobotClient(reactor, masterIP, consolePort, commPort, extIP,
                         extPort, loader, converter)
    d = factory.login(cred, client)
    d.addCallback(lambda ref: setattr(client, '_avatar', ref))
    d.addErrback(_err)

    # portal = Portal(client, (client,))
    robot = CloudEngineWebSocketFactory(client,
                                        'ws://localhost:{0}'.format(extPort))
    listenWS(robot)

    reactor.addSystemEventTrigger('before', 'shutdown', client.terminate)
    reactor.run()
