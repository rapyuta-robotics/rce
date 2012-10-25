#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     manager.py
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
import os
from datetime import datetime, timedelta
from uuid import uuid4
from functools import wraps

# zope specific imports
from zope.interface import implements

# twisted specific imports
from twisted.python import log
from twisted.internet.defer import Deferred, DeferredList
from twisted.internet.task import LoopingCall

# Custom imports
from errors import InternalError, InvalidRequest, UserConstraintError, \
    AuthenticationError
from util.interfaces import verifyObject
from util.loader import Loader
from util.converter import Converter
from util import path as pathUtil
from core.types import req
from core.interfaces import IRequestSender, IControlFactory, IMessenger, \
    ILoadBalancer
from core.user import User
from core.monitor import NodeMonitor
from core.container import DeploymentContainer
from comm import definition
from comm import types as msgTypes
from comm.protocol import RCEClientFactory
from comm.message import Message
from remote.control import RemoteNodeControl
from remote.callback import RelayCallbackFromRelay
from client import types as clientTypes


class _ManagerBase(object):
    """ Base class for all node managers.
    """
    def __init__(self, reactor):
        """ Initialize the basic fields.
            
            @param reactor:     twisted::reactor which should be used with this
                                manager.
        """
        self._reactor = reactor
        self._commManager = None
    
    def registerCommManager(self, commManager):
        """ Register the CommManager.
            
            @param commManager:    CommManager which should be registered.
            @type  commManager:    comm.manager.CommManager
        """
        if self._commManager:
            raise InternalError('There is already a CommManager registered.')
        
        self._commManager = commManager
    
    @property
    def reactor(self):
        """ Reference to twisted reactor of this node. """
        return self._reactor
    
    def shutdown(self):
        pass


class _UserManagerBase(_ManagerBase):
    """ Base class for all node managers which need a user struct.
    """
    def __init__(self, reactor):
        super(_UserManagerBase, self).__init__(reactor)
        self._users = {}
    
    def shutdown(self):
        """ Method should be called when the manager is stopped.
        """
        self._users = {}
        
        super(_UserManagerBase, self).shutdown()
    
    @staticmethod
    def verifySingleUser(f):
        """ Decorator which checks that there is at most one active user in the
            manager. Additionally, if the user does not exist he will be
            created.
            
            This decorator requires a reference to the class which is used to
            represent a user in the manager in the class attribute _USER_CLS.
            
            @param f:       Method which should be decorated. It has to be a
                            handler method which has the following arguments:
                                self, userID, *args, **kw
        """
        @wraps(f)
        def handler(self, userID, *args, **kw):
            if userID not in self._users:
                if self._users:
                    raise UserConstraintError('More than one user active.')
                
                self._users[userID] = self._USER_CLS()
            
            return f(self, userID, *args, **kw)
        
        return handler
    
    @staticmethod
    def verifyUser(f):
        """ Decorator which checks that the user does exist, otherwise he will
            be created.
            
            This decorator requires a reference to the class which is used to
            represent a user in the manager in the class attribute _USER_CLS.
            
            @param f:       Method which should be decorated. It has to be a
                            handler method which has the following arguments:
                                self, userID, *args, **kw
        """
        @wraps(f)
        def handler(self, userID, *args, **kw):
            if userID not in self._users:
                self._users[userID] = self._USER_CLS()
            
            return f(self, userID, *args, **kw)
        
        return handler


class _ROSManagerBase(_UserManagerBase):
    """ Base class for managers which handle ROS specific tasks.
    """
    _ROS_PACKAGE_PATH = None
    
    def __init__(self, reactor):
        super(_ROSManagerBase, self).__init__(reactor)
        
        # References used by the manager
        self._loader = Loader(self._ROS_PACKAGE_PATH)
    
    @property
    def loader(self):
        """ Loader for ROS resources. """
        return self._loader


class NodeManager(_ROSManagerBase):
    """ Manager which handles the management of nodes.
    """
    def __init__(self, reactor):
        super(NodeManager, self).__init__(reactor)
        
    @_UserManagerBase.verifySingleUser
    def addNode(self, userID, node):
        """ Add a Node to the ROS environment.
            
            @param node:    Node instance containing the necessary launch
                            information which should be added.
            @type  node:    core.command.NodeCommand
        """
        if node.tag in self._users[userID].nodes:
            raise InvalidRequest('Node already exists.')
        
        nodeMonitor = NodeMonitor(self, userID, node)
        nodeMonitor.start()
    
    @_UserManagerBase.verifySingleUser
    def removeNode(self, userID, tag):
        """ Remove a Node from the ROS environment.
            
            @param tag:     Tag which is used to identify the node which should
                            be removed.
            @type  tag:     str
        """
        try:
            self._users[userID].nodes[tag].stop()
        except KeyError:
            raise InvalidRequest('Node does not exist.')
    
    @_UserManagerBase.verifySingleUser
    def registerNode(self, userID, node):
        """ Callback for NodeMonitor to register the node.
        """
        nodes = self._users[userID].nodes
        
        if node.tag in nodes:
            raise InvalidRequest('Node already exists.')
        
        nodes[node.tag] = node
    
    @_UserManagerBase.verifySingleUser
    def unregisterNode(self, userID, node):
        """ Callback for NodeMonitor to unregister the node.
        """
        try:
            del self._users[userID].nodes[node.tag]
        except KeyError:
            raise InvalidRequest('Node does not exist.')
    
    def shutdown(self):
        """ Method is called when the manager is stopped.
        """
        for user in self._users.itervalues():
            for node in user.nodes.itervalues():
                node.stop()
        
        super(NodeManager, self).shutdown()
        ### TODO: Can't wait for termination at the moment
        ###       Need something similar to initSlave.py
#        if self._nodes:
#            deferreds = []
#            
#            for nodes in self._nodes.itervalues():
#                deferred = Deferred()
#                deferreds.append(deferred)
#                nodes.stop()
#            
#            deferredList = DeferredList(deferreds)
#            return deferredList


class NodeFwdManager(_UserManagerBase):
    """ Manager which handles the management of nodes.
    """
    def __init__(self, reactor):
        super(NodeFwdManager, self).__init__(reactor)
    
    @_UserManagerBase.verifySingleUser
    def addNode(self, userID, node):
        """ Add a Node to the ROS environment.
            
            @param node:    Node instance containing the necessary launch
                            information which should be added.
            @type  node:    core.command.NodeForwarderCommand
        """
        user = self._users[userID]
        
        if not user.nodes:
            user.nodes = RemoteNodeControl(userID, self._NODE_FWD_ID,
                                           self._commManager)
        
        user.nodes.addNode(node)
    
    @_UserManagerBase.verifySingleUser
    def removeNode(self, userID, tag):
        """ Remove a Node from the ROS environment.
            
            @param tag:     Tag which is used to identify the node which should
                            be removed.
            @type  tag:     str
        """
        user = self._users[userID]
        
        if not user.nodes:
            user.nodes = RemoteNodeControl(userID, self._NODE_FWD_ID,
                                           self._commManager)
        
        user.nodes.removeNode(tag)


class ParameterManager(_ROSManagerBase):
    """ Manager which handles the management of parameters.
    """
    @_UserManagerBase.verifySingleUser
    def addParameter(self, userID, parameter):
        """ Add a parameter. If the parameter already exists, the old will be
            removed and the new one kept.

            @param parameter:   Parameter which should be added.
            @type  parameter:   core.command._ParameterCommand
        """
        parameters = self._users[userID].parameters
        
        try:
            parameter = self._PARAMETERS[parameter.IDENTIFIER](parameter)
        except KeyError:
            raise InvalidRequest('ParameterManager can not manage '
                                 'requested component.')
        
        name = parameter.name
        log.msg('Register parameter "{0}".'.format(name))
        
        if name in parameters:
            parameters[name].remove()
        
        parameters[name] = parameter
    
    @_UserManagerBase.verifySingleUser
    def removeParameter(self, userID, name):
        """ Remove a parameter.
            
            @param name:    Name which is used to identify the parameter which
                            should be removed.
            @type  name:    str
        """
        log.msg('Remove parameter "{0}".'.format(name))
        
        try:
            self._users[userID].pop(name).remove()
        except KeyError:
            pass

    def shutdown(self):
        for user in self._users.itervalues():
            for parameter in user.parameters.itervalues():
                parameter.remove()
            
            user.parameters = {}
        
        super(ParameterManager, self).shutdown()


class _InterfaceManager(_ROSManagerBase):
    """ Manager which handles the management of interfaces.
    """
    def __init__(self, reactor):
        super(_InterfaceManager, self).__init__(reactor)
        
        self._messenger = None
    
    def registerMessenger(self, messenger):
        """ Register the Messenger.
            
            @param messenger:   Messenger which should be registered.
            @type  messenger:   core.interfaces.IMessenger
        """
        verifyObject(IMessenger, messenger)
        
        if self._messenger:
            raise InternalError('There is already a messenger registered.')
        
        self._messenger = messenger
    
    def addInterface(self, userID, interface):
        """ Add an interface.

            @param interface:   Interface which should be added.
            @type  interface:   core.command._EndpointInterfaceCommand

            @raise:     errors.InternalError if there is already an interface
                        with the same tag registered.
        """
        interfaces = self._users[userID].interfaces
        
        try:
            monitor = self._INTERFACES[interface.IDENTIFIER](self, userID,
                                                             interface)
        except KeyError:
            raise InvalidRequest('InterfaceManager can not manage '
                                 'requested component.')
        
        tag = monitor.tag

        if tag in interfaces:
            raise InternalError('There is already an interface with '
                                'the same tag.')
        
        log.msg('Register new interface "{0}".'.format(tag))
        monitor.start()
        interfaces[tag] = monitor
    
    def removeInterface(self, userID, tag):
        """ Remove an interface.
            
            @param tag:     Tag which is used to identify the interface which
                            should be removed.
            @type  tag:     str
        """
        log.msg('Remove interface "{0}".'.format(tag))
        
        try:
            self._users[userID].interfaces.pop(tag).stop()
        except KeyError:
            pass
    
    def _getInterface(self, userID, tag):
        """ Internally used method to get an interface safely.
            
            @raise:     errors.InternalError if there is no interface matching
                        the given tag.
        """
        interfaces = self._users[userID].interfaces
        
        if tag not in interfaces:
            raise InternalError('There is no interface with the tag '
                                '"{0}".'.format(tag))
        
        return interfaces[tag]
    
    def modifyConnection(self, userID, conn):
        """ Modify a connection to another interface, i.e. either add one or
            remove an existing one.
            
            @param conn:    Connection description which should be used for
                            this operation.
            @type  conn:    core.command.ConnectionCommand
        """
        tag = conn.tag
        commID = conn.commID
        target = conn.target
        
        interface = self._getInterface(userID, tag)
        
        if conn.add:
            log.msg('Register connection "({0}, {1})" for interface '
                    '"{2}" (user: "{3}").'.format(commID, target, tag, userID))
            interface.addConnection(commID, conn.target)
        else:
            log.msg('Unregister connection "({0}, {1})" for interface '
                    '"{2}" (user: "{3}").'.format(commID, target, tag, userID))
            self._getInterface(userID, tag).removeConnection(commID, target)
    
    def send(self, userID, tag, commID, senderTag, msg, msgID):
        """ Send a message which was received from an endpoint to another
            interface.
            
            @param userID:      User ID of the interface owner.
            @type  userID:      str
            
            @param tag:         Tag of the interface at the destination.
            @type  tag:         str
            
            @param commID:      Communication ID of the destination.
            @type  commID:      str
            
            @param senderTag:   Tag of the interface who is sending the
                                message.
            @type  senderTag:   str
            
            @param msg:         ROS message in serialized form which should be
                                sent.
            @type  msg:         str
            
            @param msgID:       Identifier which is used to match a request to
                                a response.
            @type  msgID:       str
        """
        if not self._messenger:
            raise InternalError('There is no messenger registered.')
        
        self._messenger.send(userID, tag, commID, senderTag, msg, msgID)
    
    def received(self, userID, tag, commID, senderTag, msg, msgID):
        """ Send a message which was received from another endpoint to a local
            interface.
            
            @param userID:      User ID of the interface owner.
            @type  userID:      str
            
            @param tag:         Tag of the interface to which the message is
                                sent.
            @type  tag:         str
            
            @param commID:      Communication ID of the origin.
            @type  commID:      str
            
            @param senderTag:   Tag of the interface which sent the message.
            @type  senderTag:   str
            
            @param msg:         ROS message in serialized form which was sent.
            @type  msg:         str
            
            @param msgID:       Identifier which is used to match a request to
                                a response.
            @type  msgID:       str
        """
        self._getInterface(userID, tag).send(msg, msgID, commID, senderTag)

    def shutdown(self):
        for user in self._users.itervalues():
            for interface in user.interfaces.itervalues():
                interface.stop()
            
            user.interfaces = {}
        
        super(_InterfaceManager, self).shutdown()
        
        self._messenger = None


class ROSInterfaceOnlyManager(_InterfaceManager):
    """ Manager which handles the management of ROS interfaces. It should be
        used if only ROS interfaces are occurring in the manager.
    """
    addInterface = _UserManagerBase.verifySingleUser(
        _InterfaceManager.addInterface)
    removeInterface = _UserManagerBase.verifySingleUser(
        _InterfaceManager.removeInterface)
    _getInterface = _UserManagerBase.verifySingleUser(
        _InterfaceManager._getInterface)


class RobotManager(_InterfaceManager):
    """ Manager which handles the management of robots and converters.
    """
    class _Robot(object):
        """ Class which represents a robot before he actually connects.
        """
        def __init__(self, key):
            self.key = key
            self.timestamp = datetime.now()
            self.conn = None
    
    def __init__(self, reactor):
        super(RobotManager, self).__init__(reactor)
        
        self._converter = Converter(self._loader)
        self._reqSender = None
        
        # Add all custom converters to the Converter
        for converter in self._CUSTOM_CONVERTERS:
            # Get correct path/name of the converter
            convList = converter.rsplit('.', 1)
            module = convList[0]
            className = convList[1]
    
            # Load the converter
            mod = __import__(module, fromlist=[className])
            self._converter.addCustomConverter(getattr(mod, className))
        
        self.__cleaner = LoopingCall(self.__clean)
        self.__cleaner.start(self._ROBOT_TIMEOUT/2)
    
    @property
    def converter(self):
        """ ROS Message <-> JSON Message Converter. """
        return self._converter
    
    def registerRequestSender(self, sender):
        """ Register the RequestSender.
            
            @param sender:      RequestSender which should be registered.
            @type  sender:      core.interfaces.IRequestSender
        """
        if self._reqSender:
            raise InternalError('There is already a RequestSender registered.')
        
        verifyObject(IRequestSender, sender)
        
        self._reqSender = sender
    
    @_UserManagerBase.verifyUser
    def addRobot(self, userID, robot):
        """ # TODO: Add description
        """
        robotID = robot.robotID
        robots = self._users[userID].robots
        
        if robotID in robots:
            raise InternalError('There already exists a robot with the robot '
                                'ID "{0}".'.format(robotID))
        
        robots[robotID] = RobotManager._Robot(robot.key)
    
    @_UserManagerBase.verifyUser
    def removeRobot(self, userID, tag):
        """ # TODO: Add description
        """
        robots = self._users[userID].robots
        
        if tag not in robots:
            raise InternalError('Tried to remove a robot which does not '
                                'exist.')
        
        if robots[tag].conn:
            raise InternalError('Connection to robot is not closed yet.')
        
        del robots[tag]
    
    @_UserManagerBase.verifyUser
    def robotConnected(self, userID, robotID, key, conn):
        """ # TODO: Add description
                (Called by client.protocol.RobotWebSocketProtocol
                 during initialization/verification of the connection)
        """
        robots = self._users[userID].robots
        
        if robotID not in robots:
            raise AuthenticationError('There is no connection expected from '
                                      'a robot with robotID '
                                      '"{0}".'.format(robotID))
        
        robot = robots[robotID]
        
        if robot.conn:
            raise AuthenticationError('There is already a robot with the same '
                                      'robotID registered.')
        
        if not key == robot.key:
            raise AuthenticationError('The key is not correct.')
        
        robot.timestamp = None
        robot.conn = conn
    
    @_UserManagerBase.verifyUser
    def robotClosed(self, userID, robotID):
        """ # TODO: Add description
                (Called by client.protocol.RobotWebSocketProtocol
                 in onClose)
        """
        robots = self._users[userID].robots
        
        if robotID in robots and robots[robotID]:
            robot = robots[robotID]
            robot.timestamp = datetime.now()
            robot.conn = None
    
    def sendRequest(self, request):
        """ Send a request received from the robot to the master manager for
            processing.
                (Called by client.handler.*)
            
            @param request:     Request which should be sent to the master
                                manager.
            @type  request:     { 'user' : str, 'type' : str, 'args' : tuple }
        """
        self._reqSender.processRequest(request)
    
    @_UserManagerBase.verifyUser
    def receivedFromClient(self, userID, robotID, iTag, msgType, msgID, msg):
        """ Received a message from a client and should now be processed.
                (Called by client.handler.DataMessageHandler)
            
            @param userID:      User ID of the converter owner.
            @type  userID:      str
            
            @param robotID:     Robot ID of the robot for which this converter
                                works.
            @type  robotID:     str
            
            @param iTag:        Tag of the converter to which this message
                                should sent.
            @type  iTag:        str
            
            @param msgType:     Message type of the received message.
            @type  msgType:     str
            
            @param msgID:       Identifier which is used to match request /
                                response message.
            @type  msgID:       str
            
            @param msg:         JSON compatible message which should be sent to
                                the client/robot.
            @type  msg:         dict
        """
        # TODO: robotID at the moment not used...
        self._getInterface(userID, iTag).receive(msgType, msgID, msg)
    
    @_UserManagerBase.verifyUser
    def sendToClient(self, userID, robotID, msg):
        """ Send a message to a client.
                (Called by core.monitor._ConverterMonitor)
            
            @param userID:      User ID of the converter owner.
            @type  userID:      str
            
            @param robotID:     Robot ID of the robot for which this converter
                                works.
            @type  robotID:     str
            
            @param msg:         JSON compatible message which should be sent to
                                the client/robot.
            @type  msg:         dict
        """
        try:
            self._users[userID].robots[robotID].conn.sendMessage(
                {'data' : msg, 'type' : clientTypes.DATA_MESSAGE})
        except KeyError:
            log.msg('Message could not be sent: The robot "{0}" is no longer '
                    'connected.'.format(robotID))
        except AttributeError:
            raise InternalError('Tried to send a message to an unverified '
                                'robot.')
    
    def shutdown(self):
        self.__cleaner.stop()
        
        for user in self._users.itervalues():
            user.robots = {}
        
        super(RobotManager, self).shutdown()
        
        self._reqSender = None
    
    def __clean(self):
        """ Internally used method to remove unclaimed robot connections.
        """
        limit = datetime.now() - timedelta(seconds=self._ROBOT_TIMEOUT)
        
        for userID, user in self._users.iteritems():
            for robotID in [
                robotID for robotID, robot in user.robots.iteritems()
                    if robot.timestamp and robot.timestamp < limit]:
                self._reqSender.processRequest({'user' : userID,
                                                'type' : req.DESTROY_ROBOT,
                                                'args' : (robotID,) })
    
    addInterface = _UserManagerBase.verifyUser(
        _InterfaceManager.addInterface)
    removeInterface = _UserManagerBase.verifyUser(
        _InterfaceManager.removeInterface)
    _getInterface = _UserManagerBase.verifyUser(
        _InterfaceManager._getInterface)


class RelayManager(_ManagerBase):
    """ # TODO: Add description
    """
    def __init__(self, reactor):
        super(RelayManager, self).__init__(reactor)
        
        self._masterIP = None
        self._satellites = set()
        self._relays = set()
        
        self._updater = LoopingCall(self._updateLoadInfo)
        # self._updater.start(self.__LOAD_INFO_UPDATE)
    
    def registerMasterIP(self, ip):
        """ Register IP address of the machine where the master manager can
            be found.
        """
        self._masterIP = ip
    
    def _publishEndpointChange(self, info):
        """ Send a routing info/update to the registered relays and the master
            manager.
            
            @param info:    Information which should be distributed. Tuple
                            consisting commID of endpoint which changed and
                            a bool indicated whether the endpoint is added
                            (True) or removed (False).
            @type  info:    ( str, bool )
        """
        msg = Message()
        msg.msgType = msgTypes.ROUTE_INFO
        msg.dest = self._MASTER_ID
        msg.content = [info]
        self._commManager.sendMessage(msg)
        
        for relayID in self._relays:
            msg = Message()
            msg.msgType = msgTypes.ROUTE_INFO
            msg.dest = relayID
            msg.content = [info]
            self._commManager.sendMessage(msg)
    
    def registerEndpoint(self, commID):
        """ Register a new endpoint with this manager.
            
            @param commID:      CommID of new endpoint.
            @type  commID:      str
        """
        if commID in self._satellites:
            raise InternalError('Tried to register an endpoint, but '
                                'communication ID is already registered.')
        
        self._satellites.add(commID)
        self._publishEndpointChange((commID, True))
    
    def unregisterEndpoint(self, commID):
        """ Unregister an endpoint from this manager.
            
            @param commID:      CommID of endpoint.
            @type  commID:      str
        """
        if commID not in self._satellites:
            raise InternalError('Tried to unregister a satellite, but '
                                'communication ID is not registered.')
        
        self._satellites.remove(commID)
        self._publishEndpointChange((commID, False))
    
    def registerRelay(self, commID):
        """ Method is called by post initialization callback of relay-relay
            connections.
        """
        if commID in self._relays:
            raise InternalError('Tried to register same connection twice.')
        
        self._relays.add(commID)
    
    def unregisterRelay(self, commID):
        """ Method is called by post close callback of relay-relay connections.
        """
        if commID not in self._relays:
            raise InternalError('Tried to unregister a non-existent '
                                'connection.')
        
        self._relays.remove(commID)
    
    def getEndpointRouting(self):
        """ Returns the routing information for all nodes which should be
            routed through this node.
            
            @rtype:     (str)
        """
        return tuple(self._satellites)
    
    def connectToRelays(self, relays):
        """ Callback for remote.message.ConnectDirectiveProcessor to connect
            to the specified relays.
            
            @param relays:  List of (commID, IP address) tuples describing the
                            relays to which connections should be established.
            @type  relay:  [ (str, str) ]
        """
        for relay in relays:
            commID, ip = relay
            
            if commID in self._relays:
                continue  # We are already connected to this relay.
            
            # TODO: Small hack to circumvent problem with localhost IP
            if ip == '127.0.0.1':
                ip = self._masterIP
            
            cb = RelayCallbackFromRelay(self, self._commManager)
            factory = RCEClientFactory(self._commManager, commID, [cb], [cb])
            factory.addApprovedMessageTypes([msgTypes.ROS_MSG])
            self._reactor.connectTCP(ip, self._RELAY_PORT, factory)
    
    def _updateLoadInfo(self):
        """ This method is called regularly and is used to send the newest load
            info to the master manager/load balancer.
        """
        # TODO: Add code here for load info update
    
    def shutdown(self):
        # self._updater.stop()
        
        super(RelayManager, self).shutdown()


class ContainerManager(_UserManagerBase):
    """ Manager which handles the management of containers.
    """
    def __init__(self, reactor):
        """ Initialize the Container Manager.
            
            @raise:         ValueError, if the paths/names are invalid.
        """
        super(ContainerManager, self).__init__(reactor)
        
        self._relayID = None
        
        # Keys:    Container tag
        # Values:  bool (True = ok; False = delete again)
        self._pending = {}
        
        self._rootfs = self._ROOTFS
        self._confDir = self._CONF_DIR
        self._dataDir = self._DATA_DIR
        self._srcDir = self._ROOT_SRC_DIR
        
        # Validate directory paths
        pathUtil.checkPath(self._confDir, 'Configuration')
        pathUtil.checkPath(self._dataDir, 'Data')
        pathUtil.checkPath(self._rootfs, 'Container file system')
        pathUtil.checkPath(self._srcDir, 'RCE source')
        
        # Validate executable paths
        pathUtil.checkExe(self._srcDir, 'environment.py')
        pathUtil.checkExe(self._srcDir, 'launcher.py')
        
        # Process ROS package paths
        self._pkgDir = pathUtil.processPackagePaths(self._ROOT_PKG_DIR)
        
        for _, path in self._pkgDir:
            os.mkdir(os.path.join(self._rootfs, path))
    
    @property
    def relayID(self):
        """ Communication ID of associated relay manager.
        """
        if not self._relayID:
            raise InternalError('There is no relay ID registered.')
        
        return self._relayID
    
    @relayID.setter
    def relayID(self, relayID):
        """ Register the communication ID of the relay manager which is used
            in this machine.
        """
        if self._relayID:
            if self._relayID == relayID:
                return
            else:
                raise InternalError('There is already a relay ID registered.')
        
        if not definition.validateAddress(relayID):
            raise InvalidRequest('Relay ID "{0}" is not '
                                 'valid.'.format(relayID))
        
        self._relayID = relayID
    
    @property
    def rootfs(self):
        """ Host filesystem path of container filesystem root directory.
        """
        return self._rootfs
    
    @property
    def confDir(self):
        """ Filesystem path of configuration directory.
        """
        return self._confDir
    
    @property
    def dataDir(self):
        """ Filesystem path of temporary data directory.
        """
        return self._dataDir
    
    @property
    def srcDir(self):
        """ Filesystem path of RCE source directory.
        """
        return self._srcDir
    
    @property
    def pkgDirIter(self):
        """ Iterator over all file system paths of package directories.
        """
        return self._pkgDir.__iter__()
    
    def _startContainer(self, deferred, commID):
        """ Internally used method to start a container.
        """
        try:
            container = DeploymentContainer(self, commID)
            
            def cb(_):
                deferred.callback(container)
            
            def eb(m):
                deferred.errback(m)
            
            internal = Deferred()
            internal.addCallbacks(cb, eb)
        except Exception as e:
            log.msg('Caught an exception when trying to instantiate a '
                    'deployment container.')
            import traceback
            deferred.errback('\n'.join(traceback.format_exception_only(type(e),
                                                                       e)))
        else:
            container.start(internal)
    
    @_UserManagerBase.verifyUser
    def createContainer(self, userID, container):
        """ Callback for message processor to stop a container.
        """
        if not self._relayID:
            raise InternalError('Can not use the container manager before the '
                                'communication ID of the relay manager is '
                                'registered.')
        
        containers = self._users[userID].containers
        
        tag = container.tag
        commID = container.commID
        
        deferred = Deferred()
        
        def reportSuccess(contnr):
            if self._pending.pop(tag):
                containers[tag] = contnr
                log.msg('Container successfully started.')
            else:
                self.reactor.callInThread(self._stopContainer, Deferred(), 
                                          contnr)
        
        def reportFailure(msg):
            log.msg('Container could not be started: {0}.'.format(msg))
        
        deferred.addCallbacks(reportSuccess, reportFailure)
        
        if tag not in containers:
            self._pending[tag] = True
            self.reactor.callInThread(self._startContainer, deferred, commID)
        elif tag in self._pending:
            deferred.errback('There is already a pending container registered '
                             'under the same tag.')
        else:
            deferred.errback('There is already a container registered under '
                             'the same tag.')
    
    def _stopContainer(self, deferred, container):
        """ Internally used method to stop a container.
        """
        container.stop(deferred)
    
    @_UserManagerBase.verifyUser
    def destroyContainer(self, userID, tag):
        """ Callback for message processor to stop a container.
        """
        containers = self._users[userID].containers
        
        def reportSuccess(_):
            log.msg('Container successfully stopped.')
        
        def reportFailure(msg):
            log.msg('Container could not be stopped: {0}.'.format(msg))
        
        deferred = Deferred()
        deferred.addCallbacks(reportSuccess, reportFailure)
        
        if tag not in containers:
            if tag not in self._pending:
                deferred.errback('There is no container registered under '
                                 'this tag.')
            else:
                self._pending[tag] = False
        else:
            self.reactor.callInThread(self._stopContainer, deferred, 
                                      containers.pop(tag))
    
    def _cleanPackageDir(self, arg=None):
        for _, path in self._pkgDir:
            os.rmdir(os.path.join(self._rootfs, path))
        
        return arg
    
    def shutdown(self):
        deferreds = []
        
        for user in self._users.itervalues():
            for container in user.containers.itervalues():
                deferred = Deferred()
                deferreds.append(deferred)
                self._stopContainer(deferred, container)
        
        super(ContainerManager, self).shutdown()
        
        if deferreds:
            deferredList = DeferredList(deferreds)
            deferredList.addCallback(self._cleanPackageDir)
            return deferredList
        else:
            self._cleanPackageDir()


class MasterManager(_ManagerBase):
    """ Manager which is responsible for the Master node.
    """
    implements(IRequestSender)
    
    _REQUEST_MAP = {
        req.CREATE_CONTAINER :
            lambda user, args: user.createContainer(*args),
        req.DESTROY_CONTAINER :
            lambda user, args: user.destroyContainer(*args),
        req.DESTROY_ROBOT :
            lambda user, args: user.destroyRobot(*args),
        req.ADD_NODE :
            lambda user, args: user.addNode(*args),
        req.REMOVE_NODE :
            lambda user, args: user.removeNode(*args),
        req.ADD_PARAMETER :
            lambda user, args: user.addParameter(*args),
        req.REMOVE_PARAMETER :
            lambda user, args: user.removeParameter(*args),
        req.ADD_INTERFACE :
            lambda user, args: user.addInterface(*args),
        req.REMOVE_INTERFACE :
            lambda user, args: user.removeInterface(*args),
        req.ADD_CONNECTION :
            lambda user, args: user.createConnection(*args),
        req.REMOVE_CONNECTION :
            lambda user, args: user.destroyConnection(*args)
                    }
    
    def __init__(self, reactor):
        super(MasterManager, self).__init__(reactor)
        
        self.__users = {}
        
        # List of used commIDs
        self._commIDs = set()
    
    def registerLoadBalancer(self, loadBalancer):
        """ Register the LoadBalancer.
            
            @param loadBalancer:    LoadBalancer which should be registered.
            @type  loadBalancer:    core.interfaces.ILoadBalancer
        """
        verifyObject(ILoadBalancer, loadBalancer)
        self._loadBalancer = loadBalancer
    
    def registerControlFactory(self, ctrlFactory):
        """ Register the ControlFactory.
            
            @param ctrlFactory:     ControlFactory which should be registered.
            @type  ctrlFactory:     core.interfaces.IControlFactory
        """
        verifyObject(IControlFactory, ctrlFactory)
        self._ctrlFactory = ctrlFactory
    
    def _createUser(self, userID):
        """ Create a new user.
            
            @param userID:      Associate the user ID with the newly created
                                user.
            @type  userID:      str
            
            @raise:     errors.InvalidRequest
        """
        if userID in self.__users:
            raise InvalidRequest('User does already exist.')
        
        self.__users[userID] = User(self._ctrlFactory, self, userID)
    
    # TODO: Not called from anywhere
    def _destroyUser(self, userID):
        """ Destroy a user.
            
            @param userID:      Destroy the user matching the userID.
            @type  userID:      str
            
            @raise:     errors.InvalidRequest
        """
        if userID not in self.__users:
            raise InvalidRequest('User does not exist.')
        
        self.__users.pop(userID).delete()
    
    def newConnection(self, userID, robotID):
        """ # TODO: Add description
        """
        # TODO: Here would probably be the place to authenticate the user
        if userID not in self.__users:
            self._createUser(userID)
        
        user = self.__users[userID]
        key = uuid4().hex
        commID, ip = self._loadBalancer.getNextRobotLocation()
        
        user.createRobot(robotID, key, commID)
        
        return (key, ip)
    
    def processRequest(self, request):
        """ Process a request received from a robot.
            
            @param request:     Request which should be processed.
            @type  request:     { 'user' : str, 'type' : str, 'args' : tuple }
        """
        try:
            user = request['user']
            identifier = request['type']
            args = request['args']
        except KeyError as e:
            raise InvalidRequest('Request is missing the key "{0}".'.format(e))
        
        try:
            user = self.__users[user]
        except KeyError:
            raise InvalidRequest('User "{0}" does not exist.'.format(user))
        
        if identifier not in self._REQUEST_MAP:
            InvalidRequest('Received request type "{0}", which can not be '
                           'handled.'.format(identifier))
        
        self._REQUEST_MAP[identifier](user, args)
    
    def getCommID(self):
        """ Get a new unique communication ID.
            
            @return:    New CommID.
            @rtype:     str
        """
        while 1:
            commID = definition.generateAddress()
            
            if commID not in self._commIDs:
                self._commIDs.add(commID)
                log.msg('Created CommID "{0}".'.format(commID))
                return commID
    
    def delCommID(self, commID):
        """ Delete the returned communication ID.
            
            @param commID:      CommID which should be destroyed.
            @type  commID:      str
        """
        try:
            self._commIDs.remove(commID)
        except KeyError:
            log.msg('Tried to destroy a non existent CommID "{0}".'.format(
                                                                    commID))
        else:
            log.msg('Destroyed CommID "{0}".'.format(commID))
    
    def getNextContainerLocation(self):
        """ Get the CommID of the container manager where the next container
            should be created.
            
            @return:    CommID of the container manager where the next
                        container should be created.
            @rtpye:     str
        """
        return self._loadBalancer.getNextContainerLocation()
    
    def shutdown(self):
        super(MasterManager, self).shutdown()
        
        self._loadBalancer = None
        self._ctrlFactory = None
