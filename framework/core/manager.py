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
import shutil
from datetime import datetime, timedelta
from uuid import uuid4
from functools import wraps

# zope specific imports
from zope.interface import implements

# twisted specific imports
from twisted.python import log
from twisted.internet.defer import Deferred, DeferredList
from twisted.internet.protocol import ProcessProtocol
from twisted.internet.task import LoopingCall

# Custom imports
from errors import InternalError, InvalidRequest, UserConstraintError, \
    AuthenticationError
from util.interfaces import verifyObject
from util.loader import Loader
from util.converter import Converter
from util import template
from core.types import req
from core.interfaces import IRequestSender, IControlFactory, IMessenger, \
    ILoadBalancer
from core.user import User
from core.monitor import NodeMonitor
from comm import definition
from remote.control import RemoteNodeControl

from util.ssl import createKeyCertPair, loadCertFile, loadKeyFile, \
    writeCertToFile, writeKeyToFile


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
        return self.reactor
    
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
    def __init__(self, reactor):
        super(_ROSManagerBase, self).__init__(reactor)
        
        # References used by the manager
        self._loader = Loader()
    
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
        
        nodeMonitor = NodeMonitor(self, node)
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
            user.nodes = RemoteNodeControl(userID, self._NODE_FWD_PORT,
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
            user.nodes = RemoteNodeControl(userID, self._NODE_FWD_PORT,
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
            interface = self._INTERFACES[interface.IDENTIFIER](
                            self, self._commManager.reactor, userID, interface)
        except KeyError:
            raise InvalidRequest('InterfaceManager can not manage '
                                 'requested component.')
        
        tag = interface.tag

        if tag in interfaces:
            raise InternalError('There is already an interface with '
                                'the same tag.')
        
        log.msg('Register new interface "{0}".'.format(tag))
        interface.start()
        interfaces[tag] = interface
    
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
    @_UserManagerBase.verifySingleUser
    def addInterface(self, userID, interface):
        """ Add an interface.

            @param interface:   Interface which should be added.
            @type  interface:   core.command._EndpointInterfaceCommand

            @raise:     errors.InternalError if there is already an interface
                        with the same tag registered.
        """
        super(ROSInterfaceOnlyManager, self).addInterface(userID, interface)
    
    @_UserManagerBase.verifySingleUser
    def removeInterface(self, userID, tag):
        """ Remove an interface.
            
            @param tag:     Tag which is used to identify the interface which
                            should be removed.
            @type  tag:     str
        """
        super(ROSInterfaceOnlyManager, self).removeInterface(userID, tag)
    
    @_UserManagerBase.verifySingleUser
    def _getInterface(self, userID, tag):
        """ Internally used method to get an interface safely.
            
            @raise:     errors.InternalError if there is no interface matching
                        the given tag.
        """
        super(ROSInterfaceOnlyManager, self)._getInterface(userID, tag)


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
            # TODO: Temporary error message!
            raise AuthenticationError('The key is not correct. '+key+" - "+robot.key)
        
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
    
    @_UserManagerBase.verifyUser
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
            self._users[userID].robots[robotID].conn.sendMessage(msg)
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
    
    @_UserManagerBase.verifyUser
    def addInterface(self, userID, interface):
        """ Add an interface.

            @param interface:   Interface which should be added.
            @type  interface:   core.command._EndpointInterfaceCommand

            @raise:     errors.InternalError if there is already an interface
                        with the same tag registered.
        """
        super(RobotManager, self).addInterface(userID, interface)
    
    @_UserManagerBase.verifyUser
    def removeInterface(self, userID, tag):
        """ Remove an interface.
            
            @param tag:     Tag which is used to identify the interface which
                            should be removed.
            @type  tag:     str
        """
        super(RobotManager, self).removeInterface(userID, tag)
    
    @_UserManagerBase.verifyUser
    def _getInterface(self, userID, tag):
        """ Internally used method to get an interface safely.
            
            @raise:     errors.InternalError if there is no interface matching
                        the given tag.
        """
        super(RobotManager, self)._getInterface(userID, tag)


class RelayManager(_ManagerBase):
    """ # TODO: Add description
    """
    def __init__(self, reactor):
        super(RelayManager, self).__init__(reactor)
        
        self._satellite = set()
        self._updater = LoopingCall(self._updateLoadInfo)
        # self._updater.start(self.__LOAD_INFO_UPDATE)
    
    def registerEndpoint(self, commID):
        """ Register a new endpoint with this manager.
            
            @param commID:      CommID of new endpoint.
            @type  commID:      str
        """
        if commID in self._satellite:
            raise InternalError('Tried to register an endpoint, but '
                                'communication ID is already registered.')
        
        self._satellite.add(commID)
    
    def unregisterEndpoint(self, commID):
        """ Unregister an endpoint from this manager.
            
            @param commID:      CommID of endpoint.
            @type  commID:      str
        """
        if commID not in self._satellite:
            raise InternalError('Tried to unregister a satellite, but '
                                'communication ID is not registered.')
        
        self._satellite.remove(commID)
    
    def getServerRouting(self):
        """ Returns the routing information for all nodes which should be
            routed through this node, i.e. all container nodes managed by this
            server node.
            
            @rtype:     [ str ]
        """
        return tuple(self._satellite)
    
    def _connectToRelay(self, commID, ip):
        """ Connect to another relay manager.
        """
        factory = RCEClientFactory( self._commManager, commID,
                                            '',
                                            ServerRoutingTrigger(self._commManager, self) )
        factory.addApprovedMessageTypes([ MsgTypes.ROUTE_INFO,
                                          MsgTypes.ROS_MSG ])
        if self._USE_SSL:
            self.reactor.connectSSL(ip, settings.PORT_SERVER_SERVER, factory, self._ctx)
        else:
            self.reactor.connectTCP(ip, settings.PORT_SERVER_SERVER, factory)
    
    def connectToRelays(self, servers):
        """ Callback for MessageProcessor to connect to specified servers.
            
            @param servers:  List of dictionaries containing the necessary
                                information of each server (ip, port, commID).
            @type  servers:  [ { str : str } ]
        """
        for server in servers:
            self._connectToRelays(server['commID'], server['ip'])
    
    def _updateLoadInfo(self):
        """ This method is called regularly and is used to send the newest load
            info to the master manager/load balancer.
        """
        # TODO: Add code here for load info update
    
    def shutdown(self):
        # self._updater.stop()
        
        super(RelayManager, self).shutdown()
        
        self._satellite = set()

class ContainerManager(_UserManagerBase):
    """ Manager which handles the management of containers.
    """
    class LXCProtocol(ProcessProtocol):
        """ Protocol which is used to handle the LXC commands.
        """
        def __init__(self, deferred):
            self._deferred = deferred
        
        def processEnded(self, reason):
            self._deferred.callback(reason)
    
    def __init__(self, reactor):
        super(ContainerManager, self).__init__(reactor)
        
        self._confDir = self._CONF_DIR
        self._dataDir = self._DATA_DIR
        self._rootfs = self._ROOTFS
        self._srcRoot = self._ROOT_SRC_DIR
        self._pkgRoot = self._ROOT_PKG_DIR
        
        if not os.path.isabs(self._confDir):
            raise ValueError('Configuration directory is not an '
                             'absolute path.')
        
        if not os.path.isdir(self._confDir):
            raise ValueError('Configuration directory does not exist: '
                             '{0}'.format(self._confDir))
        
        if not os.path.isabs(self._dataDir):
            raise ValueError('Data directory is not an absolute path.')
        
        if not os.path.isdir(self._dataDir):
            raise ValueError('Data directory does not exist: {0}'.format(
                                 self._dataDir))
        
        if not os.path.isabs(self._rootfs):
            raise ValueError('Root file system directory is not an '
                             'absolute path.')
        
        if not os.path.isabs(self._srcRoot):
            raise ValueError('Root source directory is not an absolute path.')
        
        if not os.path.isabs(self._pkgRoot):
            raise ValueError('Root package directory is not an absolute path.')
        
        if not os.path.isdir(self._pkgRoot):
            raise ValueError('Root package directory does not exist: '
                             '{0}'.format(self._pkgRoot))
        
        # Validate the executables used in the container
        environmentExe = os.path.join(self._srcRoot, 'environment.py')
        launcherExe = os.path.join(self._srcRoot, 'launcher.py')
        
        if not os.path.isfile(environmentExe):
            raise ValueError('Root source directory does not contain the '
                             'file "environment.py".')
        
        if not os.access(environmentExe, os.X_OK):
            raise ValueError('File "environment.py" in root source directory '
                             'is not executable.')
        
        if not os.path.isfile(launcherExe):
            raise ValueError('Root source directory does not contain the '
                             'file "launcher.py".')
        
        if not os.access(launcherExe, os.X_OK):
            raise ValueError('File "launcher.py" in root source directory '
                             'is not executable.')
    
    def _createConfigFile(self, confDir):
        """ Create a config file based on the given parameters.
        """
        with open(os.path.join(confDir, 'config'), 'w') as f:
            f.write(template.CONFIG.format(
                        rootfs=self._rootfs,
                        fstab=os.path.join(confDir, 'fstab')))

    def _createFstabFile(self, confDir, dataDir):
        """ Create a fstab file based on the given parameters.
        """
        with open(os.path.join(confDir, 'fstab'), 'w') as f:
            f.write(template.FSTAB.format(
                        proc=os.path.join(self._rootfs, 'proc'),
                        devpts=os.path.join(self._rootfs, 'dev/pts'),
                        sysfs=os.path.join(self._rootfs, 'sys'),
                        homeDir=os.path.join(dataDir, 'ros'),
                        fsHome=os.path.join(self._rootfs, 'home/ros'),
                        srcDir=self._srcRoot,
                        fsSrc=os.path.join(self._rootfs, 'opt/rce/src'),
                        pkgDir=self._pkgRoot,
                        fsPkg=os.path.join(self._rootfs,
                                           'opt/rce/packages'),
                        dataDir=os.path.join(dataDir, 'rce'),
                        fsData=os.path.join(self._rootfs, 'opt/rce/data'),
                        upstartComm=os.path.join(confDir, 'upstartComm'),
                        fsComm=os.path.join(self._rootfs,
                                            'etc/init/rceComm.conf'),
                        upstartLauncher=os.path.join(confDir,
                                                     'upstartLauncher'),
                        fsLauncher=os.path.join(self._rootfs,
                                                'etc/init/rceLauncher.conf')))
    
    def _createUpstartScripts(self, commID, relayID, confDir):
        """ Create an upstart script based on the given parameters.
        """
        with open(os.path.join(confDir, 'upstartComm'), 'w') as f:
            f.write(template.UPSTART_COMM.format(commID=commID,
                                                 serverID=relayID))
        
        with open(os.path.join(confDir, 'upstartLauncher'), 'w') as f:
            f.write(template.UPSTART_LAUNCHER)
    
    def _startContainer(self, deferred, tag, commID, relayID):
        """ Internally used method to start a container.
        """
        # Assemble path for config and fstab file
        confDir = os.path.join(self._confDir, commID)
        
        if os.path.isdir(confDir):
            deferred.errback('There exists already a configuration directory '
                             'with the name "{0}".'.format(commID))
            return
        
        # Assemble path for temporary data
        dataDir = os.path.join(self._dataDir, commID)
        
        if os.path.isdir(dataDir):
            deferred.errback('There exists already a configuration directory '
                             'with the name "{0}".'.format(commID))
            return
        
        # Create files
        try:
            log.msg('Create files and directories...')
            os.mkdir(confDir)
            os.mkdir(dataDir)
            
            rceDir = os.path.join(dataDir, 'rce')
            rosDir = os.path.join(dataDir, 'ros')
            os.mkdir(rceDir)
            os.mkdir(rosDir)
            
            if self._USE_SSL:
                # Create a new certificate and key for environment node
                caCertPath = os.path.join(self._SSL_DIR,
                                          'Container.cert')
                caCert = loadCertFile(caCertPath)
                caKey = loadKeyFile(os.path.join(self._SSL_DIR,
                                                 'container/env.key'))
                (cert, key) = createKeyCertPair(commID, caCert, caKey)
                
                # Copy/save file to data directory
                shutil.copyfile(caCertPath, os.path.join(rceDir, 'ca.pem'))
                writeCertToFile(cert, os.path.join(rceDir, 'cert.pem'))
                writeKeyToFile(key, os.path.join(rceDir, 'key.pem'))
            
            self._createConfigFile(confDir)
            self._createFstabFile(confDir, dataDir)
            self._createUpstartScripts(commID, relayID, confDir)
        except:
            log.msg('Caught and exception when trying to create the '
                    'files for the container.')
            import sys, traceback
            etype, value, _ = sys.exc_info()
            deferred.errback('\n'.join(traceback.format_exception_only(etype,
                                                                       value)))
            return
        
        # Start container
        _deferred = Deferred()
        
        def callback(reason):
            if reason.value.exitCode != 0:
                deferred.errback('Received non zero exit code after start of '
                                 'container: {0}'.format(
                                     reason.getErrorMessage()))
            else:
                deferred.callback(None)
        
        _deferred.addCallbacks(callback, callback)
        
        try:
            log.msg('Start container...')
            cmd = [ '/usr/bin/lxc-start',
                    '-n', commID,
                    '-f', os.path.join(confDir, 'config'),
                    '-d' ]
            self.reactor.spawnProcess(ContainerManager.LXCProtocol(_deferred),
                                      cmd[0], cmd, env=os.environ)
        except:
            log.msg('Caught an exception when trying to start the container.')
            import sys, traceback #@Reimport
            etype, value, _ = sys.exc_info()
            deferred.errback('\n'.join(traceback.format_exception_only(etype,
                                                                       value)))
            return
    
    @_UserManagerBase.verifyUser
    def createContainer(self, userID, container):
        """ Callback for message processor to stop a container.
        """
        containers = self._users[userID].containers
        
        tag = container.tag
        commID = container.commID
        relayID = container.relayID
        
        deferred = Deferred()
        
        def reportSuccess(_):
            self.reactor.callFromThread(
                             lambda: containers.__setitem__(tag, commID))
            log.msg('Container successfully started.')
        
        def reportFailure(msg):
            log.msg('Container could not be started: {0}.'.format(msg))
        
        deferred.addCallbacks(reportSuccess, reportFailure)
        
        if tag in containers:
            deferred.errback('There is already a container registered under '
                             'the same tag.')
        else:
            self.reactor.callInThread(self._startContainer, deferred,
                                      tag, commID, relayID)
    
    def _stopContainer(self, deferred, commID):
        """ Internally used method to stop a container.
        """
        _deferred = Deferred()
        
        def callback(reason):
            error = None
            
            if reason.value.exitCode != 0:
                error = ('Received non zero exit code after stop of '
                         'container: {0}'.format(reason.getErrorMessage()))
            
            try:
                shutil.rmtree(os.path.join(self._confDir, commID))
                shutil.rmtree(os.path.join(self._dataDir, commID))
                self.reactor.callFromThread(self._commIDs.remove, commID)
            except:
                import sys, traceback
                etype, value, _ = sys.exc_info()
                error = '\n'.join(traceback.format_exception_only(etype,
                                                                  value))
            
            if error:
                deferred.errback(error)
            else:
                deferred.callback(None)
        
        _deferred.addCallbacks(callback, callback)
        
        try:
            cmd = ['/usr/bin/lxc-stop', '-n', commID]
            self.reactor.spawnProcess(ContainerManager.LXCProtocol(_deferred),
                                      cmd[0], cmd, env=os.environ)
        except:
            import sys, traceback
            etype, value, _ = sys.exc_info()
            msg = '\n'.join(traceback.format_exception_only(etype, value))
            log.msg(msg)
            deferred.errback(msg)
            return
    
    @_UserManagerBase.verifyUser
    def destroyContainer(self, userID, tag):
        """ Callback for message processor to stop a container.
        """
        containers = self._users[userID].containers
        
        deferred = Deferred()
        
        def reportSuccess(_):
            log.msg('Container successfully stopped.')
        
        def reportFailure(msg):
            log.msg('Container could not be stopped: {0}.'.format(msg))
        
        deferred.addCallbacks(reportSuccess, reportFailure)
        
        if tag not in containers:
            deferred.errback('There is no container registered under '
                             'this tag.')
            return
        else:
            self.reactor.callInThread(self._stopContainer, deferred,
                                      containers[tag])
    
    def shutdown(self):
        deferreds = []
        for user in self._users.itervalues():
            for commID in user.containers.itervalues():
                deferred = Deferred()
                deferreds.append(deferred)
                self._stopContainer(deferred, commID)
        
        super(ContainerManager, self).shutdown()
        
        if deferreds:
            deferredList = DeferredList(deferreds)
            return deferredList


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
        
        self.__users[userID] = User(self._ctrlFactory, self._commManager,
                                   userID)
    
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
        
        return { 'key' : key, 'ip' : ip }
    
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
