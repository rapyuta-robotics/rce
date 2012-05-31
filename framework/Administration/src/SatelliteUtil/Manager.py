#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Manager.py
#       
#       Copyright 2012 dominique hunziker <dominique.hunziker@gmail.com>
#       
#       This program is free software; you can redistribute it and/or modify
#       it under the terms of the GNU General Public License as published by
#       the Free Software Foundation; either version 2 of the License, or
#       (at your option) any later version.
#       
#       This program is distributed in the hope that it will be useful,
#       but WITHOUT ANY WARRANTY; without even the implied warranty of
#       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#       GNU General Public License for more details.
#       
#       You should have received a copy of the GNU General Public License
#       along with this program; if not, write to the Free Software
#       Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#       MA 02110-1301, USA.
#       
#       

# ROS specific imports
import genpy

# twisted specific imports
from twisted.python import log
from twisted.internet.defer import Deferred, DeferredList

# Python specific imports
import os

# Custom imports
import settings
from Exceptions import InternalError, InvalidRequest
from Comm.Message import MsgDef
from Comm.Message import MsgTypes
from Comm.Message.Base import Message
from Comm.Factory import ReappengineClientFactory
from Comm.CommUtil import validateAddress #@UnresolvedImport

from ContainerUtil.Type import StartContainerMessage, StopContainerMessage, ContainerStatusMessage #@UnresolvedImport
from ROSUtil.Type import ROSAddMessage, ROSMsgMessage, ROSRemoveMessage
from MasterUtil.Type import ConnectDirectiveMessage, GetCommIDRequestMessage, GetCommIDResponseMessage, DelCommIDRequestMessage #@UnresolvedImport

from Processor import ConnectDirectiveProcessor, GetCommIDProcessor, ContainerStatusProcessor, ROSMsgProcessor #@UnresolvedImport
from Triggers import SatelliteRoutingTrigger #@UnresolvedImport

from DBUtil.DBInterface import DBInterface #@UnresolvedImport

from ROSUtil.Parser import createParameterParser, createInterfaceParser, createNodeParser

from Converter.Core import Converter #@UnresolvedImport
from Comm.Message.FIFO import MessageFIFO

class _Container(object):
    """ Class which represents a container.
    """
    def __init__(self, commMngr, robotID, commID, homeDir):
        """ Initialize the Container.
            
            @param commMngr:    CommManager which should be used to communicate.
            @type  commMngr:    CommManager
            
            @param robotID:     ID of the robot to which this container belongs.
            @type  robotID:     str
            
            @param commID:  CommID which is used for the environment node inside the container
                            and which is used to identify the container.
            @type  commID:  str
            
            @param homeDir:     Directory which should be used as home directory for container.
            @type  homeDir:     str
        """
        self._commManager = commMngr
        self._robotID = robotID
        self._commID = commID
        self._homeDir = homeDir
        
        # Key:     Interface name
        # Value:   ROS Message class
        self._interfaces = {}
        
        self._running = False
        self._connected = False
    
    def checkOwner(self, robotID):
        """ Check if the robot is the owner of this container.
        """
        return self._robotID == robotID
    
    def setConnectedFlag(self, flag):
        """ Set the 'connected' flag for the container.
            
            @param flag:    Flag which should be set. True for connected and False for
                            not connected.
            @type  flag:    bool
            
            @raise:         InvalidRequest if the container is already registered as connected.
        """
        if not self._running:
            raise InvalidRequest('Tried to manipulate "connected" flag of container which is not running.')
        
        if flag:
            if self._connected:
                raise InvalidRequest('Tried to set container to connected which already is registered as connected.')
            
            self._connected = True
        else:
            self._connected = False
    
    def start(self):
        """ Send a Request to start the container.
        """
        if self._running:
            raise InternalError('Container can be started only once.')
        
        msg = Message()
        msg.msgType = MsgTypes.CONTAINER_START
        msg.dest = MsgDef.PREFIX_CONTAINER_ADDR + self._commManager.commID[MsgDef.PREFIX_LENGTH_ADDR:]
        msg.content = { 'commID' : self._commID,
                        'home'   : self._homeDir }
        
        log.msg('Start container "{0}".'.format(self._commID))
        self._commManager.sendMessage(msg)
        self._running = True
    
    def addInterface(self, name, msgType):
        """ Send a Request to add an Interface.
        """
        if not self._connected:
            raise InternalError('Container has to be connected before an interface can be added.')
        
        if name in self._interfaces:
            raise InvalidRequest('Can not add the interface. Name already exists.')
        
        MsgCls = genpy.message.get_message_class(msgType)
        
        # TODO: Check if necessary...
        #if not MsgCls:
        #    MsgCls = genpy.message.get_service_class(msgType)

        if not MsgCls:
            raise ValueError('Message class for the interface could not be loaded.')
        
        msg = Message()
        msg.msgType = MsgTypes.ROS_ADD
        msg.dest = self._commID
        msg.content = {} # TODO: Add correct content
        self._commManager.sendMessage(msg)
        
        self._interfaces[name] = MsgCls
    
    def getMessageClass(self, name):
        """ Get the message class associated with the interface matching the given name.
        """
        return self._interfaces[name]
    
    def removeInterface(self, name):
        """ Send a Request to remove an Interface.
        """
        if not self._connected:
            raise InternalError('Container has to be connected before an interface can be removed.')
        
        if name not in self._interfaces:
            raise InvalidRequest('Can not remove the interface. Name does not exist.')
        
        msg = Message()
        msg.msgType = MsgTypes.ROS_REMOVE
        msg.dest = self._commID
        msg.content = {} # TODO: Add correct content
        self._commManager.sendMessage(msg)
        
        del self._interfaces[name]
    
    def stop(self):
        """ Send a Request to stop the container.
        """
        if not self._running:
            raise InternalError('Container has to be started before it can be stopped.')
        
        msg = Message()
        msg.msgType = MsgTypes.CONTAINER_STOP
        msg.dest = MsgDef.PREFIX_CONTAINER_ADDR + self._commManager.commID[MsgDef.PREFIX_LENGTH_ADDR:]
        msg.content = { 'commID' : self._commID }
        
        log.msg('Start container "{0}".'.format(self._commID))
        self._commManager.sendMessage(msg)
        self._running = False

class SatelliteManager(object):
    """ Manager which is used for the satellites nodes, which represent the communication
        relay for the container nodes on a single machine.
    """
    def __init__(self, commMngr, ctx):
        """ Initialize the necessary variables for the SatelliteManager.
            
            @param commMngr:    CommManager which should be used to communicate.
            @type  commMngr:    CommManager
            
            @param ctx:     SSLContext which is used for the connections to
                            the other satellite nodes.
            @type  ctx:     # TODO: Determine type of argument
        """
        # References used by the manager
        self._commMngr = commMngr
        self._dbInterface = DBInterface(commMngr)
        self._converter = Converter()
        
        # SSL Context which is used to connect to other satellites
        self._ctx = ctx
        
        # Storage for all running containers
        self._containers = {}
        
        # Storage for pending requests for a new CommID
        self._pendingCommIDReq = []
        
        # Register Content Serializers
        self._commMngr.registerContentSerializers([ ConnectDirectiveMessage(),
                                                    GetCommIDRequestMessage(),
                                                    GetCommIDResponseMessage(),
                                                    DelCommIDRequestMessage(),
                                                    StartContainerMessage(),
                                                    StopContainerMessage(),
                                                    ContainerStatusMessage(),    # <- necessary?
                                                    ROSAddMessage(),
                                                    ROSRemoveMessage(),
                                                    ROSMsgMessage() ])
        # TODO: Check if all these Serializers are necessary
        
        # Register Message Processors
        self._commMngr.registerMessageProcessors([ ConnectDirectiveProcessor(self),
                                                   GetCommIDProcessor(self),
                                                   ContainerStatusProcessor(self),
                                                   ROSMsgProcessor(self) ])
        # TODO: Add all valid messages
    
    ##################################################
    ### DB Interactions
    
    def _getRobotSpecs(self, robotID):
        """ Get the specifications for the robot.
            
            @param robotID:     Unique Identifier of the robot.
            @type  robotID:     str
            
            @return:    Deferred which will fire as soon as a response was received with the
                        following argument:
                            Home folder which belongs to the robot.
                            str
            @rtype:     Deferred
        """
        return self._dbInterface.getRobotSpecs(robotID)
    
    def _getNodeDefParser(self, nodeID):
        """ Get the node definition.
            
            @param nodeID:  Unique Identifier of the node.
            @type  nodeID:  str
            
            @return:    Deferred which will fire as soon as a response was received with the
                        following argument:
                            NodeParser which can be used to parse the received message data.
                            NodeParser
            @rtype:     Deferred
        """
        pkgName, nodeName, params, interfaces = self._dbInterface.getNodeSpecs(nodeID)
    
        params = [createParameterParser(*param) for param in params]
        interfaces = [createInterfaceParser(*interface) for interface in interfaces]
    
        return createNodeParser(pkgName, nodeName, params, interfaces)
    
    ##################################################
    ### Container
    
    def _getContainer(self, robotID, containerID):
        """ Check if the robot is authorized to modify the indicated container. If the
            robot has the permission return the container instance.
            
            @param robotID:     Unique Identifier of the robot.
            @type  robotID:     str
            
            @param containerID:     Identifier of the container which should be modified.
                                    This corresponds to the communication ID of the container.
            @type  containerID:     str
            
            @return:    Container instance which matches the given containerID
            
            @raise:     InvalidRequest if the robotID / containerID pair could not be matched.
        """
        container = self._containers.get(containerID, None)
        
        if not container:
            raise InvalidRequest('ContainerID does not match any container.')
        
        if not container.checkOwner(robotID):
            raise InvalidRequest('Robot is not the owner of the container.')
        
        return container
    
    def createContainer(self, robotID):
        """ Callback for RobotServerFactory. # TODO: Add description
            
            @param robotID:     Unique Identifier of the robot.
            @type  robotID:     str
        """
        def processRobotSpecs(results):
            if not (results[0][0] and results[1][0]):
                log.msg('Could not get necessary data to start a new container.')
                return
            
            homeFolder = results[0][1]
            commID = results[1][1]
            
            if not validateAddress(commID):
                log.msg('The CommID is not a valid address.')
                return
            
            if commID in self._containers:
                log.msg('There is already a container with the same CommID.')
                return
            
            if not os.path.isdir(homeFolder):
                log.msg('The home folder is not a valid directory.')
                return
            
            container = _Container(commID, homeFolder)
            
            container.start(self._commMngr)
            self._containers[commID] = container
        
        deferredRobot = self._getRobotSpecs(robotID)
        deferredCommID = self._getNewCommID()
        deferredList = DeferredList([deferredRobot, deferredCommID])
        deferredList.addCallback(processRobotSpecs)
    
    def authenticateContainerConnection(self, commID):
        """ Callback for EnvironmentServerFactory to authenticate connection from container.
                            
            @param commID:  CommID from which the connection originated.
            @type  commID:  str
            
            @return:        True if connection is successfully authenticated; False otherwise
        """
        if commID not in self._containers:
            log.msg('Received a initialization request from an unexpected source.')
            return False
        else:
            return True
    
    def setConnectedFlagContainer(self, commID, flag):
        """ Callback for EnvironmentServerFactory/PostInitTrigger to set the 'connected'
            flag for the container matching the commID.
            
            @param commID:  CommID which should be used to identify the container.
            @type  commID:  str
            
            @param flag:    Flag which should be set. True for connected and False for
                            not connected.
            @type  flag:    bool
            
            @raise:         InvalidRequest if the container is already registered as connected
                            or if the CommID does not match any container.
        """
        if commID not in self._containers:
            if flag:
                raise InvalidRequest('CommID does not match any container.')
            else:
                return
        
        self._containers[commID].setConnectedFlag(flag)
    
    def destroyContainer(self, robotID, containerID):
        """ Callback for RobotServerFactory. # TODO: Add description
            
            @param robotID:     Unique Identifier of the robot.
            @type  robotID:     str
            
            @param containerID:     Identifier of the container which should be modified.
                                    This corresponds to the communication ID of the container.
            @type  containerID:     str
        """
        container = self._getContainer(robotID, containerID)
        container.stop(self._commMngr)
        del self._containers[containerID]
    
    ##################################################
    ### ROS
    
    def addNode(self, robotID, containerID, nodeID, config):
        """ Callback for RobotServerFactory. # TODO: Add description
            
            @param robotID:     Unique Identifier of the robot.
            @type  robotID:     str
            
            @param containerID:     Identifier of the container which should be modified.
                                    This corresponds to the communication ID of the container.
            @type  containerID:     str
            
            # TODO: Add description
        """
        self._getContainer(robotID, containerID)
        
        def parseAndSend(parser):
            def _parseAndSend():
                parser.parse(config) # TODO: Check code for parsing ; Remove binary/files
                
                msg = Message()
                msg.msgType = MsgTypes.ROS_ADD
                msg.dest = containerID
                msg.content = parser.serialize() # TODO: Check code for serializing
                self._commMngr.sendMessage(msg)
            
            self._commMngr.reactor.callInThread(_parseAndSend)
        
        deferred = self._getNodeDefParser(nodeID)
        deferred.addCallback(parseAndSend)
    
    def sendROSMsgToContainer(self, robotID, containerID, interfaceName, msg):
        """ Callback for RobotServerFactory. # TODO: Add description
            
            @param robotID:     Unique Identifier of the robot.
            @type  robotID:     str
            
            @param containerID:     Identifier of the container which should be modified.
                                    This corresponds to the communication ID of the container.
            @type  containerID:     str
            
            # TODO: Add description
        """
        container = self._getContainer(robotID, containerID)
        MsgCls = container.getMessageClass(interfaceName)
        
        def convertAndSend():
            try:
                rosMsg = self._converter.decode(MsgCls, msg)
            except (TypeError, ValueError) as e:
                raise InvalidRequest(str(e))
            
            fifo = MessageFIFO()
            rosMsg.serialize(fifo)
            
            msg = Message()
            msg.msgType = MsgTypes.ROS_MSG
            msg.dest = containerID
            msg.content = { 'msg'  : fifo,
                            'name' : interfaceName,
                            'uid'  : robotID,
                            'push' : True }
            
            self._commMngr.sendMessage(msg)
        
        self._commMngr.reactor.callInThread(convertAndSend)
    
    def sendROSMsgToRobot(self, robotID, containerID, interfaceName, msg):
        """ # TODO: Add description
        """
        container = self._getContainer(robotID, containerID)
        MsgCls = container.getMessageClass(interfaceName)
        
        def convertAndSend():
            rosMsg = MsgCls()
            rosMsg.deserialize(msg)
            
            try:
                msg = self._converter.encode(rosMsg)
            except (TypeError, ValueError) as e:
                raise InvalidRequest(str(e))
            
            # TODO: Send to robot using robotID
        
        self._commMngr.reactor.callInThread(convertAndSend)
    
    def removeNode(self, robotID, containerID, nodeID):
        """ Callback for RobotServerFactory. # TODO: Add description
            
            @param robotID:     Unique Identifier of the robot.
            @type  robotID:     str
            
            @param containerID:     Identifier of the container which should be modified.
                                    This corresponds to the communication ID of the container.
            @type  containerID:     str
            
            # TODO: Add description
        """
        self._getContainer(robotID, containerID)
        
        msg = Message()
        msg.msgType = MsgTypes.ROS_REMOVE
        msg.dest = containerID
        msg.content = nodeID
        self._commMngr.sendMessage(msg)
    
    ##################################################
    ### Routing
    
    def getSatelliteRouting(self):
        """ Callback for PostInitTrigger.
            
            Returns the routing information for all nodes which should be
            routed through this node, i.e. all container nodes managed by this
            satellite node.
            
            @rtype:     [ str ]
        """
        return self._containers.keys()
    
    def _connectToSatellite(self, commID, ip):
        """ Connect to another satellite node.
        """
        factory = ReappengineClientFactory( self._commMngr, commID,
                                            '',
                                            SatelliteRoutingTrigger(self._commMngr, self) )
        factory.addApprovedMessageTypes([ MsgTypes.ROUTE_INFO,
                                          MsgTypes.ROS_MSG ])
        #self._commMngr.reactor.connectSSL(ip, port, factory, self._ctx)
        self._commMngr.reactor.connectTCP(ip, settings.PORT_SATELLITE_SATELLITE, factory)
        # TODO: Set to SSL
    
    def connectToSatellites(self, satellites):
        """ Callback for MessageProcessor to connect to specified satellites.
            
            @param satellites:  List of dictionaries containing the necessary
                                information of each satellite (ip, port, commID).
            @type  satellites:  [ { str : str } ]
        """
        for satellite in satellites:
            self._connectToSatellite(satellite['commID'], satellite['ip'])
    
    ##################################################
    ### Management
    
    def _getNewCommID(self):
        """ Internally used method to request a new unique CommID.
            
            @return:    Deferred which will fire as soon as the new CommID is available.
            @rtype:     Deferred
        """
        deferred = Deferred()
        self._pendingCommIDReq.append(deferred)
        
        msg = Message()
        msg.msgType = MsgTypes.ID_REQUEST
        msg.dest = MsgDef.MASTER_ADDR
        self._commMngr.sendMessage(msg)
        
        return deferred
    
    def setNewCommID(self, commID):
        """ Callback method used to set new unique CommID.
            
            @param CommID:  New CommID which will be used for a waiting container.
            @type  CommID:  str
        """
        self._pendingCommIDReq.pop().callback(commID)
    
    def updateLoadInfo(self):
        """ This method is called regularly and is used to send the newest load info
            to the master node/load balancer.
        """
        return # TODO: Until content is valid, keep this here
        
        msg = Message()
        msg.msgType = MsgTypes.ROUTE_INFO
        msg.dest = settings.LOAD_INFO_UPDATE
        msg.content = None # TODO: Add meaningful information
        self._commMngr.sendMessage(msg)
    
    def shutdown(self):
        """ Method is called when the manager is stopped.
        """
        for container in self._containers.itervalues():
            container.stop()
        
        self._containers = {}
