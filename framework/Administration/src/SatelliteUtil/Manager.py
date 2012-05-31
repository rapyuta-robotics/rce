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

# twisted specific imports
from twisted.python import log
from twisted.internet.threads import deferToThread 

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
from Type import ConnectDirectiveMessage #@UnresolvedImport

from Processor import ConnectDirectiveProcessor, ContainerStatusProcessor, ROSMsgProcessor #@UnresolvedImport
from Triggers import SatelliteRoutingTrigger #@UnresolvedImport

from DBUtil.DBInterface import DBInterface #@UnresolvedImport

from ROSUtil.Parser import createParameterParser, createInterfaceParser, createNodeParser

from Converter.Core import Converter #@UnresolvedImport

class _Container(object):
    """ Class which represents a container.
    """
    def __init__(self, commID, homeDir):
        """ Initialize the Container.
            
            @param commID:  CommID which is used for the environment node inside the container
                            and which is used to identify the container.
            @type  commID:  str
            
            @param homeDir:     Directory which should be used as home directory for container.
            @type  homeDir:     str
        """
        self._commID = commID
        self._homeDir = homeDir
        
        self._running = False
        self._connected = False
    
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
    
    def start(self, commManager):
        """ Send a Request to start the container.
        """
        if self._running:
            raise InternalError('Container can be started only once.')
        
        msg = Message()
        msg.msgType = MsgTypes.CONTAINER_START
        msg.dest = MsgDef.PREFIX_CONTAINER_ADDR + commManager.commID[MsgDef.PREFIX_LENGTH_ADDR:]
        msg.content = { 'commID' : self._commID,
                        'home'   : self._homeDir }
        
        log.msg('Start container "{0}".'.format(self._commID))
        commManager.sendMessage(msg)
        self._running = True
    
    def stop(self, commManager):
        """ Send a Request to stop the container.
        """
        if not self._running:
            raise InternalError('Container has to be started before it can be stopped.')
        
        msg = Message()
        msg.msgType = MsgTypes.CONTAINER_STOP
        msg.dest = MsgDef.PREFIX_CONTAINER_ADDR + commManager.commID[MsgDef.PREFIX_LENGTH_ADDR:]
        msg.content = { 'commID' : self._commID }
        
        log.msg('Start container "{0}".'.format(self._commID))
        commManager.sendMessage(msg)
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
        
        # Register Content Serializers
        self._commMngr.registerContentSerializers([ ConnectDirectiveMessage(),
                                                    StartContainerMessage(),
                                                    StopContainerMessage(),
                                                    ContainerStatusMessage(),    # <- necessary?
                                                    ROSAddMessage(),
                                                    ROSRemoveMessage(),
                                                    ROSMsgMessage() ])
        # TODO: Check if all these Serializers are necessary
        
        # Register Message Processors
        self._commMngr.registerMessageProcessors([ ConnectDirectiveProcessor(self),
                                                   ContainerStatusProcessor(self),
                                                   ROSMsgProcessor(self) ])
        # TODO: Add all valid messages
    
    ##################################################
    ### DB Interactions
    
    def _validateRobot(self, robotID, containerID):
        """ Check if the robot is authorized to modify the indicated container.
            
            @param robotID:     Unique Identifier of the robot.
            @type  robotID:     str
            
            @param containerID:     Identifier of the container which should be modified.
                                    This corresponds to the communication ID of the container.
            @type  containerID:     str
            
            @return:    True if the robot could be validated; False otherwise.
        """
        return self._dbInterface.validateRobot(robotID, containerID)
        # TODO: Implement method in the DB server.
        # TODO: At the moment dbInterface returns Deferred!!!
    
    def _getRobotSpecs(self, robotID):
        """ Get the specifications for the robot.
            
            @param robotID:     Unique Identifier of the robot.
            @type  robotID:     str
            
            @return:    Home folder which belongs to the robot.
            @rtype:     str
        """
        return self._dbInterface.getRobotSpecs(robotID)
        # TODO: Implement method in the DB server.
        # TODO: At the moment dbInterface returns Deferred!!!
    
    def _getNodeDefParser(self, uid):
        """ Get the node definition.
            
            @param uid:     Unique Identifier of the node.
            @type  uid:     str
            
            @return:    NodeParser which can be used to parse the received message data.
        """
        pkgName, nodeName, params, interfaces = self._dbInterface.getNodeSpecs(uid)
        # TODO: Implement method in the DB server.
        # TODO: At the moment dbInterface returns Deferred!!!
    
        params = [createParameterParser(*param) for param in params]
        interfaces = [createInterfaceParser(*interface) for interface in interfaces]
    
        return createNodeParser(pkgName, nodeName, params, interfaces)
    
    ##################################################
    ### Container
    
    def _createContainer(self, robotID):
        """ Internally used method to send a start container request to the container node.
            
            For more details refer to 'createContainer'.
        """
        homeFolder = self._getRobotSpecs(robotID)
        
        commID = 'TODO' # TODO: Generate a unique commID for container
        
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
    
    def createContainer(self, robotID):
        """ Callback for RobotServerFactory. # TODO: Add description
            
            @param robotID:     Unique Identifier of the robot.
            @type  robotID:     str
        """
        deferToThread(self._createContainer, robotID)
    
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
    
    def _destroyContainer(self, robotID, containerID):
        """ Internally used method to send a stop container request to the container node.
            
            For more details refer to 'destroyContainer'.
        """
        if not self._validateRobot(robotID, containerID):
            raise InvalidRequest('')    # TODO: Add error
        
        if containerID not in self._containers:
            log.msg('Tried to terminate a nonexistent container.')
            return
        
        self._containers[containerID].stop(self._commMngr)
        del self._containers[containerID]
    
    def destroyContainer(self, robotID, containerID):
        """ Callback for RobotServerFactory. # TODO: Add description
            
            @param robotID:     Unique Identifier of the robot.
            @type  robotID:     str
            
            @param containerID:     Identifier of the container which should be modified.
                                    This corresponds to the communication ID of the container.
            @type  containerID:     str
        """
        deferToThread(self._destroyContainer, robotID, containerID)
    
    ##################################################
    ### ROS
    
    def _addNode(self, robotID, containerID, nodeID, config, binary):
        """ Internally method used to send add a node to a container.
            
            For more details refer to 'addNode'.
        """
        if not self._validateRobot(robotID, containerID):
            raise InvalidRequest('')    # TODO: Add error
        
        parser = self._getNodeDefParser(nodeID)
        parser.parse(config, binary) # TODO: Check code for parsing
        
        msg = Message()
        msg.msgType = MsgTypes.ROS_ADD
        msg.dest = containerID
        msg.content = parser.serialize() # TODO: Check code for serializing
        self._commMngr.sendMessage(msg)
    
    def addNode(self, robotID, containerID, nodeID, config, binary):
        """ Callback for RobotServerFactory. # TODO: Add description
            
            @param robotID:     Unique Identifier of the robot.
            @type  robotID:     str
            
            @param containerID:     Identifier of the container which should be modified.
                                    This corresponds to the communication ID of the container.
            @type  containerID:     str
            
            # TODO: Add description
        """
        deferToThread(self._addNode, robotID, containerID, nodeID, config, binary)
    
    def _sendROSMsgToContainer(self, robotID, containerID, msgType, msg, binary):
        """ Internally method used to send a ROS message to a container.
            
            For more details refer to 'sendROSMsgToContainer'.
        """
        if not self._validateRobot(robotID, containerID):
            raise InvalidRequest('')    # TODO: Add error
        
        try:
            rosMsg = self._converter.decode(msgType, msg['???'], binary)    # TODO: What exactly
        except (TypeError, ValueError) as e:
            raise InvalidRequest(str(e))
        
        msg = Message()
        msg.msgType = MsgTypes.ROS_MSG
        msg.dest = containerID
        msg.content = { 'msg'  : rosMsg,
                        'name' : msg['???'],    # TODO: What exactly (Interface Name requiered)
                        'uid'  : robotID,
                        'push' : True }
        
        self._commMngr.sendMessage(msg)
    
    def sendROSMsgToContainer(self, robotID, containerID, msgType, msg, binary):
        """ Callback for RobotServerFactory. # TODO: Add description
            
            @param robotID:     Unique Identifier of the robot.
            @type  robotID:     str
            
            @param containerID:     Identifier of the container which should be modified.
                                    This corresponds to the communication ID of the container.
            @type  containerID:     str
            
            # TODO: Add description
        """
        deferToThread(self._sendROSMsgToContainer, containerID, msgType, msg, binary)
    
    def sendROSMsgToRobot(self, robotID, msgType, msg):
        """ # TODO: Add description
        """
        msg = self._converter.encode(msg)
        
        # TODO: Send to robot using robotID
    
    def _removeNode(self, robotID, containerID, nodeID):
        """ Internally method used to send remove a node from a container.
            
            For more details refer to 'removeNode'.
        """
        if not self._validateRobot(robotID, containerID):
            raise InvalidRequest('')    # TODO: Add error
        
        msg = Message()
        msg.msgType = MsgTypes.ROS_REMOVE
        msg.dest = containerID
        msg.content = nodeID
        self._commMngr.sendMessage(msg)
    
    def removeNode(self, robotID, containerID, nodeID):
        """ Callback for RobotServerFactory. # TODO: Add description
            
            @param robotID:     Unique Identifier of the robot.
            @type  robotID:     str
            
            @param containerID:     Identifier of the container which should be modified.
                                    This corresponds to the communication ID of the container.
            @type  containerID:     str
            
            # TODO: Add description
        """
        deferToThread(self._removeNode, robotID, containerID, nodeID)
    
    ##################################################
    ### Routing / Management
    
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
        factory = ReappengineClientFactory(self._commMngr, commID, '', SatelliteRoutingTrigger(self._commMngr, self))
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
