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
from twisted.internet.defer import Deferred

# Custom imports
import settings
from Exceptions import InvalidRequest
from NodeManager import ManagerBase

from Comm.Message import MsgDef
from Comm.Message import MsgTypes
from Comm.Message.Base import Message
from Comm.Factory import ReappengineClientFactory
from Triggers import SatelliteRoutingTrigger

from ContainerUtil.Type import StartContainerMessage, StopContainerMessage #, ContainerStatusMessage
from EnvironmentUtil.Type import ROSAddMessage, ROSRemoveMessage, ROSUserMessage, ROSMsgMessage
from MasterUtil.Type import ConnectDirectiveMessage, GetCommIDRequestMessage, GetCommIDResponseMessage, DelCommIDRequestMessage

from Processor import ConnectDirectiveProcessor, GetCommIDProcessor, ROSMsgProcessor #, ContainerStatusProcessor

from DBUtil.DBInterface import DBInterface

from Converter.Core import Converter

from ROSComponents.Node import Node
from ROSComponents.Interface import ServiceInterface, PublisherInterface, SubscriberInterface
from ROSComponents.Parameter import IntParam, StrParam, FloatParam, BoolParam, FileParam
from ROSUtil import Loader

class SatelliteManager(ManagerBase):
    """ Manager which is used for the satellites nodes, which represent the communication
        relay for the container nodes on a single machine.
    """
    def __init__(self, commManager, ctx):
        """ Initialize the necessary variables for the SatelliteManager.
            
            @param commManager:     CommManager which should be used to communicate.
            @type  commManaggr:     CommManager
            
            @param ctx:     SSLContext which is used for the connections to
                            the other satellite nodes.
            @type  ctx:     # TODO: Determine type of argument
        """
        super(SatelliteManager, self).__init__(commManager)
        
        # References used by the manager
        self._dbInterface = DBInterface(commManager)   # TODO: Atm not used!
        self._loader = Loader()
        self._converter = Converter()
        
        # SSL Context which is used to connect to other satellites
        self._ctx = ctx
        
        # Storage for all connected robots and all containers
        self._robots = {}
        self._containers = {}
        
        # Storage for pending requests for a new CommID
        self._pendingCommIDReq = []
        
        # Register Content Serializers
        rosAdd = ROSAddMessage()
        rosAdd.registerComponents([ Node,
                                    ServiceInterface,
                                    PublisherInterface,
                                    SubscriberInterface,
                                    IntParam,
                                    StrParam,
                                    FloatParam,
                                    BoolParam,
                                    FileParam ])
        self._commManager.registerContentSerializers([ ConnectDirectiveMessage(),
                                                       GetCommIDRequestMessage(),
                                                       GetCommIDResponseMessage(),
                                                       DelCommIDRequestMessage(),
                                                       StartContainerMessage(),
                                                       StopContainerMessage(),
                                                       #ContainerStatusMessage(),    # <- necessary?
                                                       rosAdd,
                                                       ROSRemoveMessage(),
                                                       ROSUserMessage(),
                                                       ROSMsgMessage() ])
        # TODO: Check if all these Serializers are necessary
        
        # Register Message Processors
        self._commManager.registerMessageProcessors([ ConnectDirectiveProcessor(self),
                                                      GetCommIDProcessor(self),
                                                      #ContainerStatusProcessor(self),
                                                      ROSMsgProcessor(self) ])
        # TODO: Add all valid messages
    
    @property
    def loader(self):
        """ Loader for ROS resources. """
        return self._loader
    
    ##################################################
    ### Robot
    
    def registerRobot(self, robot):
        """ Callback method for robots to register themselves with the manager.
            
            @param robot:   Robot which should be registered. The robot needs to have
                            an unique robotID.
            @type  robot:   Robot
        """
        uid = robot.robotID
        
        if uid in self._robots:
            raise InvalidRequest('Robot with the same ID is already registered.')
        
        self._robots[uid] = robot
    
    # TODO: Needs to be called from somewhere!
    def unregisterRobot(self, robot):
        """ Unregister robot from the manager.
            
            @param robot:   Robot which should be unregistered. The robot needs to have
                            an unique robotID.
            @type  robot:   Robot
        """
        uid = robot.robotID
        
        if uid not in self._robots:
            raise InvalidRequest('Robot is not registered.')
        
        del self._robots[uid]
    
    ##################################################
    ### Container
    
    def registerContainer(self, container):
        """ Register a container.
        """
        self._containers[container.commID] = container
    
    def unregisterContainer(self, container):
        """ Unregister a container.
        """
        del self._containers[container.commID]
    
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
    
    ##################################################
    ### ROS
    
    def receivedROSMessage(self, msg):
        """ Callback to process a received ROS message from a container.
            
            @param msg:     Received message.
            @type  msg:     Message
        """
        try:
            container = self._containers[msg.origin]
        except KeyError:
            log.msg('Received a ROS message from an invalid source.')
            return
        
        container.receive(msg)
    
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
        factory = ReappengineClientFactory( self._commManager, commID,
                                            '',
                                            SatelliteRoutingTrigger(self._commManager, self) )
        factory.addApprovedMessageTypes([ MsgTypes.ROUTE_INFO,
                                          MsgTypes.ROS_MSG ])
        #self.reactor.connectSSL(ip, port, factory, self._ctx)
        self.reactor.connectTCP(ip, settings.PORT_SATELLITE_SATELLITE, factory)
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
    
    def getNewCommID(self):
        """ Internally used method to request a new unique CommID.
            
            @return:    Deferred which will fire as soon as the new CommID is available.
            @rtype:     Deferred
        """
        deferred = Deferred()
        self._pendingCommIDReq.append(deferred)
        
        msg = Message()
        msg.msgType = MsgTypes.ID_REQUEST
        msg.dest = MsgDef.MASTER_ADDR
        self._commManager.sendMessage(msg)
        
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
        self._commManager.sendMessage(msg)
    
    def shutdown(self):
        """ Method is called when the manager is stopped.
        """
        for container in self._containers.itervalues():
            container.stop()
        
        self._containers = {}
