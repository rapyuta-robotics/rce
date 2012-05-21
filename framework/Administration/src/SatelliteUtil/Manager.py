#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       SatelliteManager.py
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

# Python specific imports
import os

# Custom imports
import settings
from Exceptions import InternalError, InvalidRequest
from Comm.Message import MsgDef
from Comm.Message import MsgTypes
from Comm.Message.Base import Message, validateAddress
from Comm.Message.SatelliteType import ConnectDirectiveMessage, CreateEnvMessage, DestroyEnvMessage #@UnresolvedImport
from Comm.Message.SatelliteProcessor import ConnectDirectiveProcessor, CreateEnvProcessor, DestroyEnvProcessor #@UnresolvedImport
from Comm.Message.ContainerType import StartContainerMessage, StopContainerMessage, ContainerStatusMessage #@UnresolvedImport
from Comm.Message.ContainerProcessor import ContainerStatusProcessor #@UnresolvedImport
from Comm.Factory import ReappengineClientFactory
from Triggers import SatelliteRoutingTrigger #@UnresolvedImport
from MiscUtility import generateID

class Container(object):
    """ Class which represents a container.
    """
    def __init__(self, commID, ip, homeDir, key):
        """ Initialize the Container.
            
            @param commID:  CommID which is used for the environment node inside the container
                            and which is used to identify the container.
            @type  commID:  str
            
            @param ip:      IP address which is used for the container.
            @type  ip:      str
            
            @param homeDir:     Directory which should be used as home directory for container.
            @type  homeDir:     str
            
            @param key:     Key which should be used to identify the container.
            @type  key:     str
        """
        self._commID = commID
        self._ip = ip
        self._homeDir = homeDir
        self._key = key
        
        self._running = False
        self._connected = False
    
    def authenticate(self, ip, key):
        """ Authenticate connection from container.
                            
            @param ip:      IP address which should be used for the authentication attempt.
            @type  ip:      str
                            
            @param key:     Key which should be used for the authentication attempt.
            @type  key:     str
            
            @return:        True if connection is successfully authenticated; False otherwise.
        """
        return ip == self._ip and key == self._key
    
    def setConnectedFlag(self, flag, newKey=None):
        """ Set the 'connected' flag for the container.
            
            @param flag:    Flag which should be set. True for connected and False for
                            not connected.
            @type  flag:    bool
            
            @param newKey:  New key which should be used to authenticate the container in
                            next connection attempt. This parameter has to be set if flag
                            is True and is ignored when the flag is False.
            @type  newKey:  str
            
            @raise:         InvalidRequest if the container is already registered as connected.
                            InternalError if the container should be set to connected without a new key.
        """
        if not self._running:
            raise InvalidRequest('Tried to manipulate "connected" flag of container which is not running.')
        
        if flag:
            if self._connected:
                raise InvalidRequest('Tried to set container to connected which already is registered as connected.')
            
            if not newKey:
                raise InternalError('Tried to set container to connected without a new key.')
            
            self._key = newKey
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
                        'home'   : self._homeDir,
                        'key'    : self._key }
        
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
        # References used by the ROS manager
        self._commMngr = commMngr
        
        # SSL Context which is used to connect to other satellites
        self._ctx = ctx
        
        # Storage for all running containers
        self._containers = {}
        
        # Register Content Serializers
        self.registerContentSerializers([ ConnectDirectiveMessage(),
                                          CreateEnvMessage(),
                                          DestroyEnvMessage(),
                                          StartContainerMessage(),
                                          StopContainerMessage(),
                                          ContainerStatusMessage() ])    # <- necessary?
        # TODO: Check if all these Serializers are necessary
        
        # Register Message Processors
        self._commMngr.registerMessageProcessors([ ConnectDirectiveProcessor(self),
                                                   CreateEnvProcessor(self),
                                                   DestroyEnvProcessor(self),
                                                   ContainerStatusProcessor(self) ])
        # TODO: Add all valid messages    
    
    def createContainer(self, commID, homeFolder):
        """ Method which is used to send a start container request to the container node.
            
            @param commID:  CommID which should be used for the node.
            @type  commID:  str
            
            @param homeFolder:  Directory which should be used as home folder for the
                                container.
            @type  homeFolder:  str
        """
        if not validateAddress(commID):
            log.msg('The CommID is not a valid address.')
            return
        
        if commID in self._containers:
            log.msg('There is already a container with the same CommID.')
            return
        
        if not os.path.isdir(homeFolder):
            log.msg('The home folder is not a valid directory.')
            return
        
        key = generateID()
        container = Container(commID, homeFolder, key)
        
        container.start(self._commMngr)
        self._containers[commID] = container
    
    def authentiateContainerConnection(self, commID, key):
        """ Authenticate connection from container.
                            
            @param commID:  CommID from which the connection originated.
            @type  commID:  str
            
            @param key:     Key which should be used for the authentication attempt.
            @type  key:     str
            
            @return:        True if connection is successfully authentiated; False otherwise
        """
        if commID not in self._containers:
            log.msg('Received a initialization request from an unexpected source.')
            return False
        
        if not self._containers[commID].authenticate(key):
            log.msg('Tried to initialize a connection without a valid key.')
            return False
        
        return True
    
    def setConnectedFlagContainer(self, commID, flag, newKey=None):
        """ Set the 'connected' flag for the container matching the commID.
            
            @param commID:  CommID which should be used to identify the container.
            @type  commID:  str
            
            @param flag:    Flag which should be set. True for connected and False for
                            not connected.
            @type  flag:    bool
            
            @param newKey:  New key which should be used to authenticate the container in
                            next connection attempt. This parameter has to be set if flag
                            is True and is ignored when the flag is False.
            @type  newKey:  str
            
            @raise:         InvalidRequest if the container is already registered as connected
                            or if the CommID does not match any container.
                            InternalError if the key is missing and flag is True.
        """
        if commID not in self._containers:
            raise InvalidRequest('CommID does not match any container.')
        
        self._containers[commID].setConnectedFlag(flag, newKey)
    
    def destroyContainer(self, commID):
        """ Method which is used to send a stop container request to the container node.
            
            @param commID:  CommID of the container which should be stopped.
            @type  commID:  str
        """
        if commID not in self._containers:
            log.msg('Tried to terminate a nonexistent container.')
            return
        
        self._containers[commID].stop(self._commMngr)
        del self._containers[commID]
    
    def getSatelliteRouting(self):
        """ Returns the routing information for all nodes which should be
            routed through this node, i.e. all container nodes managed by this
            satellite node.
            
            @rtype:     [ str ]
        """
        return self._containers.keys()
    
    def _connectToSatellite(self, commID, ip, port):
        """ Connect to another satellite node.
        """
        factory = ReappengineClientFactory(self._commMngr, commID, '', SatelliteRoutingTrigger(self._commMngr, self))
        factory.addApprovedMessageTypes([ MsgTypes.ROUTE_INFO,
                                          MsgTypes.ROS_MSG ])
        self._commMngr.reactor.connectSSL(ip, port, factory, self._ctx)
    
    def connectToSatellites(self, satellites):
        """ Connect to specified satellites.
            
            @param satellites:  List of dictionaries containing the necessary
                                information of each satellite (ip, port, commID).
            @type  satellites:  [ { str : str } ]
        """
        for satellite in satellites:
            self._connectToSatellite(satellite['commID'], satellite['ip'], satellite['port'])
    
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
        """ Method is called when the manager/factory is stopped.
        """
        # TODO: Terminate the containers
