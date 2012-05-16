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
from twisted.internet.ssl import ClientContextFactory

# Custom imports
import settings
from Exceptions import InternalError
from Comm.Message import MsgTypes
from Comm.Message.StdProcessor import ConnectDirectiveProcessor
from Comm.Message.ContainerProcessor import CreateEnvProcessor, DestroyEnvProcessor
from Comm.Message.Base import Message
from Comm.Factory import ReappengineClientFactory

class SatelliteManager(object):
    """ Manager which is used for the satellites nodes, which represent the communication
        relay for the container nodes on a single machine.
    """
    def __init__(self, ctx, commMngr):
        """ Initialize the necessary variables for the environment manager.
            
            @param ctx:     SSLContext which is used for the connections to
                            the other satellite nodes.
            @type  ctx:     # TODO: Determine type of argument
            
            @param commMngr:    CommManager which should be used to communicate.
            @type  commMngr:    CommManager
        """
        # References used by the ROS manager
        self._commMngr = commMngr
        
        # SSL Context which is used to connect to other satellites
        self._ctx = ClientContextFactory()
        self._ctx._contextFactory = ctx
        
        # Storage for all running containers
        self._containers = {}
        self.containerMngrID = None
        
        # Register Message Processors
        self._commMngr.registerMessageProcessor(ConnectDirectiveProcessor(self))
        self._commMngr.registerMessageProcessor(CreateEnvProcessor(self))
        self._commMngr.registerMessageProcessor(DestroyEnvProcessor(self))
        # TODO: Add all valid messages    
    
    def createContainer(self, commID, homeFolder):
        """
        """
        if commID in self._containers:
            raise InternalError('There is already a container with the same CommID.')
        
        container = None # TODO: Add container object here
        
        self._containers[commID] = container
    
    def destroyContainer(self, commID):
        """
        """
        if commID not in self._containers:
            log.msg('Tried to terminate an nonexistent container.')
            return
        
        container = self._containers[commID]
        
        # TODO: Delete container
        
        del self._containers[commID]
    
    def getSatelliteRouting(self):
        """ Returns the routing information for all nodes which should be
            routed through this node, i.e. all container nodes managed by this
            satellite node.
            
            @rtype:     [ str ]
        """
        return self._containers.keys()
    
    def connectToSatellites(self, satellites):
        """ Connect to specified satellites.
            
            @param satellites:  List of dictionaries containing the necessary
                                information each (ip, port, commID).
            @type  satellites:  [ { str : str } ]
        """
        for satellite in satellites:
            satelliteID = satellite['commID']
            ip = satellite['ip']
            port = satellite['port']
            self._commMngr.reactor.connectSSL(ip, port, ReappengineClientFactory(self._commMngr, satelliteID, ''), self._ctx)
    
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
