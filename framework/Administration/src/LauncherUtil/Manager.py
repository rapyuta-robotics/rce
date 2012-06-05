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
from twisted.internet.defer import Deferred, DeferredList

# Python specific imports
import os
from threading import Event

# Custom imports
import settings
from Exceptions import InvalidRequest
from Comm.Message import MsgDef
from Type import StartContainerMessage, StopContainerMessage, ContainerStatusMessage
from Processor import StartContainerProcessor, StopContainerProcessor

from NodeMonitor import NodeMonitor

class LauncherManager(object):
    """ Manager which handles launching the ROS nodes.
    """
    def __init__(self, commMngr):
        """ Initialize the LauncherManager.
            
            @param commMngr:    CommManager which should be used to communicate.
            @type  commMngr:    CommManager
        """
        # References used by the manager
        self._commMngr = commMngr
        
        # Storage of all nodes
        self._nodes = []
        
        # Register Content Serializers
        self._commMngr.registerContentSerializers([ StartContainerMessage(),
                                                    StopContainerMessage(),
                                                    ContainerStatusMessage() ])
        
        # Register Message Processors
        self._commMngr.registerMessageProcessors([ StartContainerProcessor(self),
                                                   StopContainerProcessor(self) ])
    
    def addNode(self, node):
        """
        """
        uid = node.namespace + node.exe     # TODO: Important: UID == namespace + exe atm!!!
        
        if uid in self._nodes:
            raise InvalidRequest('Node already exists.')
        
        nodeMonitor = NodeMonitor(self._commMngr.reactor, node)
        nodeMonitor.start()
        self._nodes[uid] = nodeMonitor
    
    def removeNode(self, uid):
        """
        """
        try:
            del self._nodes[uid]
            # self._nodes[uid].pop(uid).stop() <- implicitly called by destructor of NodeMonitor
        except KeyError:
            raise InvalidRequest('Node does not exist.')
    
    def shutdown(self):
        """ Method is called when the manager is stopped.
        """
        #event = Event()
        #deferreds = []
        
        for node in self._nodes:
            node.stop()
            #deferreds.append(self._stopContainer(commID))
        
        #deferredList = DeferredList(deferreds)
        #deferredList.addCallback(event.set)
        
        #event.wait()
