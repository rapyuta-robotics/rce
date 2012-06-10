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

# Custom imports
from Exceptions import InvalidRequest
from NodeManager import ManagerBase

from EnvironmentUtil.Type import ROSAddMessage, ROSRemoveMessage
from Processor import ROSAddProcessor, ROSRemoveProcessor

from ROSComponents.Node import Node
from ROSComponents.NodeMonitor import NodeMonitor
from ROSUtil import Loader

class LauncherManager(ManagerBase):
    """ Manager which handles launching the ROS nodes.
    """
    def __init__(self, commManager):
        """ Initialize the LauncherManager.
            
            @param commManager:     CommManager which should be used to communicate.
            @type  commManager:     CommManager
        """
        super(LauncherManager, self).__init__(commManager)
        
        # References used by the manager
        self._loader = Loader()
        
        # Storage of all nodes
        self._nodes = {}
        
        # Register Content Serializers
        rosAdd = ROSAddMessage()
        rosAdd.registerComponents([ Node ])
        self._commManager.registerContentSerializers([ rosAdd,
                                                       ROSRemoveMessage() ])
        
        # Register Message Processors
        self._commManager.registerMessageProcessors([ ROSAddProcessor(self),
                                                      ROSRemoveProcessor(self) ])
    
    @property
    def loader(self):
        """ Loader for ROS components """
        return self._loader
    
    def addNode(self, node):
        """ Add a Node to the ROS environment.
            
            @param node:    Node instance containing the necessary launch
                            information which should be added.
            @type  node:    Node
        """
        tag = node.tag
        
        if tag in self._nodes:
            raise InvalidRequest('Node already exists.')
        
        nodeMonitor = NodeMonitor(self, node)
        nodeMonitor.start()
        self._nodes[tag] = nodeMonitor
    
    def removeNode(self, tag):
        """ Remove a Node from the ROS environment.
            
            @param tag:     Tag which is used to identify the node which should
                            be removed.
            @type  tag:     str
        """
        try:
            del self._nodes[tag]
            # self._nodes[tag].pop(tag).stop() <- implicitly called by destructor of NodeMonitor
        except KeyError:
            raise InvalidRequest('Node does not exist.')
    
    def shutdown(self):
        """ Method is called when the manager is stopped.
        """
        self._nodes = {}
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
