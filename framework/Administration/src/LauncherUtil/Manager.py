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
from ROSUtil.Type import ROSAddMessage, ROSRemoveMessage
from Processor import ROSAddProcessor, ROSRemoveProcessor

from ROSComponents.Node import Node
from ROSComponents.NodeMonitor import NodeMonitor

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
        rosAdd = ROSAddMessage()
        rosAdd.registerComponents([ Node ])
        self._commMngr.registerContentSerializers([ rosAdd,
                                                    ROSRemoveMessage() ])
        
        # Register Message Processors
        self._commMngr.registerMessageProcessors([ ROSAddProcessor(self),
                                                   ROSRemoveProcessor(self) ])
    
    def addNode(self, node):
        """
        """
        tag = node.tag
        
        if tag in self._nodes:
            raise InvalidRequest('Node already exists.')
        
        nodeMonitor = NodeMonitor(self._commMngr.reactor, node)
        nodeMonitor.start()
        self._nodes[tag] = nodeMonitor
    
    def removeNode(self, tag):
        """
        """
        try:
            del self._nodes[tag]
            # self._nodes[tag].pop(tag).stop() <- implicitly called by destructor of NodeMonitor
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
