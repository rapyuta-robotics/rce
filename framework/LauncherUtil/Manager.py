#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Manager.py
#       
#       This file is part of the RoboEarth Cloud Engine framework.
#       
#       This file was originally created for RoboEearth - http://www.roboearth.org/
#       The research leading to these results has received funding from the European Union 
#       Seventh Framework Programme FP7/2007-2013 under grant agreement no248942 RoboEarth.
#       
#       Copyright 2012 RoboEarth
#       
#       Licensed under the Apache License, Version 2.0 (the "License");
#       you may not use this file except in compliance with the License.
#       You may obtain a copy of the License at
#       
#       http://www.apache.org/licenses/LICENSE-2.0
#       
#       Unless required by applicable law or agreed to in writing, software
#       distributed under the License is distributed on an "AS IS" BASIS,
#       WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#       See the License for the specific language governing permissions and
#       limitations under the License.
#       
#       \author/s: Dominique Hunziker <dominique.hunziker@gmail.com> 
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
        if node.tag in self._nodes:
            raise InvalidRequest('Node already exists.')
        
        nodeMonitor = NodeMonitor(self, node)
        nodeMonitor.start()
    
    def removeNode(self, tag):
        """ Remove a Node from the ROS environment.
            
            @param tag:     Tag which is used to identify the node which should
                            be removed.
            @type  tag:     str
        """
        try:
            self._nodes[tag].stop()
        except KeyError:
            raise InvalidRequest('Node does not exist.')
    
    def registerNode(self, node):
        """ Callback for NodeMonitor to register the node.
        """
        if node.tag in self._nodes:
            raise InvalidRequest('Node already exists.')
        
        self._nodes[node.tag] = node
    
    def unregisterNode(self, node):
        """ Callback for NodeMonitor to unregister the node.
        """
        try:
            del self._nodes[node.tag]
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
