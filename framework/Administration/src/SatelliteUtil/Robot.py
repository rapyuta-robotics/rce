#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Robot.py
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
from twisted.internet.defer import DeferredList

# Python specific imports
import os

# Custom imports
from Comm.CommUtil import validateAddress
from Container import Container

class Robot(object):
    """ Class which represents a robot. It is associated with a websocket connection.
        A robot can have multiple containers.
    """
    def __init__(self, robotID, connection, manager):
        """ Initialize the Robot.
            
            # TODO: Add description
        """
        self._robotID = robotID
        self._connection = connection
        self._manager = manager
        
        self._manager.registerRobot(self)
        self._containers = {}
    
    @property
    def robotID(self):
        """ RobotID """
        return self._robotID
    
    def createContainer(self, containerTag):
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
            
            container = Container(commID, homeFolder)
            
            container.start(self._commMngr)
            
            # Register container in this robot instance
            self._containers[commID] = container
            
            # Register container in the manager
        
        deferredRobot = self._manager.getRobotSpecs(self._robotID)
        deferredCommID = self._manager.getNewCommID()
        deferredList = DeferredList([deferredRobot, deferredCommID])
        deferredList.addCallback(processRobotSpecs)
    
    def destroyContainer(self, containerTag):
        self._containers[containerTag].stop(self._commMngr)
        del self._containers[containerTag]
    
    def addNode(self, containerTag, nodeID, config):
        deferred = self._manager.getNodeDefParser(nodeID)
        deferred.addCallback(self._containers[containerTag].addNode, config)
    
    def addInterface(self, containerTag, name, interfaceType, className):
        self._containers[containerTag].addInterface(name, className, interfaceType)
    
    def addParameter(self, containerTag, name, value):
        pass
    
    def sendROSMsgToContainer(self, containerTag, interfaceName, msg):
        self._containers[containerTag].send(msg, interfaceName, self._robotID)
    
    def sendROSMsgToRobot(self, msg):
        # TODO: Add header and separate binary files if necessary
        self._connection.sendMsg("Add Message here!")
    
    def removeParameter(self, containerTag, name):
        pass
    
    def removeInterface(self, containerTag, name):
        self._containers[containerTag].removeInterface(name)
    
    def removeNode(self, containerTag, nodeID):
        self._containers[containerTag].removeNode(nodeID)