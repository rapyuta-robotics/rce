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
import uuid

# Custom imports
from Comm.CommUtil import validateAddress
from Container import Container

class Robot(object):
    """ Class which represents a robot. It is associated with a websocket connection.
        A robot can have multiple containers.
    """
    def __init__(self, commMngr, satelliteMngr, connection, robotID):
        """ Initialize the Robot.
            
            # TODO: Add description
        """
        self._commManager = commMngr
        self._satelliteManager = satelliteMngr
        self._connection = connection
        self._robotID = robotID
        
        self._containers = {}
        
        # Register robot with Satellite Manager
        self._satelliteManager.registerRobot(self)
    
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
            
            container = Container(self._commManager, self, commID, homeFolder)
            
            # Send request to start the container
            container.start()
            
            # Register container in this robot instance
            self._containers[containerTag] = container
            
            # Register container in the manager
            self._satelliteManager.registerContainer(container)
        
        deferredRobot = self._satelliteManager.getRobotSpecs(self._robotID)
        deferredCommID = self._satelliteManager.getNewCommID()
        deferredList = DeferredList([deferredRobot, deferredCommID])
        deferredList.addCallback(processRobotSpecs)
    
    def destroyContainer(self, containerTag):
        self._containers[containerTag].stop()
        container = self._containers.pop(containerTag)
        self._satelliteManager.unregisterContainer(container)
    
    # Note to Dominique: Interface changed. Please check with commands.py
    def addNode(self, containerTag, config):
        deferred = self._satelliteManager.getNodeDefParser(config['nodeID'])
        deferred.addCallback(self._containers[containerTag].addNode, config)
    
    def addInterface(self, containerTag, name, interfaceType, className):
        self._containers[containerTag].addInterface(name, className, interfaceType)
    
    def addParameter(self, containerTag, name, value):
        pass
    
    def sendROSMsgToContainer(self, containerTag, msg):
        """ Method is called when a complete message has been received by the robot
            and should now be processed and forwarded.
            
            @param containerTag:    Container tag which is used to identify a previously
                                    started container which should be used as message
                                    destination.
            @type  containerTag:    str
            
            @param msg:     Corresponds to the dictionary of the field 'data' of the received
                            message. (Necessary keys: type, msgID, interfaceTag, msg)
            @type  msg:     { str : ... }
        """
        self._containers[containerTag].send(msg)
    
    def recursiveBinaryDataSearch(self, multiddict):
        URIBinary = []
        for k,v in multiddict.items():
            if isinstance(v, dict):
                iURIBinary,iMultiDDict = self.recursiveBinaryDataSearch(v)
                URIBinary += iURIBinary
                multiddict[k] = iMultiDDict 
            elif k[-1:]=='*':
                tmpURI = uuid.uuid4().hex
                URIBinary.append({'URI':tmpURI,'binaryData':v})
                multiddict[k] = tmpURI
        return URIBinary, multiddict
    
    def sendROSMsgToRobot(self, containerTag, msg):
        """ Method is called when a message should be sent to the robot.
            
            @param containerTag:    Container tag which is used to identify the container
                                    from which the message originated.
            @type  containerTag:    str
            
            @param msg:     Corresponds to the dictionary of the field 'data' of the received
                            message. (Necessary keys: type, msgID, interfaceTag, msg)
            @type  msg:     { str : ... }
        """
        URIBinary,msgURI = recursiveBinaryDataSearch(msg)
        
        self._connection.sendMessage(msgURI)
        if URIBinary:
            for binData in URIBinary:
                self._connection.sendMessage(binData['URI']+(binData['binaryData'].getvalue()),binary=true)
        
        self._connection.sendMsg("Add Message here!")
    
    def removeParameter(self, containerTag, name):
        pass
    
    def removeInterface(self, containerTag, name):
        self._containers[containerTag].removeInterface(name)
    
    def removeNode(self, containerTag, nodeID):
        self._containers[containerTag].removeNode(nodeID)
