#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Container.py
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

# Custom imports
from Exceptions import InternalError, InvalidRequest
from Comm.Message import MsgDef
from Comm.Message import MsgTypes
from Comm.Message.Base import Message

from Interface import Interface #@UnresolvedImport

class Container(object):
    """ Class which represents a container.
    """
    def __init__(self, commMngr, robotID, commID, homeDir):
        """ Initialize the Container.
            
            @param commMngr:    CommManager which should be used to communicate.
            @type  commMngr:    CommManager
            
            @param robot:       ID of the robot to which this container belongs.
            @type  robot:       Robot
            
            @param commID:  CommID which is used for the environment node inside the container
                            and which is used to identify the container.
            @type  commID:  str
            
            @param homeDir:     Directory which should be used as home directory for container.
            @type  homeDir:     str
        """
        self._commManager = commMngr
        self._robotID = robotID
        self._commID = commID
        self._homeDir = homeDir
        
        # Key:     Interface name
        # Value:   ROS Message class
        self._interfaces = {}
        
        self._running = False
        self._connected = False
    
    def checkOwner(self, robotID):
        """ Check if the robot is the owner of this container.
        """
        return self._robotID == robotID
    
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
    
    def start(self):
        """ Send a Request to start the container.
        """
        if self._running:
            raise InternalError('Container can be started only once.')
        
        msg = Message()
        msg.msgType = MsgTypes.CONTAINER_START
        msg.dest = MsgDef.PREFIX_CONTAINER_ADDR + self._commManager.commID[MsgDef.PREFIX_LENGTH_ADDR:]
        msg.content = { 'commID' : self._commID,
                        'home'   : self._homeDir }
        
        log.msg('Start container "{0}".'.format(self._commID))
        self._commManager.sendMessage(msg)
        self._running = True
    
    def addNode(self, parser, config):
        """ # TODO: Add description
        """
        def parseAndSend():
            parser.parse(config) # TODO: Check code for parsing ; Remove binary/files
            
            msg = Message()
            msg.msgType = MsgTypes.ROS_ADD
            msg.dest = self._containerID
            msg.content = parser.serialize() # TODO: Check code for serializing
            self._commMngr.sendMessage(msg)
        
        self._commMngr.reactor.callInThread(parseAndSend)
    
    def addInterface(self, name, msgType, interfaceType):
        """ # TODO: Add description
        """
        if name in self._interfaces:
            raise InvalidRequest('Another interface with the same name already exists.')
        
        self._interfaces[name] = Interface(self._commManager, name, msgType, interfaceType)
    
    def activateInterface(self, name, user):
        """ # TODO: Add description
        """
        if not self._connected:
            raise InternalError('Container has to be connected before an interface can be activated.')
        
        try:
            self._interfaces[name].registerUser(user)
        except KeyError:
            raise InternalError('Can not activate an interface which does not exist.')
    
    def send(self, msg, interfaceName, sender):
        """ # TODO: Add description
        """
        try:
            self._commMngr.reactor.callInThread(
                self._interfaces[interfaceName].send,
                msg,
                self._commID,
                sender
            )
        except KeyError:
            raise InternalError('Can not send message. Interface does not exist.')
    
    def receive(self, msg, interfaceName, receiver):
        """ # TODO: Add description
        """
        try:
            self._commMngr.reactor.callInThread(
                self._interfaces[interfaceName].receive,
                msg,
                receiver
            )
        except KeyError:
            raise InternalError('Can not send message. Interface does not exist.')
    
    def getInterface(self, name):
        """ Get the interface instance associated with the interface matching the given name.
        """
        return self._interfaces[name]
    
    def deactivateInterface(self, name, user):
        """ # TODO: Add description
        """
        if not self._connected:
            raise InternalError('Container has to be connected before an interface can be deactivated.')
        
        try:
            self._interfaces[name].unregisterUser(user)
        except KeyError:
            raise InternalError('Can not deactivate an interface which does not exist.')
    
    def removeInterface(self, name):
        """ Send a Request to remove an Interface.
        """
        if name not in self._interfaces:
            raise InvalidRequest('Can not remove the interface. Name does not exist.')
        
        del self._interfaces[name]
    
    def removeNode(self, nodeID):
        """ # TODO: Add description
        """
        msg = Message()
        msg.msgType = MsgTypes.ROS_REMOVE
        msg.dest = self._containerID
        msg.content = nodeID
        self._commMngr.sendMessage(msg)
    
    def stop(self):
        """ Send a Request to stop the container.
        """
        if not self._running:
            raise InternalError('Container has to be started before it can be stopped.')
        
        msg = Message()
        msg.msgType = MsgTypes.CONTAINER_STOP
        msg.dest = MsgDef.PREFIX_CONTAINER_ADDR + self._commManager.commID[MsgDef.PREFIX_LENGTH_ADDR:]
        msg.content = { 'commID' : self._commID }
        
        log.msg('Start container "{0}".'.format(self._commID))
        self._commManager.sendMessage(msg)
        self._running = False
