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

# ROS specific imports
import rosgraph.names

# twisted specific imports
from twisted.python import log

# Custom imports
from Exceptions import InternalError, InvalidRequest
from Comm.Message import MsgDef
from Comm.Message import MsgTypes
from Comm.Message.Base import Message

from Interface import Interface

from ROSComponents import ComponentDefinition
from ROSComponents.Node import Node
from ROSComponents.Parameter import IntParam, StrParam, FloatParam, BoolParam, FileParam

class Container(object):
    """ Class which represents a container. It is associated with a robot.
        A container can have multiple interfaces.
    """
    def __init__(self, commMngr, robot, tag, commID, homeDir):
        """ Initialize the Container.
            
            @param commMngr:    CommManager which should be used to communicate.
            @type  commMngr:    CommManager
            
            @param robot:       Robot instance to which this container belongs.
            @type  robot:       Robot
            
            @param tag:         Tag which is used by the robot to identify this container.
            @type  tag:         str
            
            @param commID:      CommID which is used for the environment node inside the
                                container and which is used to identify the container.
            @type  commID:      str
            
            @param homeDir:     Directory which should be used as home directory for container.
            @type  homeDir:     str
        """
        self._commManager = commMngr
        self._robot = robot
        self._tag = tag
        self._commID = commID
        self._homeDir = homeDir
        
        self._interfaces = {}
        self._rosAddrs = set()
        
        self._running = False
        self._connected = False
    
    @property
    def tag(self):
        """ Tag of the container used by the robot. """
        return self._tag
    
    @property
    def commID(self):
        """ Communication ID of the container. """
        return self._commID
    
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
    
    def addNode(self, tag, pkg, exe, name):
        """ Add a node to the ROS environment in the container.
            
            @param tag:     Tag which is used to identify the ROS node which
                            should added.
            @type  tag:     str

            @param pkg:     Package name where the node can be found.
            @type  pkg:     str

            @param exe:     Name of executable which should be launched.
            @type  exe:     str
            
            @param name:    Name which the node should use as a ROS address
                            in the environment.
            @type  name:    str
            
            @raise:     InvalidRequest if the name is not a valid ROS name.
        """
        # First make sure, that the received strings are str objects and not unicode
        if isinstance(pkg, unicode):
            try:
                pkg = str(pkg)
            except UnicodeEncodeError:
                raise InternalError('The package {0} is not valid.'.format(pkg))
        
        if isinstance(exe, unicode):
            try:
                exe = str(exe)
            except UnicodeEncodeError:
                raise InternalError('The executable {0} is not valid.'.format(exe))
        
        if isinstance(name, unicode):
            try:
                name = str(name)
            except UnicodeEncodeError:
                raise InternalError('The executable {0} is not valid.'.format(name))
        
        if not rosgraph.names.is_legal_name(name):
            raise InvalidRequest('The name "{0}" is not valid.'.format(name))
        
        msg = Message()
        msg.msgType = MsgTypes.ROS_ADD
        msg.content = Node(tag, pkg, exe, name)
        self.send(msg)
    
    def removeNode(self, tag):
        """ Remove a node from the ROS environment in the container.
            
            @param tag:     Tag which is used to identify the ROS node which should
                            be removed.
            @type  tag:     str
        """
        msg = Message()
        msg.msgType = MsgTypes.ROS_REMOVE
        msg.content = { 'type' : ComponentDefinition.RM_NODE,
                        'tag'  : tag }
        self.send(msg)
    
    def addParameter(self, name, value, paramType):
        """ Add a parameter to the parameter server in the container.
            
            @param name:    Name of the parameter which should be added.
            @type  name:    str
            
            @param value:   Value of the parameter which should be added.
            @type  value:   Depends on @param paramType
            
            @param paramType:   Type of the parameter to add. Valid options are:
                                    int, str, float, bool, file
            @type  paramType:   str
            
            @raise:     InvalidRequest if the name is not a valid ROS name.
        """
        if not rosgraph.names.is_legal_name(name):
            raise InvalidRequest('The name "{0}" is not valid.'.format(name))
        
        if paramType == 'int':
            content = IntParam(name, value)
        elif paramType == 'str':
            content = StrParam(name, value)
        elif paramType == 'float':
            content = FloatParam(name, value)
        elif paramType == 'bool':
            content = BoolParam(name, value)
        elif paramType == 'file':
            content = FileParam(name, value)
        
        msg = Message()
        msg.msgType = MsgTypes.ROS_ADD
        msg.content = content
        self.send(msg)
    
    def removeParameter(self, name):
        """ Remove a parameter from the parameter server in the container.
            
            @param name:    Name of the parameter which should be removed.
            @type  name:    str
        """
        msg = Message()
        msg.msgType = MsgTypes.ROS_REMOVE
        msg.content = { 'type' : ComponentDefinition.RM_PARAMETER,
                        'tag'  : name }
        self.send(msg)
    
    def reserveAddr(self, addr):
        """ Callback method for Interface to reserve the ROS address.
            
            @param addr:    ROS address which should be reserved.
            @type  adrr:    str
            
            @raise:     ValueError if the address is already in use.
        """
        if addr in self._rosAddrs:
            raise ValueError('Address already is in use.')
        
        self._rosAddrs.add(addr)
    
    def freeAddr(self, addr):
        """ Callback method for Interface to free the ROS address.
            
            @param addr:    ROS address which should be freed.
            @type  adrr:    str
        """
        if addr not in self._rosAddrs:
            log.msg('Tried to free an address which was not reserved.')
        else:
            self._rosAddrs.remove(addr)
    
    def addInterface(self, interfaceTag, rosAddr, msgType, interfaceType):
        """ Add an interface to the container.
            
            @param interfaceTag:    Tag which is used to identify the interface to add.
            @type  interfaceTag:    str
            
            @param rosAddr:     ROS name/address which the interface should use.
            @type  rosAddr:     str
            
            @param msgType:     Message type/Service type consisting of the package and the name
                                of the message/service, i.e. 'std_msgs/Int32'.
            @type  msgType:     str
            
            @param interfaceType:   Type of the interface. Valid types are 'service', 'publisher',
                                    and 'subscriber'.
            @type  interfaceType:   str
        """
        if interfaceTag in self._interfaces:
            if not self._interfaces[interfaceTag].validate(interfaceTag, rosAddr, msgType, interfaceType):
                raise InvalidRequest('Another interface with the same tag already exists.')
        else:
            self._interfaces[interfaceTag] = Interface( self._commManager,
                                                        interfaceTag,
                                                        rosAddr,
                                                        msgType,
                                                        interfaceType )
    
    def removeInterface(self, interfaceTag):
        """ Remove an interface to the container.
            
            @param interfaceTag:    Tag which is used to identify the interface to remove.
            @type  interfaceTag:    str
        """
        if interfaceTag not in self._interfaces:
            raise InvalidRequest('Can not remove the interface. Tag does not exist.')
        
        del self._interfaces[interfaceTag]
    
    def activateInterface(self, interfaceTag, user):
        """ Activate an interface for a user.
            
            # TODO: Add description
        """
        if not self._connected:
            raise InternalError('Container has to be connected before an interface can be activated.')
        
        try:
            self._interfaces[interfaceTag].registerUser(user)
        except KeyError:
            raise InternalError('Can not activate an interface which does not exist.')
    
    def deactivateInterface(self, interfaceTag, user):
        """ # TODO: Add description
        """
        if not self._connected:
            raise InternalError('Container has to be connected before an interface can be deactivated.')
        
        try:
            self._interfaces[interfaceTag].unregisterUser(user)
        except KeyError:
            raise InternalError('Can not deactivate an interface which does not exist.')
    
    def send(self, msg):
        """ Send a message to the container.
            
            @param msg:     Message which should be sent to the container. The message
                            has to be a Message instance which contains all data
                            except the destination which are necessary to send the message.
            @type  msg:     Message
        """
        if not isinstance(msg, Message):
            raise InternalError('Can not send an object which is not of type "Message".')
        
        msg.dest = self._commID
        self._commManager.reactor.callInThread(self._commManager.sendMessage, msg)
    
    def receive(self, msg):
        """ Process a received ROS message.
            
            @param msg:     Received message.
            @type  msg:     Message
        """
        msg = msg.content
        
        if msg['user'] != self._robot.robotID:
            log.msg('Received a ROS message from a container with an invalid destination.')
            return
        
        try:
            self._interfaces[msg['tag']].receive(msg)
        except KeyError:
            raise InternalError('Can not process received message. Interface does not exist.')
    
    def sendToInterface(self, msg):
        """ Send a message to the interface matching the given tag. (Called by the Robot)
            
            @param msg:     Corresponds to the dictionary of the field 'data' of the received
                            message. (Necessary keys: type, msgID, interfaceTag, msg)
            @type  msg:     { str : ... }
        """
        try:
            self._commMngr.reactor.callInThread(
                self._interfaces[msg['interfaceTag']].send,
                msg,
                self._robot.robotID
            )
        except KeyError:
            raise InternalError('Can not send message. Interface does not exist.')
    
    def receivedFromInterface(self, msg):
        """ Received a message from an interface and should now be processed.
            (Called by the Interface)
            
            @param msg:     Message which was received in form of a dictionary matching
                            the structure of the ROS message.
            @type  msg:     { str : ... }
        """
        self._robot.sendROSMsgToRobot(self._tag, msg)
