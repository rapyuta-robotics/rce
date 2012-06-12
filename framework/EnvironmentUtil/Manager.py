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

# Custom imports
from Exceptions import InternalError
from NodeManager import ManagerBase

from Comm.Message.FIFO import MessageFIFO
from Comm.Message.Base import Message
from Comm.Message import MsgTypes
from Type import ROSAddMessage, ROSRemoveMessage, ROSUserMessage, ROSMsgMessage #, ROSResponseMessage, ROSGetMessage
from Processor import ROSAddProcessor, ROSRemoveProcessor, ROSUserProcessor, ROSMessageContainerProcessor #, ROSGetProcessor

from ROSComponents.Node import NodeForwarder
from ROSComponents.Interface import ServiceInterface, PublisherInterface, SubscriberInterface
from ROSComponents.Parameter import IntParam, StrParam, FloatParam, BoolParam, FileParam
from ROSUtil import Loader

class ROSManager(ManagerBase):
    """ Manager which handles ROS specific tasks.
    """
    def __init__(self, commManager):
        """ Initialize the ROSManager.
            
            @param commManager:     CommManager which should be used to communicate.
            @type  commManager:     CommManager
        """
        super(ROSManager, self).__init__(commManager)
        
        # References used by the manager
        self._loader = Loader()
        
        # Storage for references
        self._interfaces = {}
        self._parameters = {}
        
        # Register Content Serializers
        rosAdd = ROSAddMessage()
        rosAdd.registerComponents([ NodeForwarder,
                                    ServiceInterface,
                                    PublisherInterface,
                                    SubscriberInterface,
                                    IntParam,
                                    StrParam,
                                    FloatParam,
                                    BoolParam,
                                    FileParam ])
        self._commManager.registerContentSerializers([ rosAdd,
                                                       ROSRemoveMessage(),
                                                       ROSUserMessage(),
                                                       ROSMsgMessage() ])
        # ROSResponseMessage(), ROSGetMessage()
        
        # Register Message Processors
        self._commManager.registerMessageProcessors([ ROSAddProcessor(self, commManager),
                                                      ROSRemoveProcessor(self, commManager),
                                                      ROSUserProcessor(self, commManager),
                                                      ROSMessageContainerProcessor(self, commManager) ])
        # ROSGetProcessor(self, commMngr)
    
    @property
    def loader(self):
        """ Loader for ROS resources. """
        return self._loader
    
    def addInterface(self, interface):
        """ Callback for MessageProcessor to add an interface.

            @param interface:   InterfaceMonitor which should be registered.
            @type  interface:   InterfaceMonitor

            @raise:     InternalError if there is already an interface
                        with the same tag registered.
        """
        tag = interface.tag

        if tag in self._interfaces:
            raise InternalError('There is already an interface with the same tag.')
        
        log.msg('Register new interface "{0}".'.format(tag))
        interface.start()
        self._interfaces[tag] = interface
    
    def removeInterface(self, tag):
        """ Callback for MessageProcessor to remove an interface.
            
            @param tag:     Tag which is used to identify the interface which should
                            be removed.
            @type  tag:     str
        """
        log.msg('Remove interface "{0}".'.format(tag))
        
        try:
            self._interfaces.pop(tag).stop()
        except KeyError:
            pass
    
    def getInterface(self, tag):
        """ Get the interface matching the given tag.
            
            @raise:     InternalError if there is no interface matching the given tag.
        """
        if tag not in self._interfaces:
            raise InternalError('There is no interface with the tag "{0}".'.format(tag))
        
        return self._interfaces[tag]
    
    def addInterfaceUser(self, tag, target, commID):
        """ Activate an interface for a user.
            
            @param tag:         Tag which is used to identify the interface which
                                should be activated for a user.
            @type  tag:         str
            
            @param target:      Tag which is used to identify the user/interface for
                                which the interface should be activated.
            @type  target:      str
            
            @param commID:      Communication ID where the target is coming from.
            @type  commID:      str
        """
        log.msg('Register user "({0}, {1})" for interface "{2}".'.format(commID, target, tag))
        self.getInterface(tag).addUser(commID, target)
    
    def removeInterfaceUser(self, tag, target, commID):
        """ Deactivate an interface for a user.
            
            @param tag:         Tag which is used to identify the interface which
                                should be deactivated for a user.
            @type  tag:         str
            
            @param target:      Tag which is used to identify the user/interface for
                                which the interface should be deactivated.
            @type  target:      str
            
            @param commID:      Communication ID where the target is coming from.
            @type  commID:      str
        """
        log.msg('Unregister user "({0}, {1})" for interface "{2}".'.format(commID, target, tag))
        self.getInterface(tag).removeUser(commID, target)
    
    def addParameter(self, parameter):
        """ Callback for MessageProcessor to add a parameter. If the parameter
            already exists, the old will be removed and the new one kept.

            @param parameter:   ParameterMonitor which should be registered.
            @type  parameter:   ParameterMonitor
        """
        name = parameter.name
        log.msg('Register parameter "{0}".'.format(name))
        
        if name in self._parameters:
            self._parameters[name].remove()
        
        self._parameters[name] = parameter
    
    def removeParameter(self, name):
        """ Callback for MessageProcessor to remove a parameter.
            
            @param name:    Name which is used to identify the parameter which should
                            be removed.
            @type  name:    str
        """
        log.msg('Remove parameter "{0}".'.format(name))
        
        try:
            self._parameters.pop(name).remove()
        except KeyError:
            pass
    
    def sendROSMessage(self, rosMsg, dest, tag, user, uid):
        """ Send a ROS message to the specified destination.
                            
            @param rosMsg:  ROS Message which should be sent in serialized form.
            @type  rosMsg:  str
            
            @param dest:    Communication ID of destination.
            @type  dest:    str
            
            @param tag:     Interface tag which should be used to send the message
            @type  tag:     str
            
            @param user:    Identifier for the user responsible for sending the message
            @type  user:    str
            
            @param uid:     Unique ID to identify the message
            @type  uid:     str
        """
        fifo = MessageFIFO()
        fifo.push(rosMsg)
        
        msg = Message()
        msg.msgType = MsgTypes.ROS_MSG
        msg.dest = dest
        msg.content = { 'msg'  : fifo,
                        'tag'  : tag,
                        'user' : user,
                        'push' : True,
                        'uid'  : uid }
        self._commManager.sendMessage(msg)

    def shutdown(self):
        """ Method is called when the manager is stopped.
        """
        self._interfaces = {}
        self._parameters = {}
