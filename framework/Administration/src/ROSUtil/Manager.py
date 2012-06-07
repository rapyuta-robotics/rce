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
from Exceptions import InternalError
from Comm.Message.Base import Message
from Comm.Message import MsgTypes
from Type import ROSAddMessage, ROSRemoveMessage, ROSAddUserMessage, ROSRemoveUserMessage, ROSMsgMessage #, ROSResponseMessage, ROSGetMessage
from Processor import ROSAddProcessor, ROSRemoveProcessor, ROSAddUserProcessor, ROSRemoveUserProcessor, ROSMessageContainerProcessor #, ROSGetProcessor

from ROSComponents.Node import NodeForwarder
from ROSComponents.Interface import ServiceInterface, PublisherInterface, SubscriberInterface
from ROSComponents.Parameter import IntParam, StrParam, FloatParam, BoolParam, FileParam

class ROSManager(object):
    """ Manager which handles ROS specific tasks.
    """
    def __init__(self, commMngr):
        """ Initialize the ROSManager.
            
            @param commMngr:    CommManager which should be used to communicate.
            @type  commMngr:    CommManager
        """
        # References used by the manager
        self._commMngr = commMngr
        
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
        self._commMngr.registerContentSerializers([ rosAdd,
                                                    ROSRemoveMessage(),
                                                    ROSAddUserMessage(),
                                                    ROSRemoveUserMessage(),
                                                    ROSMsgMessage() ])
        # ROSResponseMessage(), ROSGetMessage()
        
        # Register Message Processors
        self._commMngr.registerMessageProcessors([ ROSAddProcessor(self, commMngr),
                                                   ROSRemoveProcessor(self, commMngr),
                                                   ROSAddUserProcessor(self, commMngr),
                                                   ROSRemoveUserProcessor(self, commMngr),
                                                   ROSMessageContainerProcessor(self, commMngr) ])
        # ROSGetProcessor(self, commMngr)
    
    def addInterface(self, interface):
        """ Callback for InterfaceMonitor instance to register the interface.

            @param interface:   InterfaceMonitor which should be registered.
            @type  interface:   InterfaceMonitor

            @raise:     InternalError if there is already an interface
                        with the same tag registered.
        """
        tag = interface.tag

        if tag in self._interfaces:
            raise InternalError('There is already an interface with the same tag.')

        self._interfaces[tag] = interface
    
    def removeInterface(self, tag):
        """ Callback for MessageProcessor to remove an interface.
        """
        self._interfaces.pop(tag, None)
    
    def getInterface(self, tag):
        """ Get the interface matching the given tag.
            
            @raise:     InternalError if there is no interface matching the given tag.
        """
        if tag not in self._interfaces:
            raise InternalError('There is no interface with the tag "{0}".'.format(tag))
        
        return self._interfaces[tag]
    
    def addInterfaceUser(self, tag, target, commID):
        """ # TODO: Add description.
        """
        self.getInterface(tag).addPushReceiver(commID, target)
    
    def removeInterfaceUser(self, tag, target, commID):
        """ # TODO: Add description.
        """
        self.getInterface(tag).removePushReceiver(commID, target)
    
    def addParameter(self, parameter):
        """ Callback for Parameter instance to register the parameter.
        """
        self._parameters[parameter.name] = parameter
    
    def removeParameter(self, name):
        """ Callback for MessageProcessor to remove a parameter.
        """
        self._parameters.pop(name, None)
    
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
        msg = Message()
        msg.msgType = MsgTypes.ROS_MSG
        msg.dest = dest
        msg.content = { 'msg'  : rosMsg,
                        'tag'  : tag,
                        'user' : user,
                        'push' : False,
                        'uid'  : uid }
        self._commMngr.sendMessage(msg)

    def runTaskInSeparateThread(self, func, *args, **kw):
        """ Convenience method to run any function in a separate thread.

            This method should not be overwritten.

            @param func:    Callable which should be executed in separate thread.
            @type  func:    callable

            @param *args:   Any positional arguments which will be passed to 'func'.

            @param *kw:     Any keyworded arguments which will be passed to 'func'.
        """
        self._commMngr.reactor.callInThread(func, args, kw)

    def shutdown(self):
        """ Method is called when the manager is stopped.
        """
        self._interfaces = {}
        self._parameters = {}
