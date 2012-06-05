#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Processor.py
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

# zope specific imports
from zope.interface import implements

# twisted specific imports
from twisted.python import log

# Custom imports
from Exceptions import InvalidRequest, InternalError
from Comm.Message.Interfaces import IMessageProcessor
from Comm.Message import MsgDef
from Comm.Message import MsgTypes
from Comm.Message.Base import Message

from ROSComponents import ComponentDefinition
from ROSComponents.Node import NodeForwarder
from ROSComponents.InterfaceMonitor import ServiceMonitor, PublisherMonitor, SubscriberMonitor
from ROSComponents.ParameterMonitor import IntMonitor, StrMonitor, FloatMonitor, BoolMonitor, FileMonitor

class ROSProcessorBase(object):
    """ Message processor base class for ROS Processors.
    """
    implements(IMessageProcessor)
    
    def __init__(self, manager, commManager):
        """ @param manager:     ROSManager which is used in this node.
            @type  manager:     ROSManager
            
            @param commManager:     CommManager which is used in this node.
            @type  commManager:     CommManager
        """
        self.manager = manager
        self.commManager = commManager

class ROSAddProcessor(ROSProcessorBase):
    """ Message processor to add a single component which can be a node, interface
        or parameter.
    """
    IDENTIFIER = MsgTypes.ROS_ADD
    
    _INTERFACES = { ServiceMonitor.IDENTIFIER    : ServiceMonitor,
                    PublisherMonitor.IDENTIFIER  : PublisherMonitor,
                    SubscriberMonitor.IDENTIFIER : SubscriberMonitor }
    _PARAMETERS = { IntMonitor.IDENTIFIER   : IntMonitor,
                    StrMonitor.IDENTIFIER   : StrMonitor,
                    FloatMonitor.IDENTIFIER : FloatMonitor,
                    BoolMonitor.IDENTIFIER  : BoolMonitor,
                    FileMonitor.IDENTIFIER  : FileMonitor }
    
    def processMessage(self, msg):
        msg = msg.content
        
        if isinstance(msg, NodeForwarder):
            fwdMsg = Message()
            fwdMsg.msgType = MsgTypes.ROS_ADD
            fwdMsg.dest = MsgDef.LAUNCHER_ADDR
            fwdMsg.content = msg
            self.commManager.sendMessage(fwdMsg)
        elif msg.IDENTIFIER in self._INTERFACES:
            self._INTERFACES[msg.IDENTIFIER](msg, self.manager)
        elif msg.IDENTIFIER in self._PARAMETERS:
            self._PARAMETERS[msg.IDENTIFIER](msg, self.manager)
        else:
            raise InvalidRequest('Unknown component to add received.')

class ROSRemoveProcessor(ROSProcessorBase):
    """ Message processor to remove a single component which can be a node, interface
        or parameter.
    """
    IDENTIFIER = MsgTypes.ROS_REMOVE
    
    def processMessage(self, msg):
        msg = msg.content
        rmType = msg['type']
        
        if rmType == ComponentDefinition.RM_NODE:
            fwdMsg = Message()
            fwdMsg.msgType = MsgTypes.ROS_REMOVE
            fwdMsg.dest = MsgDef.LAUNCHER_ADDR
            fwdMsg.content = msg
            self.commManager.sendMessage(fwdMsg)
        elif rmType == ComponentDefinition.RM_INTERFACE:
            self.manager.removeInterface(msg['tag'])
        elif rmType == ComponentDefinition.RM_PARAMETER:
            self.manager.removeParameter(msg['tag'])
        else:
            raise InvalidRequest('Unknown type to remove received.')

class ROSMessageContainerProcessor(ROSProcessorBase):
    """ Message processor for a single ROS message in the container node.
    """
    IDENTIFIER = MsgTypes.ROS_MSG
    
    def processMessage(self, msg):
        rosMsg = msg.content['msg']
        interface = self.manager.getInterface(msg.content['name'])
        
        if msg.content['push']:
            try:
                interface.send(rosMsg, (msg.origin, msg.content['uid']))
            except InternalError as e:
                log.msg('Could not send ROS Message: {0}'.format(e))
        else:
            respMsg = Message()
            respMsg.msgType = MsgTypes.ROS_RESPONSE
            respMsg.dest = msg.origin
            
            try:
                respMsg.content = { 'msg' : interface.send(rosMsg), 'error' : False }
            except InternalError as e:
                log.msg('Could not send ROS Message: {0}'.format(e))
                respMsg.content = { 'msg' : str(e), 'error' : True }
            
            self.commManager.sendMessage(respMsg)

### TODO: Not used for pure push implementation
class ROSGetProcessor(ROSProcessorBase):
    """ Message processor for a single ROS message request.
    """
    IDENTIFIER = MsgTypes.ROS_GET
    
    def processMessage(self, msg):
        interface = self.manager.getInterface(msg.content['name'])
        
        respMsg = Message()
        respMsg.msgType = MsgTypes.ROS_MSG
        respMsg.dest = msg.origin
        
        try:
            respMsg.content = { 'msg' : interface.receive(msg.content['uid']),
                                'name' : '',
                                'uid' : '#TODO: What is a good value here?',
                                'push' : False }
        except InternalError as e:
            log.msg('Could not send ROS Message: {0}'.format(e))
        
        self.commManager.sendMessage(respMsg)
