#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Processor.py
#       
#       This file is part of the RoboEarth Cloud Engine framework.
#       
#       This file was originally created for RoboEearth
#       http://www.roboearth.org/
#       
#       The research leading to these results has received funding from
#       the European Union Seventh Framework Programme FP7/2007-2013 under
#       grant agreement no248942 RoboEarth.
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
#       \author/s: Dominique Hunziker 
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
            self.manager.addInterface(
                self._INTERFACES[msg.IDENTIFIER]( self.manager,
                                                  self.commManager.reactor,
                                                  msg )
            )
        elif msg.IDENTIFIER in self._PARAMETERS:
            self.manager.addParameter(self._PARAMETERS[msg.IDENTIFIER](msg))
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

class ROSUserProcessor(ROSProcessorBase):
    """ Message processor to add a new user for an interface.
    """
    IDENTIFIER = MsgTypes.ROS_USER
    
    def processMessage(self, msg):
        msg = msg.content
        
        if msg['add']:
            self.manager.addInterfaceUser(msg['tag'], msg['target'], msg['commID'])
        else:
            self.manager.removeInterfaceUser(msg['tag'], msg['target'], msg['commID'])

class ROSMessageContainerProcessor(ROSProcessorBase):
    """ Message processor for a single ROS message in the container node.
    """
    IDENTIFIER = MsgTypes.ROS_MSG
    
    def processMessage(self, msg):
        content = msg.content
        interface = self.manager.getInterface(content['tag'])
        
        try:
            interface.send(content['msg'], content['uid'], msg.origin, content['user'], content['push'])
        except InternalError as e:
            log.msg('Could not send ROS Message: {0}'.format(e))

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
