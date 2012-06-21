#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Processor.py
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

# zope specific imports
from zope.interface import implements

# Custom imports
from Comm.Message.Interfaces import IMessageProcessor
from Comm.Message import MsgTypes
from Comm.Message.Base import Message

class MasterProcessorBase(object):
    """ Base message processor for Master node.
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

class GetCommIDProcessor(MasterProcessorBase):
    """ Message processor for a getCommID request.
    """
    IDENTIFIER = MsgTypes.ID_REQUEST
    
    def processMessage(self, msg):
        resp = Message()
        resp.msgType = MsgTypes.ID_RESPONSE
        resp.dest = msg.origin
        resp.content = { 'commID' : self.manager.getCommID() }
        self.commManager.sendMessage(resp)

class DelCommIDProcessor(MasterProcessorBase):
    """ Message processor for a delCommID request.
    """
    IDENTIFIER = MsgTypes.ID_DEL
    
    def processMessage(self, msg):
        self.manager.delCommID(msg.content['commID'])
