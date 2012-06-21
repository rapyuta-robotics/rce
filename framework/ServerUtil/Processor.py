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

class ServerProcessorBase(object):
    """ Base class for all standard processors.
    """
    implements(IMessageProcessor)
    
    def __init__(self, manager):
        """ @param manager:     ServerManager which is used in this node.
            @type  manager:     ServerManager
        """
        self.manager = manager

class ConnectDirectiveProcessor(ServerProcessorBase):
    """ Message processor which executes the directives from the message and connects
        to all specified nodes.
    """
    IDENTIFIER = MsgTypes.CONNECT
    
    def processMessage(self, msg):
        self.manager.connectToServers(msg.content)

class GetCommIDProcessor(ServerProcessorBase):
    """ Message processor for a getCommID request.
    """
    IDENTIFIER = MsgTypes.ID_RESPONSE
    
    def processMessage(self, msg):
        self.manager.setNewCommID(msg.content['commID'])

class ROSMsgProcessor(ServerProcessorBase):
    """ Message processor to handle and convert a ROS message for forwarding to robot.
    """
    IDENTIFIER = MsgTypes.ROS_MSG
    
    def processMessage(self, msg):
        self.manager.receivedROSMessage(msg)
