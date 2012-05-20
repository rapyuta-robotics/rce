#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       ContainerManager.py
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
from twisted.internet.ssl import ClientContextFactory
from twisted.internet.task import LoopingCall

# Python specific imports
import sys

# Custom imports
from Comm.Message import MsgDef
from Comm.Message import MsgTypes
from Comm.Message.Base import Message
from Comm.Factory import ReappengineServerFactory
from Comm.Message.ContainerType import StartContainerMessage, StopContainerMessage, ContainerStatusMessage #@UnresolvedImport
from Comm.Message.ContainerProcessor import StartContainerProcessor, StopContainerProcessor #@UnresolvedImport

class ContainerServerFactory(ReappengineServerFactory):
    """ Factory which is used in the container manager node for the connection to
        the satellite node.
    """
    def filterMessage(self, msgType):
        return msgType not in [ MsgTypes.ROUTE_INFO,
                                MsgTypes.CONTAINER_START,
                                MsgTypes.CONTAINER_STOP ]
    
    def postInitTrigger(self, origin):
        msg = Message()
        msg.msgType = MsgTypes.ROUTE_INFO
        msg.dest = origin
        msg.content = { None : MsgDef.NEIGHBOR_ADDR }
        self.manager.sendMessage(msg)

class ContainerManager(object):
    """ Manager which handles container specific task.
    """
    def __init__(self, commMngr):
        """ Initialize the ContainerManager.
            
            @param commMngr:    CommManager which should be used to communicate.
            @type  commMngr:    CommManager
        """
        # References used by the container manager
        self._commMngr = commMngr
        
        # Register Content Serializers
        self.registerContentSerializer(StartContainerMessage())
        self.registerContentSerializer(StopContainerMessage())
        self.registerContentSerializer(ContainerStatusMessage())
        
        # Register Message Processors
        self._commMngr.registerMessageProcessor(StartContainerProcessor(self))
        self._commMngr.registerMessageProcessor(StopContainerProcessor(self))
    
    
