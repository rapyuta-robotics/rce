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
