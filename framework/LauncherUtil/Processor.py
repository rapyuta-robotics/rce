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
from Exceptions import InvalidRequest
from Comm.Message.Interfaces import IMessageProcessor
from Comm.Message import MsgTypes

from ROSComponents import ComponentDefinition

class LauncherProcessorBase(object):
    """ Message processor base class for Launcher Processors.
    """
    implements(IMessageProcessor)
    
    def __init__(self, manager):
        """ @param manager:     LauncherManager which is used in this node.
            @type  manager:     LauncherManager
        """
        self.manager = manager

class ROSAddProcessor(LauncherProcessorBase):
    """ Message processor to add a single node.
    """
    IDENTIFIER = MsgTypes.ROS_ADD
    
    def processMessage(self, msg):
        msg = msg.content
        
        if msg.IDENTIFIER != ComponentDefinition.NODE:
            raise InvalidRequest('Unknown type to add received.')
        
        self.manager.addNode(msg)

class ROSRemoveProcessor(LauncherProcessorBase):
    """ Message processor to remove a single node.
    """
    IDENTIFIER = MsgTypes.ROS_REMOVE
    
    def processMessage(self, msg):
        msg = msg.content
        
        if msg['type'] != ComponentDefinition.RM_NODE:
            raise InvalidRequest('Unknown type to remove received.')
        
        self.manager.removeNode(msg['tag'])
