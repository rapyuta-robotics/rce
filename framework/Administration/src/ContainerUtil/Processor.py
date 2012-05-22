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

# twisted specific imports
from zope.interface import implements

# Custom imports
from Comm.Message.Interfaces import IMessageProcessor #@UnresolvedImport
from Comm.Message import MsgTypes

class ContainerProcessorBase(object):
    """ Base class for all container processors.
    """
    implements(IMessageProcessor)
    
    def __init__(self, manager):
        """ @param manager:     ContainerManager which is used in this node.
            @type  manager:     ContainerManager
        """
        self.manager = manager

class StartContainerProcessor(ContainerProcessorBase):
    """ Message processor to start a container.
    """
    IDENTIFIER = MsgTypes.CONTAINER_START
    
    def processMessage(self, msg):
        self.manager.startContainer(msg['commID'], msg['ip'], msg['homeDir'], msg['key'])

class StopContainerProcessor(ContainerProcessorBase):
    """ Message processor to stop a container.
    """
    IDENTIFIER = MsgTypes.CONTAINER_STOP
    
    def processMessage(self, msg):
        self.manager.stopContainer(msg['commID'])
