#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       SatelliteProcessor.py
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

class SatelliteProcessorBase(object):
    """ Base class for all standard processors.
    """
    implements(IMessageProcessor)
    
    def __init__(self, manager):
        """ @param manager:     SatelliteManager which is used in this node.
            @type  manager:     SatelliteManager
        """
        self.manager = manager

class ConnectDirectiveProcessor(SatelliteProcessorBase):
    """ Message processor which executes the directives from the message and connects
        to all specified nodes.
    """
    IDENTIFIER = MsgTypes.CONNECT
    
    def processMessage(self, msg):
        self.manager.connectToSatellites(msg)

class LoadInfoProcessor(SatelliteProcessorBase):
    """ Message processor to update the load information with the load balancer.
    """
    IDENTIFIER = MsgTypes.LOAD_INFO
    
    def processMessage(self, msg):
        pass # TODO: Add

class CreateEnvProcessor(SatelliteProcessorBase):
    """ Message processor to create an environment.
    """
    IDENTIFIER = MsgTypes.ENV_CREATE
    
    def processMessage(self, msg):
        self.manager.createContainer(msg.content['commID'], msg.content['home'])

class DestroyEnvProcessor(SatelliteProcessorBase):
    """ Message processor to destroy an environment.
    """
    IDENTIFIER = MsgTypes.ENV_DESTROY
    
    def processMessage(self, msg):
        self.manager.destroyContainer(msg.content['commID'])

class ContainerStatusProcessor(SatelliteProcessorBase):
    """ Message processor to handle container status information.
    """
    IDENTIFIER = MsgTypes.CONTAINER_STATUS
    
    def processMessage(self, msg):
        pass # TODO: Add necessary code here
