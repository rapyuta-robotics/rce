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
        self.manager.connectToSatellites(msg.content)

class GetCommIDProcessor(SatelliteProcessorBase):
    """ Message processor for a getCommID request.
    """
    IDENTIFIER = MsgTypes.ID_REQUEST
    
    def processMessage(self, msg):
        self.manager.setNewCommId(msg.content['commID'])

class LoadInfoProcessor(SatelliteProcessorBase):
    """ Message processor to update the load information with the load balancer.
    """
    IDENTIFIER = MsgTypes.LOAD_INFO
    
    def processMessage(self, msg):
        pass # TODO: Add

class ROSMsgProcessor(SatelliteProcessorBase):
    """ Message processor to handle and convert a ROS message for forwarding to robot.
    """
    IDENTIFIER = MsgTypes.ROS_MSG
    
    def processMessage(self, msg):
        self.manager.sendROSMsgToRobot( msg.content['uid'],
                                        msg.origin,
                                        msg.content['name'],
                                        msg.content['msg'] )

### TODO: Not used
class ContainerStatusProcessor(SatelliteProcessorBase):
    """ # TODO: Add description
    """
    IDENTIFIER = MsgTypes.CONTAINER_STATUS
    
    def processMessage(self, msg):
        pass # TODO: Add
