#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       StdProcessor.py
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

# Custom imports
from ProcessorBase import ProcessorBase
from TypeBase import MessageTypes as MsgTypes

class RouteProcessor(ProcessorBase):
    """ Message processor to update the routing information.
    """
    IDENTIFIER = MsgTypes.ROUTE_INFO
    
    def processMessage(self, msg):
        self.manager.updateRoutingInfo(msg.content)

class ConnectDirectiveProcessor(ProcessorBase):
    """ Message processor which executes the directives from the message and connects
        to all specified nodes.
    """
    IDENTIFIER = MsgTypes.CONNECT
    
    def processMessage(self, msg):
        self.manager.connectToSatellites(msg)

class LoadInfoProcessor(ProcessorBase):
    """ Message processor to update the load information with the load balancer.
    """
    IDENTIFIER = MsgTypes.LOAD_INFO
    
    def processMessage(self, msg):
        pass # TODO: Add
    