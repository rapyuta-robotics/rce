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

# zope specific imports
from zope.interface import implements

# Custom imports
from Interfaces import IMessageProcessor #@UnresolvedImport
import MsgTypes

class StdProcessorBase(object):
    """ Base class for all standard processors.
    """
    implements(IMessageProcessor)
    
    def __init__(self, manager):
        """ @param manager:     CommManager which is used in this node.
            @type  manager:     CommManager
        """
        self.manager = manager

class RouteProcessor(StdProcessorBase):
    """ Message processor to update the routing information.
    """
    IDENTIFIER = MsgTypes.ROUTE_INFO
    
    def processMessage(self, msg):
        self.manager.router.updateRoutingInfo(msg.origin, msg.content)
