#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       SatelliteTriggers.py
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
from Comm.Message import  MsgTypes
from Comm.Message.Base import Message
from Comm.Message.Interfaces import IPostInitTrigger

class DefaultRoutingTrigger(object):
    """ PostInitTrigger which is used to send the CommID of this node as default routing info.
    """
    implements(IPostInitTrigger)
    
    def __init__(self, commMngr):
        """ Initialize the DefaultRoutingTrigger.

            @param commMngr:    CommManager which is responsible for handling the communication
                                in this node.
            @type  commMngr:    CommManager
        """
        self.commManager = commMngr
    
    def trigger(self, origin):
        msg = Message()
        msg.msgType = MsgTypes.ROUTE_INFO
        msg.dest = origin
        msg.content = { None : self.commManager.commID }
        self.commManager.sendMessage(msg)

class SatelliteRoutingTrigger(object):
    """ PostInitTrigger which is used to send the available container IDs as routing info.
    """
    implements(IPostInitTrigger)
    
    def __init__(self, commMngr, satelliteMngr):
        """ Initialize the DefaultRoutingTrigger.

            @param commMngr:    CommManager which is responsible for handling the communication
                                in this node.
            @type  commMngr:    CommManager
            
            @param satelliteMngr:   SatelliteManager which is handles this node.
            @type  satelliteMngr:   SatelliteManager
        """
        self.commManager = commMngr
        self.satelliteManager = satelliteMngr
    
    def trigger(self, origin):
        msg = Message()
        msg.msgType = MsgTypes.ROUTE_INFO
        msg.dest = origin
        msg.content = self.satelliteManager.getSatelliteRouting()
        self.commManager.sendMessage(msg)
