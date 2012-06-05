#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Triggers.py
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
from Comm.Message import  MsgTypes
from Comm.Message.Base import Message
from Comm.Interfaces import IPostInitTrigger

class CommonRoutingTrigger(object):
    """ Class which implements a common method for the triggers.
    """
    implements(IPostInitTrigger)
    
    def __init__(self, commMngr):
        """ Initialize the CommonRoutingTrigger.

            @param commMngr:    CommManager which is responsible for handling the communication
                                in this node.
            @type  commMngr:    CommManager
        """
        self.commManager = commMngr
    
    def sendRoutingInfo(self, origin, info):
        """ Internally used method.
        """
        msg = Message()
        msg.msgType = MsgTypes.ROUTE_INFO
        msg.dest = origin
        msg.content = info
        self.commManager.sendMessage(msg)

class BaseRoutingTrigger(CommonRoutingTrigger):
    """ PostInitTrigger which is used to send the CommID of this node as default routing info.
    """
    def trigger(self, origin, ip):
        self.sendRoutingInfo(origin, [ (None, True) ])

class SatelliteRoutingTrigger(CommonRoutingTrigger):
    """ PostInitTrigger which is used to send the available container IDs as routing info.
    """
    def __init__(self, commMngr, satelliteMngr):
        """ Initialize the SatelliteRoutingTrigger.

            @param commMngr:    CommManager which is responsible for handling the communication
                                in this node.
            @type  commMngr:    CommManager
            
            @param satelliteMngr:   SatelliteManager which is responsible for the handling of
                                    this node.
            @type  satelliteMngr:   SatelliteManager
        """
        super(SatelliteRoutingTrigger, self).__init__(commMngr)
        
        self.satelliteManager = satelliteMngr
    
    def trigger(self, origin, ip):
        self.sendRoutingInfo(origin, self.satelliteManager.getSatelliteRouting())

class EnvironmentRoutingTrigger(CommonRoutingTrigger):
    """ PostInitTrigger which is used to send the CommID of this node as default routing info
        and additionally add set the container to connected.
    """
    def __init__(self, commMngr, satelliteMngr):
        """ Initialize the SatelliteRoutingTrigger.

            @param commMngr:    CommManager which is responsible for handling the communication
                                in this node.
            @type  commMngr:    CommManager
            
            @param satelliteMngr:   SatelliteManager which is responsible for the handling of
                                    this node.
            @type  satelliteMngr:   SatelliteManager
        """
        super(EnvironmentRoutingTrigger, self).__init__(commMngr)
        
        self.satelliteManager = satelliteMngr
    
    def trigger(self, origin, ip):
        self.sendRoutingInfo(origin, [ (None, True) ])
        self.satelliteManager.setConnectedFlagContainer(origin, True)
