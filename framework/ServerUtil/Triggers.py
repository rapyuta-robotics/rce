#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Triggers.py
#       
#       This file is part of the RoboEarth Cloud Engine framework.
#       
#       This file was originally created for RoboEearth
#       http://www.roboearth.org/
#       
#       The research leading to these results has received funding from
#       the European Union Seventh Framework Programme FP7/2007-2013 under
#       grant agreement no248942 RoboEarth.
#       
#       Copyright 2012 RoboEarth
#       
#       Licensed under the Apache License, Version 2.0 (the "License");
#       you may not use this file except in compliance with the License.
#       You may obtain a copy of the License at
#       
#       http://www.apache.org/licenses/LICENSE-2.0
#       
#       Unless required by applicable law or agreed to in writing, software
#       distributed under the License is distributed on an "AS IS" BASIS,
#       WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#       See the License for the specific language governing permissions and
#       limitations under the License.
#       
#       \author/s: Dominique Hunziker 
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

class ServerRoutingTrigger(CommonRoutingTrigger):
    """ PostInitTrigger which is used to send the available container IDs as routing info.
    """
    def __init__(self, commMngr, serverMngr):
        """ Initialize the ServerRoutingTrigger.

            @param commMngr:    CommManager which is responsible for handling the communication
                                in this node.
            @type  commMngr:    CommManager
            
            @param serverMngr:   ServerManager which is responsible for the handling of
                                    this node.
            @type  serverMngr:   ServerManager
        """
        super(ServerRoutingTrigger, self).__init__(commMngr)
        
        self.serverManager = serverMngr
    
    def trigger(self, origin, ip):
        self.sendRoutingInfo(origin, self.serverManager.getServerRouting())

class EnvironmentRoutingTrigger(CommonRoutingTrigger):
    """ PostInitTrigger which is used to send the CommID of this node as default routing info
        and additionally add set the container to connected.
    """
    def __init__(self, commMngr, serverMngr):
        """ Initialize the ServerRoutingTrigger.

            @param commMngr:    CommManager which is responsible for handling the communication
                                in this node.
            @type  commMngr:    CommManager
            
            @param serverMngr:   ServerManager which is responsible for the handling of
                                    this node.
            @type  serverMngr:   ServerManager
        """
        super(EnvironmentRoutingTrigger, self).__init__(commMngr)
        
        self.serverManager = serverMngr
    
    def trigger(self, origin, ip):
        self.sendRoutingInfo(origin, [ (None, True) ])
        self.serverManager.setConnectedFlagContainer(origin, True)
