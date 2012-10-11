#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     callback.py
#     
#     This file is part of the RoboEarth Cloud Engine framework.
#     
#     This file was originally created for RoboEearth
#     http://www.roboearth.org/
#     
#     The research leading to these results has received funding from
#     the European Union Seventh Framework Programme FP7/2007-2013 under
#     grant agreement no248942 RoboEarth.
#     
#     Copyright 2012 RoboEarth
#     
#     Licensed under the Apache License, Version 2.0 (the "License");
#     you may not use this file except in compliance with the License.
#     You may obtain a copy of the License at
#     
#     http://www.apache.org/licenses/LICENSE-2.0
#     
#     Unless required by applicable law or agreed to in writing, software
#     distributed under the License is distributed on an "AS IS" BASIS,
#     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#     See the License for the specific language governing permissions and
#     limitations under the License.
#     
#     \author/s: Dominique Hunziker 
#     
#     

# zope specific imports
from zope.interface import implements

# Custom imports
from comm import types as msgTypes
from comm.interfaces import IPostInit, IPostClose
from comm.message import Message


class _RoutingCallbackBase(object):
    """ Base class for all callbacks which include sending routing info in
        post initialization.
    """
    def __init__(self, commMngr):
        """ Initialize the relay callback base.
            
            @param commMngr:    CommManager used in this process.
            @type  commMngr:    comm.manager.CommManager
        """
        self._commManager = commMngr
    
    def sendRoutingInfo(self, dest, info):
        """ Internally used method to send routing info "info" to "dest".
        """
        if not info:
            # Only send a message if there is information to send
            return
        
        msg = Message()
        msg.msgType = msgTypes.ROUTE_INFO
        msg.dest = dest
        msg.content = info
        self._commManager.sendMessage(msg)
        

class RelayCallbackFromEndpoint(_RoutingCallbackBase):
    """ # TODO: Add description
    """
    implements(IPostInit, IPostClose)
    
    def __init__(self, manager, commMngr):
        """ # TODO: Add description
        """
        super(RelayCallbackFromEndpoint, self).__init__(commMngr)
        
        self._manager = manager
    
    def postInit(self, origin, _1, _2):
        """ This method is called when the connection has been initialized.
            
            @param origin:  CommID of initialized connection.
            @type  origin:  str
        """
        self._manager.registerEndpoint(origin)
        self.sendRoutingInfo(origin, [(None, True)])
    
    def postClose(self, origin, _):
        """ This method is called when the connection has been initialized.
            
            @param origin:  CommID of closed connection.
            @type  origin:  str
        """
        self._manager.unregisterEndpoint(origin)


class RelayCallbackFromRelay(_RoutingCallbackBase):
    """ # TODO: Add description
    """
    implements(IPostInit, IPostClose)
    
    def __init__(self, manager, commMngr):
        """ # TODO: Add description
        """
        super(RelayCallbackFromRelay, self).__init__(commMngr)
        
        self._manager = manager
    
    def postInit(self, origin, _1, _2):
        """ This method is called when the connection has been initialized.
            
            @param origin:  CommID of initialized connection.
            @type  origin:  str
        """
        self._manager.registerRelay(origin)
        self.sendRoutingInfo(origin, self._manager.getEndpointRouting())
    
    def postClose(self, origin, _):
        """ This method is called when the connection has been initialized.
            
            @param origin:  CommID of closed connection.
            @type  origin:  str
        """
        self._manager.unregisterRelay(origin)


class MasterCallbackFromRelay(object):
    """ # TODO: Add description
    """
    implements(IPostInit, IPostClose)
    
    def __init__(self, machineCache):
        """ # TODO: Add description
        """
        self._machineCache = machineCache
    
    def postInit(self, origin, ip, data):
        """ This method is called when the connection has been initialized.
            
            @param origin:  CommID of initialized connection.
            @type  origin:  str
            
            @param ip:      IP address of initialized connection.
            @type  ip:      str
            
            @param data:    Data which was sent with the INIT message.
            @type  data:    str
        """
        self._machineCache.addRelay(ip, data[0], origin)
    
    def postClose(self, origin, ip):
        """ This method is called when the connection has been initialized.
            
            @param origin:  CommID of closed connection.
            @type  origin:  str
            
            @param ip:      IP address of closed connection.
            @type  ip:      str
        """
        self._machineCache.removeRelay(ip, origin)


class MasterCallbackFromContainer(object):
    """ # TODO: Add description
    """
    implements(IPostInit, IPostClose)
    
    def __init__(self, machineCache):
        """ # TODO: Add description
        """
        self._machineCache = machineCache
    
    def postInit(self, origin, ip, _):
        """ This method is called when the connection has been initialized.
            
            @param origin:  CommID of initialized connection.
            @type  origin:  str
            
            @param ip:      IP address of initialized connection.
            @type  ip:      str
        """
        self._machineCache.addContainer(ip, origin)
    
    def postClose(self, origin, ip):
        """ This method is called when the connection has been initialized.
            
            @param origin:  CommID of closed connection.
            @type  origin:  str
            
            @param ip:      IP address of closed connection.
            @type  ip:      str
        """
        self._machineCache.removeContainer(ip, origin)
