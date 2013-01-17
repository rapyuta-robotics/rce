#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     container.py
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
#     Copyright 2013 RoboEarth
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

# twisted specific imports
from twisted.internet.address import IPv4Address
from twisted.internet.defer import Deferred

# Custom imports
from rce.master.base import Proxy


class Container(Proxy):
    """ Representation of an LXC container.
    """
    def __init__(self, machine):
        """ Initialize the Container.
            
            @param machine:     Machine in which the container was created.
            @type  machine:     rce.master.machine.Machine
        """
        super(Container, self).__init__()
        
        self._machine = machine
        machine.registerContainer(self)
    
    def getAddress(self):
        """ Get the address which should be used to connect to the environment
            process for the cloud engine internal communication.
            
            @return:            twisted::IPv4Address which can be used to
                                connect to the ServerFactory of the cloud
                                engine internal communication protocol.
            @rtype:             twisted::Deferred
        """
        d = self.callRemote('getPort', Deferred)
        d.addCallback(lambda port: IPv4Address('TCP', self._machine._ip, port))
        return d
    
    def destroy(self):
        """ Method should be called to destroy the container and will take care
            of deleting all circular references.
        """
        if self._machine:
            self._machine.unregisterContainer(self)
            self._machine = None
            
            super(Container, self).destroy()
        else:
            print('machine.Container destroy() called multiple times...')
