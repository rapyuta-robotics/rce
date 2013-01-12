#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     environment.py
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

# twisted specific imports
from twisted.internet.address import IPv4Address
from twisted.spread.pb import DeadReferenceError

# Custom imports
from rce.master.base import Proxy
from rce.master.network import Endpoint, Namespace


class Node(Proxy):
    """
    """
    def __init__(self, namespace):
        """
        """
        super(Node, self).__init__()
        
        self._namespace = namespace
        namespace.registerNode(self)
    
    def destroy(self):
        """
        """
        self._namespace.unregisterNode(self)
        self._namespace = None
        
        super(Node, self).destroy()
    
    @Proxy.destroyProxy
    def _destroy(self):
        """
        """
        try:
            self.obj.callRemote('destroy')
        except DeadReferenceError:
            pass


class Parameter(Proxy):
    """
    """
    def __init__(self, namespace):
        """
        """
        super(Parameter, self).__init__()
        
        self._namespace = namespace
        namespace.registerParameter(self)
    
    def destroy(self):
        """
        """
        self._namespace.unregisterParameter(self)
        self._namespace = None
        
        super(Parameter, self).destroy()
    
    @Proxy.destroyProxy
    def _destroy(self):
        """
        """
        try:
            self.obj.callRemote('destroy')
        except DeadReferenceError:
            pass


class Environment(Namespace):
    """
    """
    def __init__(self, endpoint):
        """
        """
        super(Environment, self).__init__(endpoint)
        
        self._nodes = set()
        self._parameters = set()
    
    @Proxy.returnProxy(Node)
    def createNode(self, pkg, exe, args, name, nspace):
        """
        """
        return self.obj.callRemote('createNode', pkg, exe, args, name, nspace)
    
    @Proxy.returnProxy(Parameter)
    def createParameter(self, name, value):
        """
        """
        return self.obj.callRemote('createParameter', name, value)
    
    def registerNode(self, node):
        assert node not in self._nodes
        self._nodes.add(node)
    
    def registerParameter(self, parameter):
        assert parameter not in self._parameters
        self._parameters.add(parameter)
    
    def unregisterNode(self, node):
        assert node in self._nodes
        self._nodes.remove(node)
    
    def unregisterParameter(self, parameter):
        assert parameter in self._parameters
        self._parameters.remove(parameter)
    
    def destroy(self):
        """
        """
        for node in self._nodes.copy():
            node.destroy()
        
        for parameter in self._parameters.copy():
            parameter.destroy()
        
        assert len(self._nodes) == 0
        assert len(self._parameters) == 0
        
        super(Environment, self).destroy()


class EnvironmentEndpoint(Endpoint):
    """
    """
    def __init__(self, network, container):
        """
        """
        super(EnvironmentEndpoint, self).__init__(network)
        
        self._container = container
    
    # TODO: At the moment single machine fix for getAddress
    def getAddress(self):
        """
        """
        import settings
        d = self._getAddress()
        d.addCallback(lambda addr: IPv4Address('TCP', addr.host,
                                               settings.RCE_INTERNAL_PORT))
        return d
    
    @Proxy.returnDeferred
    def _getAddress(self):
        """
        """
        return self.obj.broker.transport.getPeer()
    
#    def getAddress(self):
#        """
#        """
#        return self._container.getAddress()
    
    @Proxy.returnProxy(Environment)
    def createNamespace(self):
        """
        """
        return self.obj.callRemote('createNamespace')
    
    def destroy(self):
        """
        """
        if self._container:
            self._container.destroy()
            self._container = None
            
            super(EnvironmentEndpoint, self).destroy()
        else:
            print('environment.EnvironmentEndpoint destroy() called '
                  'multiple times...')
