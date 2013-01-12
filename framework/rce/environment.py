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

# Python specific imports
from uuid import UUID

# ROS specific imports
import rospy

# twisted specific imports
from twisted.python import log
from twisted.spread.pb import PBClientFactory

# Custom imports
from rce.error import InternalError
from rce.util.loader import Loader
from rce.monitor.node import Node
from rce.monitor.parameter import Parameter
from rce.monitor.interface import PublisherInterface, SubscriberInterface, \
    ServiceClientInterface, ServiceProviderInterface
from rce.slave.endpoint import Endpoint
from rce.slave.namespace import Namespace


class Environment(Namespace):
    """
    """
    _MAP = [ServiceClientInterface, PublisherInterface,
            SubscriberInterface, ServiceProviderInterface]
    
    def __init__(self, reactor):
        """
        """
        self._reactor = reactor
        self._loader = Loader()
        
        self._nodes = set()
        self._parameters = set()
        
        Namespace.__init__(self)
    
    @property
    def reactor(self):
        """ Reference to the twisted::reactor. """
        return self._reactor
    
    @property
    def loader(self):
        """ Reference to the ROS loader. """
        return self._loader
    
    def registerNode(self, node):
        assert node not in self._nodes
        self._nodes.add(node)
    
    def unregisterNode(self, node):
        assert node in self._nodes
        self._nodes.remove(node)
    
    def registerParameter(self, parameter):
        assert parameter not in self._parameters
        self._parameters.add(parameter)
    
    def unregisterParameter(self, parameter):
        assert parameter in self._parameters
        self._parameters.remove(parameter)
    
    def registerInterface(self, interface):
        assert interface not in self._interfaces
        self._interfaces.add(interface)
    
    def unregisterInterface(self, interface):
        assert interface in self._interfaces
        self._interfaces.remove(interface)
    
    def remote_createNode(self, pkg, exe, args, name, namespace):
        """
        """
        return Node(self, pkg, exe, args, name, namespace)
    
    def remote_createParameter(self, name, value):
        """
        """
        return Parameter(self, name, value)
    
    def remote_createInterface(self, uid, iType, msgType, addr):
        """
        """
        return self._MAP[iType](self, UUID(bytes=uid), msgType, addr)
    
    def remote_destroy(self):
        """
        """
        for node in self._nodes.copy():
            node.remote_destroy()
        
        for parameter in self._parameters.copy():
            parameter.remote_destroy()
        
        for interface in self._interfaces.copy():
            interface.remote_destroy()
        
        assert len(self._nodes) == 0
        assert len(self._parameters) == 0    
        assert len(self._interfaces) == 0


class EnvironmentClient(Endpoint):
    """
    """
    def __init__(self, reactor):
        """
        """
        Endpoint.__init__(self, reactor)
        
        self._namespace = None
    
    def remote_createNamespace(self):
        """
        """
        if self._namespace:
            raise InternalError('The environment can only have one namespace '
                                'at a time.')
        
        namespace = Environment(self._reactor)
        self._namespace = namespace
        return namespace
    
    def terminate(self):
        """
        """
        if self._namespace:
            self._namespace.remote_destroy()
            self._namespace = None
        
        Endpoint.terminate(self)


def main(reactor, cred, masterIP, masterPort, commPort, uid):
    f = open('/opt/rce/data/env.log', 'w')
    log.startLogging(f)
    
    rospy.init_node('RCE_Master')
    
    factory = PBClientFactory()
    reactor.connectTCP(masterIP, masterPort, factory)
    
    client = EnvironmentClient(reactor)
    
    def terminate():
        reactor.callFromThread(client.terminate)
        reactor.callFromThread(reactor.stop)
    
    rospy.on_shutdown(terminate)
    
    def _err(reason):
        print(reason)
        terminate()
    
    d = factory.login(cred, (client, uid))
    d.addCallback(lambda ref: setattr(client, '__ref', ref))
    d.addErrback(_err)
    
    reactor.run(installSignalHandlers=False)
    
    f.close()
