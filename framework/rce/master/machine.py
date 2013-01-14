#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     machine.py
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
from rce.error import InternalError, MaxNumberExceeded
from rce.master.base import Proxy


class MachineError(Exception):
    """ Exception is raised if there is no free container process.
    """


class LoadBalancer(object):
    """ The Load Balancer is responsible for selecting the appropriate
        container to launch a new container. It therefore also keeps track
        of all the container processes registered with the cloud engine.
        
        There should only one instance running in the Master process.
    """
    def __init__(self):
        """ Initialize the Load Balancer.
        """
        self._machines = set()
    
    def createMachine(self, ref, maxNr):
        """ Create a new Machine object, which can be used to create new
            containers.
            
            @param ref:         Remote reference to the ContainerClient in the
                                container process.
            @type  ref:         twisted::RemoteReference
            
            @param maxNr:       The maximum number of container which are
                                allowed in the machine.
            @type  maxNr:       int
            
            @return:            New Machine instance.
            @rtype:             rce.master.machine.Machine
        """
        machine = Machine(ref, maxNr)
        self._machines.add(machine)
        return machine
    
    def destroyMachine(self, machine):
        """ Destroy a Machine object.
            
            @param machine:     Machine instance which should be destroyed.
            @type  machine:     rce.master.machine.Machine
        """
        try:
            self._machines.remove(machine)
        except KeyError:
            raise InternalError('Tried to remove a non existent machine.')
        
        machine.destroy()
    
    def _getNextMachine(self):
        """ Internally used method to retrieve the machine where the next
            container should be created.
        """
        try:
            return min(self._machines, key=lambda m: m.active)
        except ValueError:
            raise MachineError('There is no free machine.')
    
    def createContainer(self, uid):
        """ Select an appropriate machine and create a container.
            
            @param uid:         Unique ID which is used to identify the
                                environment process when he connects to the
                                Master.
            @type  uid:         str
            
            @return:            New Container instance.
            @rtype:             rce.master.machine.Container
        """
        return self._getNextMachine().createContainer(uid)
    
    def cleanUp(self):
        """ Method should be called to destroy all machines.
        """
        for machine in self._machines.copy():
            self.destroyMachine(machine)
        
        assert len(self._machines) == 0


class Machine(object):
    """ Representation of a machine in which containers can be created. It
        keeps track of all the containers running in the machine.
    """
    def __init__(self, ref, maxNr):
        """ Initialize the Machine.
            
            @param ref:         Remote reference to the ContainerClient in the
                                container process.
            @type  ref:         twisted::RemoteReference
            
            @param maxNr:       The maximum number of container which are
                                allowed in the machine.
            @type  maxNr:       int
        """
        self._ref = ref
        self._maxNr = maxNr
        self._ip = ref.broker.transport.getPeer().host
        
        self._containers = set()
    
    @property
    def active(self):
        """ The number of active containers in the machine. """
        return len(self._containers)
    
    def createContainer(self, uid):
        """ Create a container.
            
            @param uid:         Unique ID which is used to identify the
                                environment process when he connects to the
                                Master.
            @type  uid:         str
            
            @return:            New Container instance.
            @rtype:             rce.master.machine.Container
        """
        if len(self._containers) > self._maxNr:
            raise MaxNumberExceeded('Can not create more containers in this '
                                    'machine.')
        
        container = Container(self)
        d = self._ref.callRemote('createContainer', uid)
        d.addCallback(lambda result: container.createRemoteObject(result))
        d.addErrback(lambda failure: container.registerFailureObject(failure))
        return container
    
    def registerContainer(self, container):
        assert container not in self._containers
        self._containers.add(container)
    
    def unregisterContainer(self, container):
        assert container in self._containers
        self._containers.remove(container)
    
    def destroy(self):
        """ Method should be called to destroy the machine and will take care
            of deleting all circular references.
        """
        for container in self._containers.copy():
            container.destroy()
        
        assert len(self._containers) == 0
    
    def __eq__(self, other):
        return self._ip == other._ip
    
    def __ne__(self, other):
        return self._ip != other._ip
    
    def __hash__(self):
        return hash(self._ip)


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
        d = self._getPort()
        d.addCallback(lambda port: IPv4Address('TCP', self._machine._ip, port))
        return d
    
    @Proxy.returnDeferred
    def _getPort(self):
        return self.obj.callRemote('getPort')
    
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
    
    @Proxy.destroyProxy
    def _destroy(self):
        try:
            self.obj.callRemote('destroy')
        except DeadReferenceError:
            pass
