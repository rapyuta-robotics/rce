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
from rce.error import MaxNumberExceeded
from rce.master.base import Proxy


class MachineError(Exception):
    """
    """


class LoadBalancer(object):
    """
    """
    def __init__(self):
        """
        """
        self._machines = set()
        self._iter = iter(self._machines)
    
    def createMachine(self, ref, maxNr):
        """
        """
        machine = Machine(ref, maxNr)
        self._machines.add(machine)
        return machine
    
    def destroyMachine(self, machine):
        """
        """
        try:
            self._machines.remove(machine)
        except KeyError:
            raise MachineError()
        
        machine.destroy()
    
    def _getNextMachine(self, retry=False):
        """
        """
        try:
            return self._iter.next()
        except StopIteration:
            if retry:
                raise MachineError('There is no machine registered.')
        except RuntimeError:
            self._iter = iter(self._machines)
        
        return self._getNextMachine(True)
    
    def createContainer(self, uid):
        """
        """
        return self._getNextMachine().createContainer(uid)
    
    def cleanUp(self):
        """
        """
        for machine in self._machines.copy():
            self.destroyMachine(machine)
        
        assert len(self._machines) == 0


class Machine(object):
    """
    """
    def __init__(self, ref, maxNr):
        """
        """
        self._ref = ref
        self._maxNr = maxNr
        self._ip = ref.broker.transport.getPeer().host
        
        self._containers = set()
    
    def createContainer(self, uid):
        """
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
        """
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
    """
    """
    def __init__(self, machine):
        """
        """
        super(Container, self).__init__()
        
        self._machine = machine
        machine.registerContainer(self)
    
    def getAddress(self):
        """
        """
        d = self._getPort()
        d.addCallback(lambda port: IPv4Address('TCP', self._machine._ip, port))
        return d
    
    @Proxy.returnDeferred
    def _getPort(self):
        return self.obj.callRemote('getPort')
    
    def destroy(self):
        """
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
