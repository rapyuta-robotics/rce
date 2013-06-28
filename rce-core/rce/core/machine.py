#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-core/rce/core/machine.py
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

# Python specific imports
from collections import Counter
from random import choice
from string import letters

# twisted specific imports
from twisted.spread.pb import Avatar

# rce specific imports
from rce.util.error import InternalError
from rce.util.settings import getSettings
from rce.util.network import isLocalhost
from rce.util.iaas import IaasHook
from rce.core.error import InvalidRequest, MaxNumberExceeded
from rce.core.container import Container


# Helper function to generate random strings
randomString = lambda length: ''.join(choice(letters) for _ in xrange(length))


class ContainerProcessError(Exception):
    """ Exception is raised if there is no free container process.
    """


class RobotProcessError(Exception):
    """ Exception is raised if there is no free robot process.
    """


class Distributor(object):
    """ The Distributor is responsible for selecting the appropriate robot
        process to create a WebSocket connection. It therefore also keeps track
        of all the robot processes registered with the cloud engine.

        There should only one instance running in the Master process.
    """
    def __init__(self):
        """ Initialize the Distributor.
        """
        self._robots = set()

    def registerRobotProcess(self, robot):
        assert robot not in self._robots
        self._robots.add(robot)

    def unregisterRobotProcess(self, robot):
        assert robot in self._robots
        self._robots.remove(robot)

    def getNextLocation(self):
        """ Get the next endpoint running in an robot process to create a new
            robot WebSocket connection.

            @return:            Next robot endpoint.
            @rtype:             rce.core.robot.RobotEndpoint
                                (subclass of rce.core.base.Proxy)
        """
        try:
            return min(self._robots, key=lambda r: r.active)
        except ValueError:
            raise RobotProcessError('There is no free robot process.')

    def cleanUp(self):
        assert len(self._robots) == 0


# TODO: Should probably be renamed...
class LoadBalancer(object):
    """ The Load Balancer is responsible for selecting the appropriate
        container to launch a new container. It therefore also keeps track
        of all the container processes registered with the cloud engine.

        There should only one instance running in the Master process.
    """
    _UID_LEN = 8

    def __init__(self):
        """ Initialize the Load Balancer.
        """
        self._empty = EmptyNetworkGroup()
        self._groups = {}
        self._uid = set()
        self._machines = set()
        self._iaas = None

    def createMachine(self, ref, data):
        """ Create a new Machine object, which can be used to create new
            containers.

            @param ref:         Remote reference to the ContainerClient in the
                                container process.
            @type  ref:         twisted.spread.pb.RemoteReference

            @param data:        Data about the machine
            @type  data:        dict

            @return:            New Machine instance.
            @rtype:             rce.core.machine.Machine
        """
        machine = Machine(ref, data, self)

        if machine in self._machines:
            raise InternalError('Tried to add the same machine multiple times.')

        self._machines.add(machine)
        return machine

    def destroyMachine(self, machine):
        """ Destroy a Machine object.

            @param machine:     Machine instance which should be destroyed.
            @type  machine:     rce.core.machine.Machine
        """
        try:
            self._machines.remove(machine)
        except KeyError:
            raise InternalError('Tried to remove a non existent machine.')

        machine.destroy()

    def _createContainer(self, data, userID):
        """ # TODO: Add doc
        """
        name = data.pop('group', None)

        if name:
            key = (userID, name)
            group = self._groups.get(key)

            if not group:
                while 1:
                    uid = randomString(self._UID_LEN)

                    if uid not in self._uid:
                        break

                self._uid.add(uid)
                group = NetworkGroup(self, key, uid)
                self._groups[key] = group
        else:
            # There is no group, i.e. 'special' group required
            group = self._empty

        return group.createContainer(data, userID)

    def _getMachine(self, container):
        """ Internally used method to assign a machine to the container which
            should be created.

            @param container:   Container which should be created.
            @type  container:   rce.core.container.Container

            @return:            Machine to which the container should be
                                assigned.
            @rtype:             rce.core.machine.Machine
        """
        size = container.size
        userID = container.userID

        # TODO: At the moment not used
#        cpu = container.cpu
#        memory = container.memory
#        bandwidth = container.bandwidth
#        specialFeatures = container.specialFeatures

        machines = [m for m in self._machines if m.availability >= size]

        # TODO: The above uses block assumptions, implement fine grain control
        #       at bottom. Like check memory data, use filters to get machines
        #       with special features like gpu, avxii, mmx sse
        if not machines:
#            if self._iaas:
#                self._iaas.spin_up()  # count, type, special_request
#                # TODO: Need to get the current list of machines here!
#                #       However, spin up of a new instance has most certainly a
#                #       delay; therefore, probably a Deferred has to be used...
#            else:
                raise ContainerProcessError('You seem to have run out of '
                                            'capacity. Add more nodes.')

        candidates = [m for m in machines if m.getUserCount(userID)]

        if candidates:
            return max(candidates, key=lambda m: m.availability)
        else:
            return max(machines, key=lambda m: m.availability)

    def createContainer(self, uid, userID, data):
        """ Select an appropriate machine and create a container.

            @param uid:         Unique ID which is used to identify the
                                environment process when he connects to the
                                Master.
            @type  uid:         str

            @param userID:      UserID of the user who created the container.
            @type  userID:      str

            @param data:        Extra data used to configure the container.
            @type  data:        dict

            @return:            New Container instance.
            @rtype:             rce.core.container.Container
        """
        container = self._createContainer(data, userID)
        self._getMachine(container).assignContainer(container, uid)
        return container

    def registerIAASHook(self, hook):
        """ Register an IAAS Hook object.

             # TODO: Add doc
        """
        if not isinstance(hook, IaasHook):
            raise InternalError('IAAS hook has to be a subclass of '
                                'rce.util.iaas.IaasHook.')

        self._iaas = hook

    def unregisterIAASHook(self):
        """ Method should be called to destroy all machines.
        """
        if self._iaas:
            self._iaas.disconnect()
            self._iaas = None

    def freeGroup(self, key, uid):
        """ # TODO: Add doc
        """
        self._uid.remove(uid)
        del self._groups[key]

    def cleanUp(self):
        """ Method should be called to destroy all machines.
        """
        for group in self._groups.values():
            group.destroy()

        assert len(self._groups) == 0

        for machine in self._machines.copy():
            self.destroyMachine(machine)

        assert len(self._machines) == 0

        self.unregisterIAASHook()


class Machine(object):
    """ Representation of a machine in which containers can be created. It
        keeps track of all the containers running in the machine.
    """
    def __init__(self, ref, data, balancer):
        """ Initialize the Machine.

            @param ref:         Remote reference to the ContainerClient in the
                                container process.
            @type  ref:         twisted.spread.pb.RemoteReference

            @param data:        Data about the machine.
            @type  data:        dict

            @param balancer:    Reference to the load balancer which is
                                responsible for this machine.
            @type  balancer:    rce.core.machine.LoadBalancer
        """
        self._ref = ref

        self._size = data.get('size')
        self._cpu = data.get('cpu')
        self._memeory = data.get('memory')
        self._bandwidth = data.get('bandwidth')
        self._specialFeatures = data.get('specialFeatures')

        ip = ref.broker.transport.getPeer().host
        self._ip = getSettings().internal_IP if isLocalhost(ip) else ip
        self._balancer = balancer

        self._containers = set()
        self._users = Counter()

    @property
    def active(self):
        """ The number of active containers in the machine. """
        return len(self._containers)

    @property
    def size(self):
        """ Machine Capacity. """
        return self._size

    @property
    def cpu(self):
        """ Machine CPU Info. """
        return self._cpu

    @property
    def memory(self):
        """ Machine Memory Info. """
        return self._memory

    @property
    def bandwidth(self):
        """ Machine Bandwidth Info. """
        return self._bandwidth

    @property
    def specialFeatures(self):
        """ Machine Special Features Info. """
        return self._specialFeatures

    @property
    def availability(self):
        """ Free Machine Capacity. """
        return self._size - sum(c.size for c in self._containers)

    @property
    def IP(self):
        """ The IP address used for the internal communication of the machine.
        """
        return self._ip

    def getUserCount(self, userID):
        """ # TODO: Add doc
        """
        return self._users[userID]

    def assignContainer(self, container, uid):
        """ # TODO: Add doc
        """
        if self.availability < container.size:
            raise MaxNumberExceeded('Machine has run out of container '
                                    'capacity.')

        container.assignMachine(self)
        d = self._ref.callRemote('createContainer', uid, container.serialized)
        d.chainDeferred(container)

    def createBridge(self, name):
        """ Create a new OVS Bridge.

            @param name:        Unique name of the network group.
            @type  name:        str
        """
        return self._ref.callRemote('createBridge', name)

    def destroyBridge(self, name):
        """ Destroy a OVS Bridge.

            @param name:        Unique name of the network group.
            @type  name:        str
        """
        return self._ref.callRemote('destroyBridge', name)

    def createTunnel(self, name, targetIP):
        """ Create a new GRE Tunnel.

            @param name:        Unique name of the network group.
            @type  name:        str

            @param targetIP:    Target IP for the GRE Tunnel.
            @type  targetIP:    str
        """
        return self._ref.callRemote('createTunnel', name, targetIP)

    def destroyTunnel(self, name, targetIP):
        """ Destroy a GRE Tunnel.

            @param name:        Unique name of the network group.
            @type  name:        str

            @param targetIP:    Target IP for the GRE Tunnel.
            @type  targetIP:    str
        """
        return self._ref.callRemote('destroyTunnel', name, targetIP)

    def getSysinfo(self, request):
        """ Get realtime Sysinfo data from machine.

            @param request:     data desired
            @type  request:     # TODO: Add type
        """
        return self._ref.callRemote('getSysinfo')

    def setSysinfo(self, request, value):
        """ Set some system parameter to the machine.

            @param request:     data desired
            @type  request:     # TODO: Add type

            @param value:       data value
            @type  value:       # TODO: Add type
        """
        return self._ref.callRemote('setSysinfo', value)

    def registerContainer(self, container):
        assert container not in self._containers
        self._containers.add(container)
        self._users[container.userID] += 1

    def unregisterContainer(self, container):
        assert container in self._containers
        self._containers.remove(container)
        cnt = self._users[container.userID] - 1
        if cnt:
            self._users[container.userID] = cnt
        else:
            del self._users[container.userID]

# TODO: Not used
#    def listContainers(self):
#        """ # TODO: Add doc
#        """
#        return self._containers

    def destroyContainer(self, remoteContainer):
        """ Destroy Container proxy.
        """
        for container in self._containers:
            if container.destroyExternal(remoteContainer):
                break

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


class MachineAvatar(Avatar):
    """ Avatar for internal PB connection from a Machine.
    """
    def __init__(self, machine, balancer):
        """ Initialize the Machine avatar.

            @param machine:     Representation of the Machine.
            @type  machine:     rce.core.machine.Machine

            @param balancer:    The load balancer.
            @type  balancer:    rce.core.machine.LoadBalancer
        """
        self._machine = machine
        self._balancer = balancer

    def perspective_containerDied(self, remoteContainer):
        """ Notify that a remote container died.

            @param remoteContainer: Reference to the remote Container.
            @type  remoteContainer: twisted.spread.pb.RemoteReference
        """
        self._machine.destroyContainer(remoteContainer)

    def logout(self):
        """ Callback which should be called upon disconnection of the Machine
        """
        self._balancer.destroyMachine(self._machine)


class EmptyNetworkGroup(object):
    """ # TODO: Add doc
    """
    @property
    def name(self):
        """ Name of the network group. """
        return None

    def createContainer(self, data, userID):
        return Container(data, userID, self, None)

    def registerContainer(self, _):
        pass

    def unregisterContainer(self, _):
        pass


class NetworkGroup(object):
    """ # TODO: Add doc
    """
    # TODO: Should the IP address be configurable?
    _NETWORK_ADDR = '192.168.1'

    def __init__(self, manager, key, uid):
        """ # TODO: Add doc
        """
        self._manager = manager
        self._key = key
        self._uid = uid
        self._ips = set(xrange(2, 255))
        self._containers = set()
        self._machines = {}

    @property
    def name(self):
        """ Name of the network group. """
        return self._uid

    def createContainer(self, data, userID):
        """ # TODO: Add doc
        """
        if not self._ips:
            raise InvalidRequest('No more free IP addresses in subnet.')

        ip = data.pop('groupIP', None)

        if ip:
            addr, nr = ip.rsplit('.', 1)
            nr = int(nr)

            if addr != self._NETWORK_ADDR:
                addr = '{0}.0'.format(self._NETWORK_ADDR)
                raise InvalidRequest("IP address '{0}' is not in network range "
                                     "'{1}'".format(ip, addr))

            try:
                self._ips.remove(nr)
            except KeyError:
                raise InvalidRequest("IP address '{0}' is already in "
                                     'use.'.format(ip))
        else:
            ip = '{1}.{0}'.format(self._ips.pop(), self._NETWORK_ADDR)

        return Container(data, userID, self, ip)

    def registerContainer(self, container):
        assert container not in self._containers
        self._containers.add(container)
        self._registerMachine(container.machine)

    def unregisterContainer(self, container):
        assert container in self._containers
        self._containers.remove(container)
        self._unregisterMachine(container.machine)
        if not self._containers:
            self.destroy()

    def _registerMachine(self, machine):
        """ # TODO: Add doc
        """
        if machine not in self._machines:
            machine.createBridge(self._uid)

            for m in self._machines:
                machine.createTunnel(self._uid, m.IP)
                m.createTunnel(self._uid, machine.IP)

            self._machines[machine] = 1
        else:
            self._machines[machine] += 1

    def _unregisterMachine(self, machine):
        """ # TODO: Add doc
        """
        cnt = self._machines[machine] - 1

        if cnt:
            self._machines[machine] = cnt
        else:
            del self._machines[machine]

            for m in self._machines:
                machine.destroyTunnel(self._uid, m.IP)
                m.destroyTunnel(self._uid, machine.IP)

            machine.destroyBridge(self._uid)

    def destroy(self):
        """ # TODO: Add doc
        """
        self._manager.freeGroup(self._key, self._uid)
        self._manager = None
