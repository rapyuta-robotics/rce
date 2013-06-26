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
from collections import Counter, defaultdict
from random import randint

# twisted specific imports
from twisted.spread.pb import Avatar

# rce specific imports
from rce.util.error import InternalError
from rce.util.settings import getSettings
from rce.util.network import isLocalhost
from rce.core.error import MaxNumberExceeded
from rce.core.container import Container


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
        self._iter = iter(self._robots)

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


class LoadBalancer(object):
    """ The Load Balancer is responsible for selecting the appropriate
        container to launch a new container. It therefore also keeps track
        of all the container processes registered with the cloud engine.

        There should only one instance running in the Master process.
    """
    def __init__(self):
        """ Initialize the Load Balancer.
        """
        self._networkGroupLookup = {}
        self._networkGroupIP = defaultdict(set)
        self._networkGroupNode = defaultdict(set)
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

    def _getNextMachine(self, userID, data):
        """ Internally used method to retrieve the machine where the next
            container should be created.

            @param userID:      UserID of the user who created the container.
            @type  userID:      str

            @param data:        Extra data used to configure the container.
            @type  data:        dict
        """
        size = data.get('size', 1)
        # TODO: At the moment not used
#        cpu = data.get('cpu', 0)
#        memory = data.get('memory', 0)
#        bandwidth = data.get('bandwidth', 0)
#        specialFeatures = data.get('specialFeatures', [])

        machines = [machine for machine in self._machines
                    if machine.availability >= size]

        # TODO: The above uses block assumptions, implement fine grain control
        #       at bottom. Like check memory data, use filters to get machines
        #       with special features like gpu, avxii, mmx sse
        if not machines:
#            if self._iaas:
#                self._iaas.spin_up()  # count, type, special_request
#                # TODO: Need to get the current list of machines here!
#                        However, spin up of a new instance has most certainly a
#                        delay; therefore, probably a Deferred has to be used...
#            else:
                raise ContainerProcessError('You seem to have run out of '
                                            'capacity. Add more nodes.')

        candidates = [machine for machine in machines if machine._users[userID]]

        if candidates:
            return max(candidates, key=lambda m: m.availability)
        else:
            return max(machines, key=lambda m: m.availability)

    def networkGroupAddNode(self, group, machine):
        """ # TODO: Add doc
        """
        networkGroup = self._networkGroupNode[group]
        if machine not in networkGroup:
            for target in networkGroup:
                self.buildTunnel(group, machine, target)
            networkGroup.add(machine)

    def networkGroupRemoveNode(self, group, machine):
        """ # TODO: Add doc
        """
        self._networkGroupNode[group].remove(machine)
        for target in self._networkGroupNode[group]:
            target.destroyTunnel(group, machine.IP)

    def buildTunnel(self, groupname, machineA, machineB):
        """ Internal call used to create a GRE Tunnel between two hosts
            (physical machines or instances)

            @param groupname:   Unique group name for which the tunnel has
                                to be built.
            @type  groupname:   str

            @param machineA:    The Physical Machine object.
            @type  machineA:    rce.core.machine.Machine

            @param machineB:    The Physical Machine object.
            @type  machineB:    rce.core.machine.Machine
        """
        if (machineA.checkBridge(groupname) and
            machineB.checkBridge(groupname)):
            machineA.createTunnel(groupname, machineB.IP)
            machineB.createTunnel(groupname, machineA.IP)
        else:
            raise InternalError('Tunnel nodes are invalid')

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
        groupName = data.get('group')
        groupIp = data.get('groupIp')
        if groupName:
            key = str(abs(hash(' '.join((userID, groupName)))))[:8]
            # Check for extremely rare cases of key collision
            group = self._networkGroupLookup.get(key)
            if group:
                if group != (userID, groupName):
                    raise InternalError('User Group Key collision detected')
            else:
                self._networkGroupLookup[key] = (userID, groupName)
            data['group'] = key
            networkGroup = self._networkGroupIP[key]
            if groupIp:
                networkGroup.add(groupIp)
            else:
                if len(networkGroup) > 254:
                    raise InternalError('Max limit on subnet reached')
                while 1 :
                    candidate = '192.168.1.' + str(randint(2, 254))
                    if candidate not in networkGroup:
                        networkGroup.add(candidate)
                        data['groupIp'] = candidate
                        break
        return self._getNextMachine(userID, data).createContainer(uid, userID, data)

    def registerIAASHook(self, hook):
        """ Register an IAAS Hook object.
        """
        self._iaas = hook

    def unregisterIAASHook(self):
        """ Method should be called to destroy all machines.
        """
        if self._iaas:
            self._iaas.disconnect()
            self._iaas = None

    def cleanUp(self):
        """ Method should be called to destroy all machines.
        """
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
        self._ovsBridge = {}

    @property
    def active(self):
        """ The number of active containers in the machine. """
        return len(self._containers)

    @property
    def size(self):
        """ Machine Capacity """
        return self._size

    @property
    def cpu(self):
        """Machine CPU Info. """
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
        """Machine Availability """
        return self._size - sum(c.size for c in self._containers)

    @property
    def IP(self):
        """ The IP address used for the internal communication of the machine.
        """
        return self._ip

    def createContainer(self, uid, userID, data):
        """ Create a container.

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
        if self.availability < data.get('size', 1):
            raise MaxNumberExceeded('You have run out of your container '
                                    'capacity.')
        groupname = data.get('group')
        if groupname:
            self.createBridge(groupname)
            self._ovsBridge[groupname]['locals'].add(data.get('groupIp'))
        container = Container(self, userID, data)
        self._ref.callRemote('createContainer', uid, data).chainDeferred(container)
        return container

    def createBridge(self, groupname):
        """ Create a new OVS Bridge.

            @param groupname:       Unique name of the network group.
            @type  groupname:       str
        """
        if groupname not in self._ovsBridge:
            self._ovsBridge[groupname] = {'locals':set(), 'extern':set()}
            return self._ref.callRemote('createBridge', groupname)

    def destroyBridge(self, groupname):
        """ Destroy a OVS Bridge.

            @param groupname:       Unique name of the network group.
            @type  groupname:       str
        """
        if groupname in self._ovsBridge:
            del self._ovsBridge[groupname]
            return self._ref.callRemote('destroyBridge', groupname)

    def checkBridge(self, groupname):
        """ # TODO: Add doc
        """
        return groupname in self._ovsBridge

    def createTunnel(self, groupname, targetIP):
        """ Create a new GRE Tunnel.

            @param groupname:       Unique name of the network group.
            @type  groupname:       str

            @param targetIP:        Target IP for the GRE Tunnel.
            @type  targetIP:        str
        """
        if targetIP not in self._ovsBridge[groupname]['extern']:
            self._ovsBridge[groupname]['extern'].add(targetIP)
            return self._ref.callRemote('createTunnel', groupname, targetIP)

    def destroyTunnel(self, groupname, targetIP):
        """ Destroy a GRE Tunnel

            @param groupname:       Unique name of the network group.
            @type  groupname:       str

            @param targetIP:        Target IP for the GRE Tunnel.
            @type  targetIP:        str
        """
        if targetIP in self._ovsBridge[groupname]['extern']:
            self._ovsBridge[groupname]['extern'].remove(targetIP)
            return self._ref.callRemote('destroyTunnel', groupname, targetIP)

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
        if container.group:
            self._balancer.networkGroupAddNode(container.group, self)

    def unregisterContainer(self, container):
        assert container in self._containers
        self._containers.remove(container)
        self._users[container.userID] -= 1
        if container.group:
            self._ovsBridge[container.group
                             ]['locals'].remove(container.groupIP)
            self._balancer._networkGroupIP[container.group
                                    ].remove(container.groupIP)

            if not self._ovsBridge[container.group]['locals']:
                self._balancer.networkGroupRemoveNode(container.group, self)
                self.destroyBridge(container.group)

    def listContainers(self):
        """ # TODO: Add doc
        """
        return self._containers

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


# TODO: This needs some work on specification:
#        - define interface
#        - create a class for each IAAS type implementing interface
class IaasHook(object):
    """ # TODO: Add doc
    """
    def __init__(self, balancer):
        """Base class to implement IASS Hooks for using a
           public cloud platform auto provisioning

            @param balancer:    The RCE LoadBalancer
            @type  balancer:    rce.core.machine.LoadBalancer
        """
        balancer.registerIAASHook(self)

    def disconnect(self):
        """ Method called when shutting down the engine to relieve the hook.
        """
        # TODO: Should destroy all instances which have been started dynamically
        raise NotImplementedError

    def spin_up(self, count=1, type=None, specialRequest=None):
        """ Call to spin up more instances.

            @param count:           Number of instances to be spun up.
            @type  count:           int

            @param type:            Type (generally size) of instance requested
            @type  type:            TDB by implementation

            @param specialRequest:  Special request (gpu, cluster, hadoop)
            @type  specialRequest:  TDB by implementation
        """
        raise NotImplementedError

    def spin_down(self):
        """ # TODO: ???
        """
        raise NotImplementedError
