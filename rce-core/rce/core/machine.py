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
    def __init__(self, root):
        """ Initialize the Load Balancer.

            @param root:        Reference to top level of data structure.
            @type  root:        rce.master.RoboEarthCloudEngine
        """
        self._root = root
        self._network_group_ip = defaultdict(set)
        self._network_group_node = defaultdict(set)
        self._machines = set()

    def createMachine(self, ref, maxNr):
        """ Create a new Machine object, which can be used to create new
            containers.

            @param ref:         Remote reference to the ContainerClient in the
                                container process.
            @type  ref:         twisted.spread.pb.RemoteReference

            @param maxNr:       The maximum number of container which are
                                allowed in the machine.
            @type  maxNr:       int

            @return:            New Machine instance.
            @rtype:             rce.core.machine.Machine
        """
        machine = Machine(ref, maxNr, self._root, self)
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

            @param data:        Extra data about the container.
            @type  data:        dict
        """
        # TODO :Make this smarter with all the rich data now available
        candidates = [machine for machine in self._machines
                      if machine._users[userID]]
        try:
            machine = max(candidates, key=lambda m: m.availability)
        except ValueError:
            try:
                machine = max(self._machines, key=lambda m: m.availability)
            except ValueError:
                raise ContainerProcessError('There is no free container '
                                            'process.')

        if machine.availability:
            return machine
        else:
            raise ContainerProcessError('There is no free container process.')

    def network_group_add_node(self, group, machine):
        network_group = self._network_group_node[group]
        if machine not in network_group:
            for target in network_group:
                self.build_tunnel(group, machine, target)
            network_group.add(machine)


    def network_group_remove_node(self, group, machine):
        self._network_group_node[group].remove(machine)
        for target in self._network_group_node[group]:
            target.destroyTunnel(group, machine.IP)


    def build_tunnel(self, groupname, machineA, machineB):
        """Internal call used to create a GRE Tunnel between two hosts
        (physical machines or instances)

            @param groupname:      Unique group name to be linked up
            @type  groupname:      str

            @param machineA:       The Physical Machine object
            @type  machineA:       rce.core.machine.Machine

            @param machineB:       The Physical Machine object
            @type  machineB:       rce.core.machine.Machine
        """
        if machineA.check_bridge(groupname) and \
        machineB.check_bridge(groupname):
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

            @param data:        More data about the container
            @type  data:        dict

            @return:            New Container instance.
            @rtype:             rce.core.container.Container
        """
        group_name = data.get('group')
        groupIp = data.get('groupIp')
        if group_name:
            group_name = str(abs(hash(' '.join((userID, group_name)))))[:8]
            data['group'] = group_name
            network_group = self._network_group_ip[group_name]
            if groupIp:
                network_group.add(groupIp)
            else:
                if len(network_group) > 254:
                    raise InternalError('Max limit on subnet reached')
                while 1 :
                    candidate = '192.168.1.' + str(randint(2, 254))
                    if candidate not in network_group:
                        network_group.add(candidate)
                        data['groupIp'] = candidate
                        break
        return self._getNextMachine(userID, data).createContainer(uid, userID, data)

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
    def __init__(self, ref, maxNr, root, balancer):
        """ Initialize the Machine.

            @param ref:         Remote reference to the ContainerClient in the
                                container process.
            @type  ref:         twisted.spread.pb.RemoteReference

            @param maxNr:       The maximum number of container which are
                                allowed in the machine.
            @type  maxNr:       int

            @param root:        Reference to top level of data structure.
            @type  root:        rce.master.RoboEarthCloudEngine

            @param balancer:    Reference to top level of data structure.
            @type  balancer:    rce.core.machine.LoadBalancer
        """
        self._ref = ref
        self._maxNr = maxNr

        ip = ref.broker.transport.getPeer().host
        self._ip = root.getInternalIP() if isLocalhost(ip) else ip
        self._balancer = balancer

        self._containers = set()
        self._users = Counter()
        self._ovs_bridge = {}

    @property
    def active(self):
        """ The number of active containers in the machine. """
        return len(self._containers)

    @property
    def capacity(self):
        """ The number of active containers in the machine. """
        return self._maxNr

    @property
    def availability(self):
        """ The number of available containers in the machine. """
        return self._maxNr - len(self._containers)

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

            @param data:        Extra Information about the container
            @type  data:        dict

            @return:            New Container instance.
            @rtype:             rce.core.container.Container
        """
        if len(self._containers) >= self._maxNr:
            raise MaxNumberExceeded('You have run out of your container '
                                    'capacity.')
        container = Container(self, userID, data)
        groupname = data.get('group')
        if groupname:
            self.createBridge(groupname)
            self._ovs_bridge[groupname]['locals'].add(data.get('groupIp'))
        self._ref.callRemote('createContainer', uid, data).chainDeferred(container)
        return container

    def createBridge(self, groupname):
        """ Create a new OVS Bridge

            @param groupname:       Unique name of the network group
            @type  groupname:       str
        """
        if groupname not in self._ovs_bridge.iterkeys():
            self._ovs_bridge[groupname] = {'locals':set(), 'extern':set()}
            return self._ref.callRemote('createBridge', groupname)

    def destroyBridge(self, groupname):
        """ Destroy the OVS Bridge

            @param groupname:        Unique name of the network group
            @type  groupname:        str
        """
        if groupname in self._ovs_bridge.iterkeys():
            del self._ovs_bridge[groupname]
            return self._ref.callRemote('destroyBridge', groupname)

    def check_bridge(self, groupname):
        return groupname in self._ovs_bridge.iterkeys()

    def createTunnel(self, groupname, targetIp):
        """ Destroy a new GRE Tunnel

            @param groupname:        Unique name of the network group
            @type  groupname:        str

            @param targetIp:         Target ip for the gre Tunnel
            @type  targetIp:         str
        """
        if targetIp not in self._ovs_bridge[groupname]['extern']:
                self._ovs_bridge[groupname]['extern'].add(targetIp)
                return self._ref.callRemote('createTunnel', groupname, targetIp)

    def destroyTunnel(self, groupname, targetIp):
        """ Destroy the GRE Tunnel

            @param groupname:        Unique name of the network group
            @type  groupname:        str

            @param targetIp:         Target ip for the gre Tunnel
            @type  targetIp:         str
        """
        if targetIp in self._ovs_bridge[groupname]['extern']:
                self._ovs_bridge[groupname]['extern'].remove(targetIp)
                return self._ref.callRemote('destroyTunnel', groupname, targetIp)

    def registerContainer(self, container):
        assert container not in self._containers
        self._containers.add(container)
        self._users[container._userID] += 1
        if container._group:
            self._balancer.network_group_add_node(
                                     container._group, self)

    def unregisterContainer(self, container):
        assert container in self._containers
        self._containers.remove(container)
        self._users[container._userID] -= 1
        if container._group:
            self._ovs_bridge[container._group
                             ]['locals'].remove(container._groupIp)
            self._balancer._network_group_ip[container._group
                                    ].remove(container._groupIp)

            if not self._ovs_bridge[container._group]['locals']:
                self._balancer.network_group_remove_node(
                                     container._group, self)
                self.destroyBridge(container._group)


    def listContainers(self):
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
