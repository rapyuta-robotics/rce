#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     rce-core/rce/container.py
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
import os
import sys
import stat
import shutil
from random import choice
from string import letters

pjoin = os.path.join

try:
    import iptc
except ImportError:
    print("Can not import the package 'python-iptables'.")
    print('    see: http://github.com/ldx/python-iptables')
    exit(1)

# twisted specific imports
from twisted.python import log
from twisted.internet.defer import  DeferredList, succeed
from twisted.spread.pb import Referenceable, PBClientFactory, \
    DeadReferenceError, PBConnectionLost

# rce specific imports
from rce.util.error import InternalError
from rce.util.container import Container
from rce.util.cred import salter, encodeAES, cipher
from rce.util.network import isLocalhost
from rce.util.process import execute
from rce.core.error import MaxNumberExceeded
# from rce.util.ssl import createKeyCertPair, loadCertFile, loadKeyFile, \
#    writeCertToFile, writeKeyToFile


# Helper function to generate random strings
randomString = lambda length: ''.join(choice(letters) for _ in xrange(length))


_UPSTART_COMM = """
# description
author "Dominique Hunziker"
description "RCE Comm - Framework for managing and using ROS Apps"

# start/stop conditions
start on (started rce and net-device-up IFACE=eth0)
stop on stopping rce

kill timeout 5

script
    # setup environment
    . /opt/rce/setup.sh
    {chownCmd}
    {ifconfCmd}

    # start environment node
    start-stop-daemon --start -c rce:rce -d /opt/rce/data --retry 5 --exec /usr/local/bin/rce-environment -- {masterIP} {masterPort} {internalPort} {uid} {passwd}
end script
"""


_UPSTART_ROSAPI = """
# description
author "Mayank Singh"
description "ROS API - Framework to query ROS specific information"

# start/stop conditions
start on (started rce and net-device-up IFACE=eth0)
stop on stopping rce

kill timeout 5

script
    # setup environment
    . /opt/rce/setup.sh

    # start rosapi node
    start-stop-daemon --start -c rce:rce -d /opt/rce/data --retry 5 --exec /usr/local/bin/rce-rosproxy {proxyPort}
end script
"""


# TODO: Modify name of executable
_UPSTART_LAUNCHER = """
# description
author "Dominique Hunziker"
description "RCE Launcher - Framework for managing and using ROS Apps"

# start/stop conditions
start on started rce
stop on stopping rce

# timeout before the process is killed; generous as a lot of processes have
# to be terminated by the launcher.
kill timeout 30

script
    # setup environment
    . /opt/rce/setup.sh

    # start launcher
    start-stop-daemon --start -c ros:ros -d /home/ros --retry 5 --exec /opt/rce/src/launcher.py
end script
"""


_NETWORK_INTERFACES = """
auto lo
iface lo inet loopback

auto eth0
iface eth0 inet static
    address {{ip}}
    netmask 255.255.255.0
    network {network}.0
    broadcast {network}.255
    gateway {network}.1
    dns-nameservers {network}.1 127.0.0.1
"""


_GROUP_NETWORK = """
#!/bin/bash

ifconfig $5 0.0.0.0 {if_op}
ovs-vsctl {ovs_op}-port br-{name} $5
"""


class RCEContainer(Referenceable):
    """ Container representation which is used to run a ROS environment.
    """
    def __init__(self, client, nr, uid, data):
        """ Initialize the deployment container.

            @param client:      Container client which is responsible for
                                monitoring the containers in this machine.
            @type  client:      rce.container.ContainerClient

            @param nr:          Unique number which will be used for the IP
                                address and the hostname of the container.
            @type  nr:          int

            @param uid:         Unique ID which is used by the environment
                                process to login to the Master.
            @type  uid:         str

            @param data:        Extra data used to configure the container.
            @type  data:        dict
        """
        # TODO: __init__ throws ValueError in which case the config and data
        #       directories are not removed!

        # Store the references
        self._client = client

        self._nr = nr
        self._name = 'C{0}'.format(nr)
        self._terminating = None

        # Additional container parameters to use
        # TODO: At the moment not used; currently data also does not contain
        #       these information
#        self._size = data.get('size', 1)
#        self._cpu = data.get('cpu', 0)
#        self._memory = data.get('memory', 0)
#        self._bandwidth = data.get('bandwidth', 0)
#        self._specialFeatures = data.get('specialFeatures', [])

        client.registerContainer(self)

        if data.get('name'):
            ifconfCmd = ('ifconfig eth1 netmask 255.255.255.0 '
                         'broadcast 192.168.1.255')
        else:
            ifconfCmd = "#no ovs"

        # Create the directories for the container
        self._confDir = pjoin(client.confDir, self._name)
        self._dataDir = pjoin(client.dataDir, self._name)

        if os.path.isdir(self._confDir):
            raise ValueError('There is already a configuration directory for '
                             "'{0}'.".format(self._name))

        if os.path.isdir(self._dataDir):
            raise ValueError('There is already a data directory for '
                             "'{0}'.".format(self._name))

        os.mkdir(self._confDir)
        os.mkdir(self._dataDir)

        # Create additional folders for the container
        rceDir = pjoin(self._dataDir, 'rce')
        rosDir = pjoin(self._dataDir, 'ros')

        os.mkdir(rceDir)
        os.mkdir(rosDir)

        if self._client.rosRel > 'fuerte':
            # TODO: Switch to user 'ros' when the launcher is used again
            rosdepUser = os.path.join(rceDir, '.ros/rosdep')
            rosdepRoot = os.path.join(self._client.rootfs, 'root/.ros/rosdep')
            shutil.copytree(rosdepRoot, rosdepUser)
            chownCmd = 'chown -R rce:rce /opt/rce/data/.ros'
        else:
            chownCmd = '#no chown'

        # Create network variables
        ip = '{0}.{1}'.format(client.getNetworkAddress(), nr)
        self._address = '{0}:{1}'.format(ip, client.envPort)
        self._rosproxyAddress = '{0}:{1}'.format(ip, client.rosproxyPort)
        self._fwdPort = str(nr + 8700)
        self._rosproxyFwdPort = str(nr + 10700)

        brname = data.get('name')
        brip = data.get('ip')

        if brname and brip:
            ovsup = pjoin(self._confDir, 'ovsup')
            # TODO: OVS down: Not supported on 12.04, can be added later
            #ovsdown = pjoin(self._confDir, 'ovsdown')
            ovsdown = None

        # Construct password
        passwd = encodeAES(cipher(self._client.masterPassword),
                           salter(uid, self._client.infraPassword))

        # Create upstart scripts
        with open(pjoin(self._confDir, 'upstartComm'), 'w') as f:
            f.write(_UPSTART_COMM.format(masterIP=self._client.masterIP,
                                         masterPort=self._client.masterPort,
                                         internalPort=self._client.envPort,
                                         uid=uid, passwd=passwd,
                                         chownCmd=chownCmd,
                                         ifconfCmd=ifconfCmd))

        with open(pjoin(self._confDir, 'upstartRosapi'), 'w') as f:
            f.write(_UPSTART_ROSAPI.format(proxyPort=self._client.rosproxyPort))

        # TODO: For the moment there is no upstart script for the launcher.
#        with open(pjoin(self._confDir, 'upstartLauncher'), 'w') as f:
#            f.write(_UPSTART_LAUNCHER)

        # Setup network
        with open(pjoin(self._confDir, 'networkInterfaces'), 'w') as f:
            f.write(client.getNetworkConfigTemplate().format(ip=ip))

        # Create up/down script for virtual network interface if necessary
        if ovsup:
            with open(ovsup, 'w') as f:
                f.write(_GROUP_NETWORK.format(if_op='up', ovs_op='add',
                                              name=brname))

            os.chmod(ovsup, stat.S_IRWXU)

        if ovsdown:
            with open(ovsdown, 'w') as f:
                f.write(_GROUP_NETWORK.format(if_op='down', ovs_op='del',
                                              name=brname))

            os.chmod(ovsdown, stat.S_IRWXU)

        # Create the container
        self._container = Container(client.reactor, client.rootfs,
                                    self._confDir, self._name)

        # TODO: SSL stuff
#        if self._USE_SSL:
#            # Create a new certificate and key for environment node
#            caCertPath = pjoin(self._SSL_DIR, 'Container.cert')
#            caCert = loadCertFile(caCertPath)
#            caKey = loadKeyFile(pjoin(self._SSL_DIR, 'container/env.key'))
#            (cert, key) = createKeyCertPair(commID, caCert, caKey)
#
#            # Copy/save file to data directory
#            shutil.copyfile(caCertPath, os.path.join(rceDir, 'ca.pem'))
#            writeCertToFile(cert, os.path.join(rceDir, 'cert.pem'))
#            writeKeyToFile(key, os.path.join(rceDir, 'key.pem'))

        # Add lxc bridge
        self._container.addNetworkInterface('eth0', self._client.bridgeIF, ip)

        # Add the virtual network bridge if necessary
        if brname and brip:
            self._container.addNetworkInterface('eth1', None, brip, ovsup,
                                                ovsdown)

        # Add additional lines to fstab file of container
        self._container.extendFstab(rosDir, 'home/ros', False)
        self._container.extendFstab(rceDir, 'opt/rce/data', False)
        self._container.extendFstab(pjoin(self._confDir, 'upstartComm'),
                                    'etc/init/rceComm.conf', True)
        # TODO: For the moment there is no upstart script for the launcher.
#        self._container.extendFstab(pjoin(self._confDir, 'upstartLauncher'),
#                                    'etc/init/rceLauncher.conf', True)
        self._container.extendFstab(pjoin(self._confDir, 'upstartRosapi'),
                                    'etc/init/rceRosapi.conf', True)
        self._container.extendFstab(pjoin(self._confDir, 'networkInterfaces'),
                                    'etc/network/interfaces', True)

        for srcPath, destPath in client.pkgDirIter:
            self._container.extendFstab(srcPath, destPath, True)

    def start(self):
        """ Method which starts the container.
        """
        # NOTE: can raise iptc.xtables.XTablesError
        # add remote rule for RCE internal communication
        self._remoteRule = iptc.Rule()
        self._remoteRule.protocol = 'tcp'
        self._remoteRule.dst = self._client.internalIP
        m = self._remoteRule.create_match('tcp')
        m.dport = self._fwdPort
        t = self._remoteRule.create_target('DNAT')
        t.to_destination = self._address

        # add local (loopback) rule for RCE internal communication
        self._localRule = iptc.Rule()
        self._localRule.protocol = 'tcp'
        self._localRule.out_interface = 'lo'
        self._localRule.dst = self._client.internalIP
        m = self._localRule.create_match('tcp')
        m.dport = self._fwdPort
        t = self._localRule.create_target('DNAT')
        t.to_destination = self._address

        # add remote rule for rosproxy
        self._rosremoteRule = iptc.Rule()
        self._rosremoteRule.protocol = 'tcp'
        self._rosremoteRule.dst = self._client.internalIP
        m = self._rosremoteRule.create_match('tcp')
        m.dport = self._rosproxyFwdPort
        t = self._rosremoteRule.create_target('DNAT')
        t.to_destination = self._rosproxyAddress

        # add local(loopback) rule for rosproxy
        self._roslocalRule = iptc.Rule()
        self._roslocalRule.protocol = 'tcp'
        self._roslocalRule.out_interface = 'lo'
        self._roslocalRule.dst = self._client.internalIP
        m = self._roslocalRule.create_match('tcp')
        m.dport = self._rosproxyFwdPort
        t = self._roslocalRule.create_target('DNAT')
        t.to_destination = self._rosproxyAddress

        self._client.prerouting.insert_rule(self._remoteRule)
        self._client.output.insert_rule(self._localRule)
        self._client.prerouting.insert_rule(self._rosremoteRule)
        self._client.output.insert_rule(self._roslocalRule)

        return self._container.start(self._name)

    def remote_getPort(self):
        """ Get the port which can be used together with the host IP address
            to reach connect with the container.

            @return:            Port number of host machine which will be
                                forwarded to the container.
            @rtype:             int
        """
        return int(self._fwdPort)

    def _stop(self):
        """ Method which stops the container.
        """
        self._client.prerouting.delete_rule(self._remoteRule)
        self._client.output.delete_rule(self._localRule)
        self._client.prerouting.delete_rule(self._rosremoteRule)
        self._client.output.delete_rule(self._roslocalRule)

        return self._container.stop(self._name)

    def _destroy(self, response):
        """ Internally used method to clean up after the container has been
            stopped.
        """
        self._client.returnNr(self._nr)
        self._client.unregisterContainer(self)
        self._client = None

        for path in (self._confDir, self._dataDir):
            try:
                shutil.rmtree(path)
            except:
                pass

        return response

    def remote_destroy(self):
        """ Method should be called to destroy the container.
        """
        if not self._terminating:
            if self._container:
                self._terminating = self._stop()
                self._terminating.addBoth(self._destroy)
            else:
                self._terminating = succeed(None)

        return self._terminating


class ContainerClient(Referenceable):
    """ Container client is responsible for the creation and destruction of
        containers in a machine.

        There can be only one Container Client per machine.
    """
    _UID_LEN = 8

    def __init__(self, reactor, masterIP, masterPort, masterPasswd, infraPasswd,
                 bridgeIF, intIP, bridgeIP, envPort, rosproxyPort, rootfsDir,
                 confDir, dataDir, pkgDir, rosRel, data):
        """ Initialize the Container Client.

            @param reactor:         Reference to the twisted reactor.
            @type  reactor:         twisted::reactor

            @param masterIP:        IP address of the Master process.
            @type  masterIP:        str

            @param masterPort:      Port of the Master process used for internal
                                    communications.
            @type  masterPort:      int

            @param masterPasswd:    SHA 256 Digested Master Password.
            @type  masterPasswd:    str

            @param infraPasswd:     SHA 256 Digested Infra Password.
            @type  infraPasswd:     str

            @param bridgeIF:        Network interface used for the container
                                    communication.
            @type  bridgeIF:        str

            @param intIP:           IP address of the network interface used for
                                    the internal communication.
            @type  intIP:           str

            @param bridgeIP:        IP address of the network interface used for
                                    the container communication.
            @type  bridgeIP:        str

            @param envPort:         Port where the environment process running
                                    inside the container is listening for
                                    connections to other endpoints. (Used for
                                    port forwarding.)
            @type  envPort:         int

            @param rosproxyPort:    Port where the rosproxy process running
                                    inside the container is listening for
                                    connections to console clients. (Used for
                                    port forwarding.)
            @type  rosproxyPort:    int

            @param rootfsDir:       Filesystem path to the root directory of the
                                    container filesystem.
            @type  rootfsDir:       str

            @param confDir:         Filesystem path to the directory where
                                    container configuration files should be
                                    stored.
            @type  confDir:         str

            @param dataDir:         Filesystem path to the directory where
                                    temporary data of a container should be
                                    stored.
            @type  dataDir:         str

            @param pkgDir:          Filesystem paths to the package directories
                                    as a list of tuples where each tuple
                                    contains the path to the directory in the
                                    host machine and the path to the directory
                                    to which the host directory will be bound in
                                    the container filesystem (without the
                                    @param rootfsDir).
            @type  pkgDir:          [(str, str)]

            @param rosRel:          Container filesytem ROS release in this
                                    deployment instance of the cloud engine
            @type  rosRel:          str

            @param data:            More data about the machine configuration.
            @type  data:            dict
        """
        self._reactor = reactor
        self._internalIP = intIP
        self._envPort = envPort
        self._rosproxyPort = rosproxyPort
        self._masterPort = masterPort

        if isLocalhost(masterIP):
            self._masterIP = bridgeIP
        else:
            self._masterIP = masterIP

        self._masterPasswd = masterPasswd
        self._infraPasswd = infraPasswd

        self._rootfs = rootfsDir
        self._confDir = confDir
        self._dataDir = dataDir
        self._pkgDir = pkgDir

        for _, path in self._pkgDir:
            os.mkdir(os.path.join(self._rootfs, path))

        self._nrs = set(range(100, 200))
        self._containers = set()
        self._rosRel = rosRel

        # Network configuration
        self._bridgeIF = bridgeIF
        self._network = bridgeIP[:bridgeIP.rfind('.')]
        self._networkConf = _NETWORK_INTERFACES.format(network=self._network)

        # Virtual network
        self._bridges = set()
        self._uid = {}

        # Physical parameters of machine
        # TODO: Is a human settings at this time,
        #       rce.util.sysinfo should fill this role soon
        self._size = data.get('size')
        self._cpu = data.get('cpu')
        self._memeory = data.get('memory')
        self._bandwidth = data.get('bandwidth')
        self._specialFeatures = data.get('special_features')

        # Common iptables references
        nat = iptc.Table(iptc.Table.NAT)
        self._prerouting = iptc.Chain(nat, 'PREROUTING')
        self._output = iptc.Chain(nat, 'OUTPUT')

    def remote_getSysinfo(self, request):
        """ Get realtime Sysinfo data from machine.

            @param request:     data desired
            @type  request:     # TODO: Add type

            @return:            # TODO: What?
            @rtype:             # TODO: Add type
        """
        # TODO : replace these calls with call to rce.util.sysinfo
        response_table = {
            'size':self._size,
            'cpu':self._cpu,
            'memory': self._memeory,
            'bandwidth': self._bandwidth,
            # 'keyword': some value or function to provide the data
        }

        return response_table[request]

    def remote_setSysinfo(self, request, value):
        """ Set some system parameter to the machine.

            @param request:     data desired
            @type  request:     # TODO: Add type

            @param value:       data value
            @type  value:       # TODO: Add type
        """
        raise NotImplementedError

    @property
    def reactor(self):
        """ Reference to twisted::reactor. """
        return self._reactor

    @property
    def internalIP(self):
        """ IP address of this process in the internal network. """
        return self._internalIP

    @property
    def masterPort(self):
        """ Port of the master process used for internal communications. """
        return self._masterPort

    @property
    def envPort(self):
        """ Port where the environment process running inside the container is
            listening for new connections.
        """
        return self._envPort

    @property
    def rosproxyPort(self):
        """ Port where the ROS proxy running inside the container is listening
            for new connections.
        """
        return self._rosproxyPort

    @property
    def rootfs(self):
        """ Host filesystem path of container filesystem root directory. """
        return self._rootfs

    @property
    def confDir(self):
        """ Filesystem path of configuration directory. """
        return self._confDir

    @property
    def dataDir(self):
        """ Filesystem path of temporary data directory. """
        return self._dataDir

    @property
    def pkgDirIter(self):
        """ Iterator over all file system paths of package directories. """
        return self._pkgDir.__iter__()

    @property
    def rosRel(self):
        """ Container filesytem ROS release in this deployment instance of the
            cloud engine.
        """
        return self._rosRel

    @property
    def bridgeIF(self):
        """ Network interface used for the communication with the containers.
        """
        return self._bridgeIF

    @property
    def masterIP(self):
        """ IP address of master process. """
        return self._masterIP

    @property
    def masterPassword(self):
        """ SHA 256 Digested Master Password. """
        return self._masterPasswd

    @property
    def infraPassword(self):
        """ SHA 256 Digested Infra Password. """
        return self._infraPasswd

    @property
    def prerouting(self):
        """ Reference to iptables' chain PREROUTING of the table NAT. """
        return self._prerouting

    @property
    def output(self):
        """ Reference to iptables' chain OUTPUT of the table NAT. """
        return self._output

    def getNetworkAddress(self):
        """ Get the network's IP address without the last byte as a string.

            @return:            The network's IP address, e.g. '10.0.3'.
            @rtype:             str
        """
        return self._network

    def getNetworkConfigTemplate(self):
        """ Get the template which is used to generate the network
            configuration file '/etc/network/interfaces'. The template contains
            the format field {ip} which has to be used to assign the correct
            IP address to the container.

            @return:            Network configration template containing the
                                format field {ip}.
            @rtype:             str
        """
        return self._networkConf

    def remote_createContainer(self, uid, data):
        """ Create a new Container.

            @param uid:         Unique ID which the environment process inside
                                the container needs to login to the Master
                                process.
            @type  uid:         str

            @param data:        Extra data which is used to configure the
                                container.
            @type  data:        dict

            @return:            New Container instance.
            @rtype:             rce.container.RCEContainer
        """
        try:
            nr = self._nrs.pop()
        except KeyError:
            raise MaxNumberExceeded('Can not manage any additional container.')

        container = RCEContainer(self, nr, uid, data)
        return container.start().addCallback(lambda _: container)

    def registerContainer(self, container):
        assert container not in self._containers
        self._containers.add(container)

    def remote_createBridge(self, name):
        """ Create a new OVS Bridge.

            @param name:        Unique name of the network group.
            @type  name:        str

            @return:            Exit status of command.
            @rtype:             twisted.internet.defer.Deferred
        """
        if name in self._bridges:
            raise InternalError('Bridge already exists.')

        self._bridges.add(name)
        return execute(('/usr/bin/ovs-vsctl', '--', '--may-exist', 'add-br',
                        'br-{0}'.format(name)), reactor=self._reactor)

    def remote_destroyBridge(self, name):
        """ Destroy a OVS Bridge.

            @param name:        Unique name of the network group.
            @type  name:        str

            @return:            Exit status of command.
            @rtype:             twisted.internet.defer.Deferred
        """
        if name not in self._bridges:
            raise InternalError('Bridge does not exist.')

        self._bridges.remove(name)
        return execute(('/usr/bin/ovs-vsctl', 'del-br',
                        'br-{0}'.format(name)), reactor=self._reactor)


    def remote_createTunnel(self, name, targetIP):
        """ Create a new GRE Tunnel.

            @param name:        Unique name of the network group.
            @type  name:        str

            @param targetIP:    Target IP for the GRE Tunnel.
            @type  targetIP:    str

            @return:            Exit status of command.
            @rtype:             twisted.internet.defer.Deferred
        """
        if name not in self._bridges:
            raise InternalError('Bridge does not exist.')

        key = (name, targetIP)

        if key in self._uid:
            raise InternalError('Tunnel already exists.')

        while 1:
            uid = randomString(self._UID_LEN)

            if uid not in self._uid.itervalues():
                break

        self._uid[key] = uid
        port = 'gre-{0}'.format(uid)

        return execute(('/usr/bin/ovs-vsctl', 'add-port', 'br-{0}'.format(name),
                        port, '--', 'set', 'interface', port, 'type=gre',
                        'options:remote_ip={0}'.format(targetIP)),
                       reactor=self._reactor)

    def remote_destroyTunnel(self, name, targetIP):
        """ Destroy a GRE Tunnel.

            @param name:        Unique name of the network group.
            @type  name:        str

            @param targetIP:    Target IP for the GRE Tunnel.
            @type  targetIP:    str

            @return:            Exit status of command.
            @rtype:             twisted.internet.defer.Deferred
        """
        if name not in self._bridges:
            raise InternalError('Bridge does not exist.')

        key = (name, targetIP)

        if key not in self._uid:
            raise InternalError('Tunnel deos not exist.')

        return execute(('/usr/bin/ovs-vsctl', 'del-port',
                        'gre-{0}'.format(self._uid.pop(key))),
                       reactor=self._reactor)

    def unregisterContainer(self, container):
        assert container in self._containers
        self._containers.remove(container)

        def eb(failure):
            if not failure.check(PBConnectionLost):
                log.err(failure)

        try:
            self._avatar.callRemote('containerDied', container).addErrback(eb)
        except (DeadReferenceError, PBConnectionLost):
            pass

    def returnNr(self, nr):
        """ Callback for Container to return a container number when it is
            no longer in use such that it can be reused.
        """
        if nr in self._nrs:
            raise InternalError('Number was never rented out.')

        self._nrs.add(nr)

    def _cleanPackageDir(self, *_):
        """ Internally used method to clean-up the container filesystem.
        """
        for _, path in self._pkgDir:
            os.rmdir(os.path.join(self._rootfs, path))

        assert len(self._containers) == 0

    def terminate(self):
        """ Method should be called to terminate all running containers before
            the reactor is stopped.
        """
        deferreds = []

        for container in self._containers.copy():
            deferreds.append(container.remote_destroy())

        if deferreds:
            deferredList = DeferredList(deferreds)
            deferredList.addCallback(self._cleanPackageDir)
            return deferredList
        else:
            self._cleanPackageDir()


def main(reactor, cred, masterIP, masterPort, masterPassword, infraPasswd,
         bridgeIF, internalIP, bridgeIP, envPort, rosproxyPort, rootfsDir,
         confDir, dataDir, pkgDir, rosRel, data):
    log.startLogging(sys.stdout)

    def _err(reason):
        print(reason)
        reactor.stop()

    factory = PBClientFactory()
    reactor.connectTCP(masterIP, masterPort, factory)

    client = ContainerClient(reactor, masterIP, masterPort, masterPassword,
                             infraPasswd, bridgeIF, internalIP, bridgeIP,
                             envPort, rosproxyPort, rootfsDir, confDir, dataDir,
                             pkgDir, rosRel, data)

    d = factory.login(cred, (client, data))
    d.addCallback(lambda ref: setattr(client, '_avatar', ref))
    d.addErrback(_err)

    reactor.addSystemEventTrigger('before', 'shutdown', client.terminate)
    reactor.run()
