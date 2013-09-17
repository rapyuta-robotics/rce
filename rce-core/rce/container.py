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

try:
    import pkg_resources
except ImportError:
    print("Can not import the package 'pkg_resources'.")
    exit(1)

pjoin = os.path.join
load_resource = pkg_resources.resource_string  #@UndefinedVariable

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


_UPSTART_COMM = load_resource('rce.core', 'data/comm.upstart')
# _UPSTART_LAUNCHER = load_resource('rce.core', 'data/launcher.upstart')
_UPSTART_ROSAPI = load_resource('rce.core', 'data/rosapi.upstart')
_LXC_NETWORK_SCRIPT = load_resource('rce.core', 'data/lxc-network.script')


def passthrough(f):
    """ Decorator which is used to add a function as a Deferred callback and
        passing the input unchanged to the output.

        @param f:           Function which should be decorated.
        @type  f:           callable
    """
    def wrapper(response):
        f()
        return response

    return wrapper


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
        self._client = client
        self._nr = nr
        self._name = name = 'C{0}'.format(nr)
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

        # Create the directories for the container
        self._confDir = confDir = pjoin(client.confDir, name)
        self._dataDir = dataDir = pjoin(client.dataDir, name)

        if os.path.isdir(confDir):
            raise ValueError('There is already a configuration directory for '
                             "'{0}' \n Please remove it manually if the engine "
                             'did not shut down correctly on last execution and '
                             'you are sure it is not in use. \n dir: {1}.'.format(name, confDir))

        if os.path.isdir(dataDir):
            raise ValueError('There is already a data directory for '
                             "'{0}' \n Please remove it manually if the engine "
                             'did not shut down correctly on last execution and '
                             'you are sure it is not in use. \n dir: {1}.'.format(name, dataDir))
        os.mkdir(confDir)
        os.mkdir(dataDir)

        # Create additional folders for the container
        rceDir = pjoin(dataDir, 'rce')
        rosDir = pjoin(dataDir, 'ros')

        os.mkdir(rceDir)
        os.mkdir(rosDir)

        if client.rosRel > 'fuerte':
            # TODO: Switch to user 'ros' when the launcher is used again
            shutil.copytree(pjoin(client.rootfs, 'root/.ros/rosdep'),
                            pjoin(rceDir, '.ros/rosdep'))

        # Create network variables
        bridgeIP = client.bridgeIP
        ip = '{0}.{1}'.format(bridgeIP.rsplit('.', 1)[0], nr)
        self._address = '{0}:{1}'.format(ip, client.envPort)
        self._rosproxyAddress = '{0}:{1}'.format(ip, client.rosproxyPort)
        self._fwdPort = str(nr + 8700)
        self._rosproxyFwdPort = str(nr + 10700)

        ovsname = data.get('name')
        ovsip = data.get('ip')

        if ovsname and ovsip:
            ovsif = 'eth1'
            ovsup = pjoin(confDir, 'ovsup')

            if client.ubuntuRel > 'quantal':
                ovsdown = pjoin(confDir, 'ovsdown')
            else:
                ovsdown = None
        else:
            ovsif = ovsup = ovsdown = None

        # Construct password
        passwd = encodeAES(cipher(client.masterPassword),
                           salter(uid, client.infraPassword))

        # Create upstart scripts
        upComm = pjoin(confDir, 'upstartComm')
        with open(upComm, 'w') as f:
            f.write(_UPSTART_COMM.format(masterIP=client.masterIP,
                                         masterPort=client.masterPort,
                                         internalPort=client.envPort,
                                         uid=uid, passwd=passwd))

        upRosapi = pjoin(confDir, 'upstartRosapi')
        with open(upRosapi, 'w') as f:
            f.write(_UPSTART_ROSAPI.format(proxyPort=client.rosproxyPort))

        # TODO: For the moment there is no upstart script for the launcher.
#        upLauncher = pjoin(confDir, 'upstartLauncher')
#        with open(upLauncher, 'w') as f:
#            f.write(_UPSTART_LAUNCHER)

        # Setup network
        networkIF = pjoin(confDir, 'networkInterfaces')
        with open(networkIF, 'w') as f:
            f.write('auto lo\n')
            f.write('iface lo inet loopback\n')
            f.write('\n')
            f.write('auto eth0\n')
            f.write('iface eth0 inet static\n')
            f.write('    address {0}\n'.format(ip))
            f.write('    gateway {0}\n'.format(bridgeIP))
            f.write('    dns-nameservers {0} 127.0.0.1\n'.format(bridgeIP))

            if ovsif:
                f.write('\n')
                f.write('auto {0}\n'.format(ovsif))
                f.write('iface {0} inet static\n'.format(ovsif))
                f.write('    address {0}\n'.format(ovsip))

        # Create up/down script for virtual network interface if necessary
        if ovsup:
            with open(ovsup, 'w') as f:
                f.write(_LXC_NETWORK_SCRIPT.format(if_op='up', ovs_op='add',
                                                   name=ovsname))

            os.chmod(ovsup, stat.S_IRWXU)

        if ovsdown:
            with open(ovsdown, 'w') as f:
                f.write(_LXC_NETWORK_SCRIPT.format(if_op='down', ovs_op='del',
                                                   name=ovsname))

            os.chmod(ovsdown, stat.S_IRWXU)

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

        # Create the container
        self._container = container = Container(client.reactor, client.rootfs,
                                                confDir, name)

        # Add lxc bridge
        container.addNetworkInterface('eth0', client.bridgeIF, ip)

        # Add the virtual network bridge if necessary
        if ovsname and ovsip:
            container.addNetworkInterface(ovsif, None, ovsip, ovsup, ovsdown)

        # Add additional lines to fstab file of container
        container.extendFstab(rosDir, 'home/ros', False)
        container.extendFstab(rceDir, 'opt/rce/data', False)
        container.extendFstab(upComm, 'etc/init/rceComm.conf', True)
        # TODO: For the moment there is no upstart script for the launcher.
#        container.extendFstab(upLauncher, 'etc/init/rceLauncher.conf', True)
        container.extendFstab(upRosapi, 'etc/init/rceRosapi.conf', True)
        container.extendFstab(networkIF, 'etc/network/interfaces', True)

        for srcPath, destPath in client.pkgDirIter:
            container.extendFstab(srcPath, destPath, True)

    def start(self):
        """ Method which starts the container.
        """
        # NOTE: can raise iptc.xtables.XTablesError
        # add remote rule for RCE internal communication
        rule = iptc.Rule()
        rule.protocol = 'tcp'
        rule.dst = self._client.internalIP
        m = rule.create_match('tcp')
        m.dport = self._fwdPort
        t = rule.create_target('DNAT')
        t.to_destination = self._address
        self._remoteRule = rule

        # add local (loopback) rule for RCE internal communication
        rule = iptc.Rule()
        rule.protocol = 'tcp'
        rule.out_interface = 'lo'
        rule.dst = self._client.internalIP
        m = rule.create_match('tcp')
        m.dport = self._fwdPort
        t = rule.create_target('DNAT')
        t.to_destination = self._address
        self._localRule = rule

        # add remote rule for rosproxy
        rule = iptc.Rule()
        rule.protocol = 'tcp'
        rule.dst = self._client.internalIP
        m = rule.create_match('tcp')
        m.dport = self._rosproxyFwdPort
        t = rule.create_target('DNAT')
        t.to_destination = self._rosproxyAddress
        self._rosremoteRule = rule

        # add local(loopback) rule for rosproxy
        rule = iptc.Rule()
        rule.protocol = 'tcp'
        rule.out_interface = 'lo'
        rule.dst = self._client.internalIP
        m = rule.create_match('tcp')
        m.dport = self._rosproxyFwdPort
        t = rule.create_target('DNAT')
        t.to_destination = self._rosproxyAddress
        self._roslocalRule = rule

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

    def _destroy(self):
        """ Internally used method to clean up after the container has been
            stopped.
        """
        if self._client:
            self._client.returnNr(self._nr)
            self._client.unregisterContainer(self)
            self._client = None

        if self._confDir:
            shutil.rmtree(self._confDir, True)
            self._confDir = None

        if self._dataDir:
            shutil.rmtree(self._dataDir, True)
            self._dataDir = None

    def remote_destroy(self):
        """ Method should be called to destroy the container.
        """
        if not self._terminating:
            if self._container:
                self._terminating = self._stop()
                self._terminating.addBoth(passthrough(self._destroy))
            else:
                self._terminating = succeed(None)

        return self._terminating

    def __del__(self):
        self._destroy()


class ContainerClient(Referenceable):
    """ Container client is responsible for the creation and destruction of
        containers in a machine.

        There can be only one Container Client per machine.
    """
    _UID_LEN = 8

    def __init__(self, reactor, masterIP, masterPort, masterPasswd, infraPasswd,
                 bridgeIF, intIP, bridgeIP, envPort, rosproxyPort, rootfsDir,
                 confDir, dataDir, pkgDir, ubuntuRel, rosRel, data):
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

            @param ubuntuRel:       Host filesystem Ubuntu release used in this
                                    machine.
            @type  ubuntuRel:       str

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

        # Container directories
        self._rootfs = rootfsDir
        self._confDir = confDir
        self._dataDir = dataDir
        self._pkgDir = pkgDir

        # Release info
        self._ubuntuRel = ubuntuRel
        self._rosRel = rosRel

        for _, path in self._pkgDir:
            os.mkdir(os.path.join(self._rootfs, path))

        # Container info
        self._nrs = set(range(100, 200))
        self._containers = set()

        # Network configuration
        self._bridgeIF = bridgeIF
        self._bridgeIP = bridgeIP

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
    def ubuntuRel(self):
        """ Host filesystem Ubuntu release in this machine. """
        return self._ubuntuRel

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
    def bridgeIP(self):
        """ IP address of network interface used for the communication with
            the containers.
        """
        return self._bridgeIP

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
         confDir, dataDir, pkgDir, ubuntuRel, rosRel, data):
    log.startLogging(sys.stdout)

    def _err(reason):
        print(reason)
        reactor.stop()

    factory = PBClientFactory()
    reactor.connectTCP(masterIP, masterPort, factory)

    client = ContainerClient(reactor, masterIP, masterPort, masterPassword,
                             infraPasswd, bridgeIF, internalIP, bridgeIP,
                             envPort, rosproxyPort, rootfsDir, confDir, dataDir,
                             pkgDir, ubuntuRel, rosRel, data)

    d = factory.login(cred, (client, data))
    d.addCallback(lambda ref: setattr(client, '_avatar', ref))
    d.addErrback(_err)

    reactor.addSystemEventTrigger('before', 'shutdown', client.terminate)
    reactor.run()
