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
import shutil

pjoin = os.path.join

try:
    import iptc
except ImportError:
    print 'python-iptables could not be imported.'
    print '    see: http://github.com/ldx/python-iptables'
    exit(1)

# twisted specific imports
from twisted.python import log
from twisted.internet.defer import  DeferredList, succeed
from twisted.spread.pb import Referenceable, PBClientFactory, \
    DeadReferenceError, PBConnectionLost

# Custom imports
from rce.error import InternalError, MaxNumberExceeded
from rce.util.container import Container
from rce.util.network import getIP
from rce.util.path import processPkgPath, checkPath, checkExe
#from rce.util.ssl import createKeyCertPair, loadCertFile, loadKeyFile, \
#    writeCertToFile, writeKeyToFile


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
    
    # start environment node
    start-stop-daemon --start -c rce:rce -d /opt/rce/data --retry 5 --exec /opt/rce/src/environment.py -- {masterIP} {uid}
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

    #start rosapi node
    start-stop-daemon --start -c rce:rce -d /opt/rce/data --retry 5 --exec /opt/rce/src/rosproxy.py
end script
"""

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


class RCEContainer(Referenceable):
    """ Container representation which is used to run a ROS environment.
    """
    def __init__(self, client, status, nr, uid):
        """ Initialize the deployment container.
            
            @param client:      Container client which is responsible for
                                monitoring the containers in this machine.
            @type  client:      rce.container.ContainerClient
            
            @param status:      Status observer which is used to inform the
                                Master of the container's status.
            @type  status:      twisted.spread.pb.RemoteReference
            
            @param nr:          Unique number which will be used for the IP
                                address and the hostname of the container.
            @type  nr:          int
            
            @param uid:         Unique ID which is used by the environment
                                process to login to the Master.
            @type  uid:         str
        """
        # Store the references
        self._client = client
        client.registerContainer(self)
        
        self._status = status
        self._nr = nr
        self._name = 'C{0}'.format(nr)
        self._terminating = None
        
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
        
        # Create network variables
        ip = '{0}.{1}'.format(client.getNetworkAddress(), nr)
        self._address = '{0}:{1}'.format(ip, client.envPort)
        self._rosproxyaddress = '{0}:{1}'.format(ip, client.rosproxyPort)
        self._fwdPort = str(nr + 8700)
        self._rosproxyfwdPort = str(nr + 10700)
        
        self._container = Container(client.reactor, client.rootfs,
                                    self._confDir, self._name, ip)
        
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
        
        # Add additional lines to fstab file of container
        self._container.extendFstab(rosDir, 'home/ros', False)
        self._container.extendFstab(rceDir, 'opt/rce/data', False)
        self._container.extendFstab(client.srcDir, 'opt/rce/src', True)
        self._container.extendFstab(pjoin(self._confDir, 'upstartComm'),
                                    'etc/init/rceComm.conf', True)
        # TODO: For the moment there is no upstart launcher.
#        self._container.extendFstab(pjoin(self._confDir, 'upstartLauncher'),
#                                    'etc/init/rceLauncher.conf', True)
        self._container.extendFstab(pjoin(self._confDir, 'upstartRosapi'),
                                    'etc/init/rceRosapi.conf', True)
        
        self._container.extendFstab(pjoin(self._confDir, 'networkInterfaces'),
                                    'etc/network/interfaces', False)
        
        for srcPath, destPath in client.pkgDirIter:
            self._container.extendFstab(srcPath, destPath, True)
        
        # Create upstart scripts
        with open(pjoin(self._confDir, 'upstartComm'), 'w') as f:
            f.write(_UPSTART_COMM.format(masterIP=self._client.masterIP,
                                         uid=uid))
        
        with open(pjoin(self._confDir, 'upstartRosapi'), 'w') as f:
            f.write(_UPSTART_ROSAPI)

        # TODO: For the moment there is no upstart launcher.
#        with open(pjoin(self._confDir, 'upstartLauncher'), 'w') as f:
#            f.write(_UPSTART_LAUNCHER)
        
        # Setup network
        with open(pjoin(self._confDir, 'networkInterfaces'), 'w') as f:
            f.write(client.getNetworkConfigTemplate().format(ip=ip))
    
    def start(self):
        """ Method which starts the container.
        """
        # can raise iptc.xtables.XTablesError
        #add remote rule
        self._remoteRule = iptc.Rule()
        self._remoteRule.protocol = 'tcp'
        self._remoteRule.dst = self._client.internalIP
        m = self._remoteRule.create_match('tcp')
        m.dport = self._fwdPort
        t = self._remoteRule.create_target('DNAT')
        t.to_destination = self._address
        
        #add local(loopback) rule
        self._localRule = iptc.Rule()
        self._localRule.protocol = 'tcp'
        self._localRule.out_interface = 'lo'
        self._localRule.dst = self._client.internalIP
        m = self._localRule.create_match('tcp')
        m.dport = self._fwdPort
        t = self._localRule.create_target('DNAT')
        t.to_destination = self._address

        self._client.prerouting.insert_rule(self._remoteRule)
        self._client.output.insert_rule(self._localRule)
        
        
        #rules for rosproxy
        #add remote rule
        self._rosremoteRule = iptc.Rule()
        self._rosremoteRule.protocol = 'tcp'
        self._rosremoteRule.dst = self._client.internalIP
        m = self._rosremoteRule.create_match('tcp')
        m.dport = self._rosproxyfwdPort
        t = self._rosremoteRule.create_target('DNAT')
        t.to_destination = self._rosproxyaddress
        
        #add local(loopback) rule
        self._roslocalRule = iptc.Rule()
        self._roslocalRule.protocol = 'tcp'
        self._roslocalRule.out_interface = 'lo'
        self._roslocalRule.dst = self._client.internalIP
        m = self._roslocalRule.create_match('tcp')
        m.dport = self._rosproxyfwdPort
        t = self._roslocalRule.create_target('DNAT')
        t.to_destination = self._rosproxyaddress

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
        
        if self._status:
            def eb(failure):
                if not failure.check(PBConnectionLost):
                    log.err(failure)
            
            try:
                self._status.callRemote('died').addErrback(eb)
            except (DeadReferenceError, PBConnectionLost):
                pass
            
            self._status = None
        
        return self._terminating


class ContainerClient(Referenceable):
    """ Container client is responsible for the creation and destruction of
        containers in a machine.
        
        There can be only one Container Client per machine.
    """
    def __init__(self, reactor, masterIP, intIF, bridgeIF, envPort, 
                 rosproxyPort, rootfsDir, confDir, dataDir, srcDir, pkgDir):
        """ Initialize the Container Client.
            
            @param reactor:     Reference to the twisted reactor.
            @type  reactor:     twisted::reactor
            
            @param masterIP:    IP address of the Master process.
            @type  masterIP:    str
            
            @param intIF:       Name of the network interface used for the
                                internal network.
            @type  intIF:       str
            
            @param bridgeIF:    Name of the bridge interface used for the
                                container network.
            @type  bridgeIF:    str
            
            @param envPort:     Port where the environment process running
                                inside the container is listening for
                                connections to other endpoints. (Used for
                                port forwarding.)
            @type  envPort:     int
            
            @param rootfsDir:   Filesystem path to the root directory of the
                                container filesystem.
            @type  rootfsDir:   str
            
            @param confDir:     Filesystem path to the directory where
                                container configuration files should be stored.
            @type  confDir:     str
            
            @param dataDir:     Filesystem path to the directory where
                                temporary data of a container should be stored.
            @type  dataDir:     str
            
            @param srcDir:      Filesystem path to the directory where the
                                source of the cloud engine is located.
            @type  srcDir:      str
            
            @param pkgDir:      Filesystem paths to the package directories
                                as a list of tuples where each tuple contains
                                the path to the directory in the host machine
                                and the path to the directory to which the
                                host directory will be bound in the container
                                filesystem (without the @param rootfsDir).
            @type  pkgDir:      [(str, str)]
        """
        self._reactor = reactor
        self._internalIP = getIP(intIF)
        self._envPort = envPort
        self._rosproxyPort = rosproxyPort
        
        bridgeIP = getIP(bridgeIF)
        
        if masterIP in ('localhost', '127.0.0.1'):
            self._masterIP = bridgeIP
        else:
            self._masterIP = masterIP
        
        self._rootfs = rootfsDir
        self._confDir = confDir
        self._dataDir = dataDir
        self._srcDir = srcDir
        
        # Validate directory paths
        checkPath(self._confDir, 'Configuration')
        checkPath(self._dataDir, 'Data')
        checkPath(self._rootfs, 'Container file system')
        checkPath(self._srcDir, 'RCE source')
        
        # Validate executable paths
        checkExe(self._srcDir, 'environment.py')
        checkExe(self._srcDir, 'rosproxy.py')
        #checkExe(self._srcDir, 'launcher.py')
        
        # Process ROS package paths
        self._pkgDir = processPkgPath(pkgDir)
        
        for _, path in self._pkgDir:
            os.mkdir(os.path.join(self._rootfs, path))
        
        self._nrs = set(range(100, 200))
        self._containers = set()
        
        # Network configuration
        self._network = bridgeIP[:bridgeIP.rfind('.')]
        self._networkConf = _NETWORK_INTERFACES.format(network=self._network)
        
        # Common iptables references
        nat = iptc.Table(iptc.Table.NAT)
        self._prerouting = iptc.Chain(nat, 'PREROUTING')
        self._output = iptc.Chain(nat, 'OUTPUT')
    
    @property
    def reactor(self):
        """ Reference to twisted::reactor. """
        return self._reactor
    
    @property
    def internalIP(self):
        """ IP address of this process in the internal network. """
        return self._internalIP
    
    @property
    def envPort(self):
        """ Port where the environment process running inside the container is
            for new connections.
        """
        return self._envPort
    
    @property
    def rosproxyPort(self):
        """ Port where the environment process running inside the container is
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
    def srcDir(self):
        """ Filesystem path of RCE source directory. """
        return self._srcDir
    
    @property
    def pkgDirIter(self):
        """ Iterator over all file system paths of package directories. """
        return self._pkgDir.__iter__()
    
    @property
    def masterIP(self):
        """ IP address of master process. """
        return self._masterIP
    
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
    
    def remote_createContainer(self, status, uid):
        """ Create a new Container.
            
            @param status:      Status observer which is used to inform the
                                Master of the container's status.
            @type  status:      twisted.spread.pb.RemoteReference
            
            @param uid:         Unique ID which the environment process inside
                                the container needs to login to the Master
                                process.
            @type  uid:         str
            
            @return:            New Container instance.
            @rtype:             rce.container.RCEContainer
        """
        try:
            nr = self._nrs.pop()
        except KeyError:
            raise MaxNumberExceeded('Can not manage any additional container.')
        
        container = RCEContainer(self, status, nr, uid)
        return container.start().addCallback(lambda _: container)
    
    def registerContainer(self, container):
        assert container not in self._containers
        self._containers.add(container)
    
    def unregisterContainer(self, container):
        print 'containers', self._containers
        assert container in self._containers
        self._containers.remove(container)
    
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


def main(reactor, cred, masterIP, masterPort, internalIF, bridgeIF, envPort, 
         rosproxyPort, rootfsDir, confDir, dataDir, srcDir, pkgDir, maxNr):
    log.startLogging(sys.stdout)
    
    def _err(reason):
        print(reason)
        reactor.stop()
    
    factory = PBClientFactory()
    reactor.connectTCP(masterIP, masterPort, factory)
    
    client = ContainerClient(reactor, masterIP, internalIF, bridgeIF, envPort, 
                             rosproxyPort, rootfsDir, confDir, dataDir, srcDir, 
                             pkgDir)
    
    d = factory.login(cred, (client, maxNr))
    d.addCallback(lambda ref: setattr(client, '_avatar', ref))
    d.addErrback(_err)
    
    reactor.addSystemEventTrigger('before', 'shutdown', client.terminate)
    reactor.run()
