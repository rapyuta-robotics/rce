#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Manager.py
#       
#       This file is part of the RoboEarth Cloud Engine framework.
#       
#       This file was originally created for RoboEearth
#       http://www.roboearth.org/
#       
#       The research leading to these results has received funding from
#       the European Union Seventh Framework Programme FP7/2007-2013 under
#       grant agreement no248942 RoboEarth.
#       
#       Copyright 2012 RoboEarth
#       
#       Licensed under the Apache License, Version 2.0 (the "License");
#       you may not use this file except in compliance with the License.
#       You may obtain a copy of the License at
#       
#       http://www.apache.org/licenses/LICENSE-2.0
#       
#       Unless required by applicable law or agreed to in writing, software
#       distributed under the License is distributed on an "AS IS" BASIS,
#       WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#       See the License for the specific language governing permissions and
#       limitations under the License.
#       
#       \author/s: Dominique Hunziker 
#       
#       

# twisted specific imports
from twisted.python import log
from twisted.internet.defer import Deferred, DeferredList
from twisted.internet.protocol import ProcessProtocol

# Python specific imports
import os
import shutil

# Custom imports
import settings
from NodeManager import ManagerBase
from Comm.Message import MsgDef
from Type import StartContainerMessage, StopContainerMessage
from Processor import StartContainerProcessor, StopContainerProcessor

if settings.USE_SSL:
    from SSLUtil import createKeyCertPair, loadCertFile, loadKeyFile, writeCertToFile, writeKeyToFile

class LXCProtocol(ProcessProtocol):
    """ Protocol which is used to handle the LXC commands.
    """
    def __init__(self, deferred):
        self._deferred = deferred
    
    def processEnded(self, reason):
        self._deferred.callback(reason)

class ContainerManager(ManagerBase):
    """ Manager which handles container specific task.
    """
    def __init__(self, commManager):
        """ Initialize the ContainerManager.
            
            @param commManager:     CommManager which should be used to communicate.
            @type  commManager:     CommManager
        """
        super(ContainerManager, self).__init__(commManager)
        
        # Validate loaded directories from settings
        self._confDir = settings.CONF_DIR
        self._dataDir = settings.DATA_DIR
        self._rootfs = settings.ROOTFS
        self._srcRoot = settings.ROOT_SRC_DIR
        self._pkgRoot = settings.ROOT_PKG_DIR
        
        if not os.path.isabs(self._confDir):
            raise ValueError('Configuration directory is not an absolute path.')
        
        if not os.path.isdir(self._confDir):
            raise OSError('Configuration directory does not exist: {0}'.format(self._confDir))
        
        if not os.path.isabs(self._dataDir):
            raise ValueError('Data directory is not an absolute path.')
        
        if not os.path.isdir(self._dataDir):
            raise OSError('Data directory does not exist: {0}'.format(self._dataDir))
        
        if not os.path.isabs(self._rootfs):
            raise ValueError('Root file system directory is not an absolute path.')
        
        if not os.path.isabs(self._srcRoot):
            raise ValueError('Root source directory is not an absolute path.')
        
        if not os.path.isabs(self._pkgRoot):
            raise ValueError('Root package directory is not an absolute path.')
        
        if not os.path.isdir(self._pkgRoot):
            raise OSError('Root package directory does not exist: {0}'.format(self._pkgRoot))
        
        # Validate the executables used in the container
        environmentExe = os.path.join(self._srcRoot, 'Environment.py')
        launcherExe = os.path.join(self._srcRoot, 'Launcher.py')
        
        if not os.path.isfile(environmentExe):
            raise ValueError('Root source directory does not contain the file "Environment.py".')
        
        if not os.access(environmentExe, os.X_OK):
            raise ValueError('File "Environment.py" in root source directory is not executable.')
        
        if not os.path.isfile(launcherExe):
            raise ValueError('Root source directory does not contain the file "Launcher.py".')
        
        if not os.access(launcherExe, os.X_OK):
            raise ValueError('File "Launcher.py" in root source directory is not executable.')
        
        # Storage of all CommIDs
        self._commIDs = []
        
        # Register Content Serializers
        self._commManager.registerContentSerializers([ StartContainerMessage(),
                                                       StopContainerMessage() ])
        
        # Register Message Processors
        self._commManager.registerMessageProcessors([ StartContainerProcessor(self),
                                                      StopContainerProcessor(self) ])
    
    def _createConfigFile(self, confDir):
        """ Create a config file based on the given parameters.
        """
        content = '\n'.join([ 'lxc.utsname = ros',
                              '',
                              'lxc.tty = 4',
                              'lxc.pts = 1024',
                              'lxc.rootfs = {rootfs}'.format(rootfs=self._rootfs),
                              'lxc.mount = {fstab}'.format(fstab=os.path.join(confDir, 'fstab')),
                              '',
                              'lxc.network.type = veth',
                              'lxc.network.flags = up',
                              'lxc.network.name = eth0',
                              'lxc.network.link = lxcbr0',
                              'lxc.network.ipv4 = 0.0.0.0',
                              '',
                              'lxc.cgroup.devices.deny = a',
                              '# /dev/null and zero',
                              'lxc.cgroup.devices.allow = c 1:3 rwm',
                              'lxc.cgroup.devices.allow = c 1:5 rwm',
                              '# consoles',
                              'lxc.cgroup.devices.allow = c 5:1 rwm',
                              'lxc.cgroup.devices.allow = c 5:0 rwm',
                              'lxc.cgroup.devices.allow = c 4:0 rwm',
                              'lxc.cgroup.devices.allow = c 4:1 rwm',
                              '# /dev/{,u}random',
                              'lxc.cgroup.devices.allow = c 1:9 rwm',
                              'lxc.cgroup.devices.allow = c 1:8 rwm',
                              'lxc.cgroup.devices.allow = c 136:* rwm',
                              'lxc.cgroup.devices.allow = c 5:2 rwm',
                              '# rtc',
                              'lxc.cgroup.devices.allow = c 254:0 rwm',
                              '' ])
        
        with open(os.path.join(confDir, 'config'), 'w') as f:
            f.write(content)
    
    def _createFstabFile(self, confDir, dataDir):
        """ Create a fstab file based on the given parameters.
        """
        content = '\n'.join([ 'proc     {proc}      proc     nodev,noexec,nosuid 0 0'.format(
                                  proc=os.path.join(self._rootfs, 'proc')
                              ),
                              'devpts   {devpts}   devpts   defaults            0 0'.format(
                                  devpts=os.path.join(self._rootfs, 'dev/pts')
                              ),
                              'sysfs    {sysfs}       sysfs    defaults            0 0'.format(
                                  sysfs=os.path.join(self._rootfs, 'sys')
                              ),
                              '{homeDir}    {rootfsHome}    none    bind 0 0'.format(
                                  homeDir=os.path.join(dataDir, 'ros'),
                                  rootfsHome=os.path.join(self._rootfs, 'home/ros')
                              ),
                              '{srcDir}    {rootfsLib}    none    bind,ro 0 0'.format(
                                  srcDir=self._srcRoot,
                                  rootfsLib=os.path.join(self._rootfs, 'opt/rce/src')
                              ),
                              '{pkgDir}    {rootfsPkgs}    none    bind,ro 0 0'.format(
                                  pkgDir=self._pkgRoot,
                                  rootfsPkgs=os.path.join(self._rootfs, 'opt/rce/packages')
                              ),
                              '{dataDir}    {rootfsData}    none    bind 0 0'.format(
                                  dataDir=os.path.join(dataDir, 'rce'),
                                  rootfsData=os.path.join(self._rootfs, 'opt/rce/data')
                              ),
                              '{upstart}    {initDir}    none    bind,ro 0 0'.format(
                                  upstart=os.path.join(confDir, 'upstartComm'),
                                  initDir=os.path.join(self._rootfs, 'etc/init/rceComm.conf')
                              ),
                              '{upstart}   {initDir}   none   bind,ro 0 0'.format(
                                  upstart=os.path.join(confDir, 'upstartLauncher'),
                                  initDir=os.path.join(self._rootfs, 'etc/init/rceLauncher.conf')
                              ),
                              '' ])
        
        with open(os.path.join(confDir, 'fstab'), 'w') as f:
            f.write(content)
    
    def _createUpstartScripts(self, commID, confDir):
        """ Create an upstart script based on the given parameters.
        """
        content = '\n'.join([ '# description',
                              'author "Dominique Hunziker"',
                              'description "ReCloudEngine Comm - Framework for managing and using ROS Apps"',
                              '',
                              '# start/stop conditions',
                              'start on started rce',
                              'stop on stopping rce',
                              '',
                              'kill timeout 5',
                              '',
                              'script',
                              '\t# setup environment',
                              '\t. /opt/rce/setup.sh',
                              '\t',
                              '\t# start environment node',
                              '\t'+' '.join([ 'start-stop-daemon',
                                              '--start',
                                              '-c', 'rce:rce',
                                              '-d', '/opt/rce/data',
                                              '--retry', '5',
                                              '--exec', '/opt/rce/src/Environment.py',
                                              '--',
                                              commID,
                                              '{0}{1}'.format( MsgDef.PREFIX_PUB_ADDR,
                                                               self._commManager.commID[MsgDef.PREFIX_LENGTH_ADDR:])]),
                              'end script',
                              '' ])
       
        with open(os.path.join(confDir, 'upstartComm'), 'w') as f:
            f.write(content)
        
        content = '\n'.join([ '# description',
                              'author "Dominique Hunziker"',
                              'description "ReCloudEngine Launcher - Framework for managing and using ROS Apps"',
                              '',
                              '# start/stop conditions',
                              'start on started rce',
                              'stop on stopping rce',
                              '',
                              '# timeout before the process is killed; generous as a lot of processes have',
                              '# to be terminated by the launcher.',
                              'kill timeout 30',
                              '',
                              'script',
                              '\t# setup environment',
                              '\t. /opt/rce/setup.sh',
                              '\t',
                              '\t# start launcher',
                              '\t'+' '.join([ 'start-stop-daemon',
                                              '--start',
                                              '-c', 'ros:ros',
                                              '-d', '/home/ros',
                                              '--retry', '5',
                                              '--exec', '/opt/rce/src/Launcher.py' ]),
                              'end script',
                              '' ])
       
        with open(os.path.join(confDir, 'upstartLauncher'), 'w') as f:
            f.write(content)
    
    def _startContainer(self, deferred, commID):
        """ Internally used method to start a container.
        """
        # Assemble path for config and fstab file
        confDir = os.path.join(self._confDir, commID)
        
        if os.path.isdir(confDir):
            msg = 'There exists already a configuration directory with the name "{0}".'.format(commID)
            log.msg(msg)
            deferred.errback(msg)
            return
        
        # Assemble path for temporary data
        dataDir = os.path.join(self._dataDir, commID)
        
        if os.path.isdir(dataDir):
            msg = 'There exists already a configuration directory with the name "{0}".'.format(commID)
            log.msg(msg)
            deferred.errback(msg)
            return
        
        try:
            log.msg('Create files and directories...')
            os.mkdir(confDir)
            os.mkdir(dataDir)
            
            rceDir = os.path.join(dataDir, 'rce')
            rosDir = os.path.join(dataDir, 'ros')
            os.mkdir(rceDir)
            os.mkdir(rosDir)
            
            if settings.USE_SSL:
                # Create a new certificate and key for environment node
                caCertPath = os.path.join(settings.SSL_DIR, 'Container.cert')
                caCert = loadCertFile(caCertPath)
                caKey = loadKeyFile(os.path.join(settings.SSL_DIR, 'container/env.key'))
                (cert, key) = createKeyCertPair(commID, caCert, caKey)
                
                # Copy/save file to data directory
                shutil.copyfile(caCertPath, os.path.join(rceDir, 'ca.pem'))
                writeCertToFile(cert, os.path.join(rceDir, 'cert.pem'))
                writeKeyToFile(key, os.path.join(rceDir, 'key.pem'))
            
            # Construct config file
            self._createConfigFile(confDir)
            
            # Construct fstab file
            self._createFstabFile(confDir, dataDir)
            
            # Construct startup scripts
            self._createUpstartScripts(commID, confDir)
        except:
            log.msg('Caught and exception when trying to create the config files for the container.')
            import sys, traceback
            etype, value, _ = sys.exc_info()
            msg = '\n'.join(traceback.format_exception_only(etype, value))
            log.msg(msg)
            deferred.errback(msg)
            return
        
        # Start container
        _deferred = Deferred()
        
        def callback(reason):
            if reason.value.exitCode != 0:
                log.msg('Received reason with exit code != 0 after start of container.')
                log.msg(reason)
                deferred.errback(reason.getErrorMessage())
            else:
                deferred.callback(None)
        
        _deferred.addCallbacks(callback, callback)
        
        try:
            log.msg('Start container...')
            cmd = [ '/usr/bin/lxc-start',
                    '-n', commID,
                    '-f', os.path.join(confDir, 'config'),
                    '-d' ]
            self.reactor.spawnProcess(LXCProtocol(_deferred), cmd[0], cmd, env=os.environ)
        except:
            log.msg('Caught an exception when trying to start the container.')
            import sys, traceback #@Reimport
            etype, value, _ = sys.exc_info()
            msg = '\n'.join(traceback.format_exception_only(etype, value))
            log.msg(msg)
            deferred.errback(msg)
            return
        else:
            self.reactor.callFromThread(self._commIDs.append, commID)
    
    def startContainer(self, commID):
        """ Callback for message processor to stop a container.
        """
        deferred = Deferred()
        
        if commID in self._commIDs:
            log.msg('There is already a container registered under the same CommID.')
            deferred.errback('There is already a container registered under the same CommID.')
            return
        
        self.reactor.callInThread(self._startContainer, deferred, commID)
        
        def reportSuccess(_):
            log.msg('Container successfully started.')
        
        def reportFailure(msg):
            log.msg('Container could not be started: {0}.'.format(msg))
        
        deferred.addCallbacks(reportSuccess, reportFailure)
    
    def _stopContainer(self, deferred, commID):
        """ Internally used method to stop a container.
        """
        # Stop container
        _deferred = Deferred()
        
        def callback(reason):
            error = None
            
            if reason.value.exitCode != 0:
                log.msg('Received reason with exit code != 0 after stopping container.')
                log.msg(reason)
                error = reason.getErrorMessage()
            
            try:
                # Delete config folder
                shutil.rmtree(os.path.join(self._confDir, commID))
                
                # Delete data folder
                shutil.rmtree(os.path.join(self._dataDir, commID))
                
                # Remove commID from internal list
                self.reactor.callFromThread(self._commIDs.remove, commID)
            except:
                import sys, traceback
                etype, value, _ = sys.exc_info()
                error = '\n'.join(traceback.format_exception_only(etype, value))
            
            if error:
                deferred.errback(error)
            else:
                deferred.callback(None)
        
        _deferred.addCallbacks(callback, callback)
        
        try:
            cmd = ['/usr/bin/lxc-stop', '-n', commID]
            self.reactor.spawnProcess(LXCProtocol(_deferred), cmd[0], cmd, env=os.environ)
        except:
            import sys, traceback
            etype, value, _ = sys.exc_info()
            msg = '\n'.join(traceback.format_exception_only(etype, value))
            log.msg(msg)
            deferred.errback(msg)
            return
    
    def stopContainer(self, commID):
        """ Callback for message processor to stop a container.
        """
        deferred = Deferred()
        
        if commID not in self._commIDs:
            log.msg('There is no container registered under this CommID.')
            deferred.errback('There is no container registered under this CommID.')
            return
        
        self.reactor.callInThread(self._stopContainer, deferred, commID)
        
        def reportSuccess(_):
            log.msg('Container successfully stopped.')
        
        def reportFailure(msg):
            log.msg('Container could not be stopped: {0}.'.format(msg))
        
        deferred.addCallbacks(reportSuccess, reportFailure)
    
    def shutdown(self):
        """ Method is called when the manager is stopped.
        """
        if self._commIDs:
            deferreds = []
            
            for commID in self._commIDs:
                deferred = Deferred()
                deferreds.append(deferred)
                self._stopContainer(deferred, commID)
            
            deferredList = DeferredList(deferreds)
            return deferredList
