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
from tempfile import mkdtemp

pjoin = os.path.join

# twisted specific imports
from twisted.python import log
from twisted.internet.defer import Deferred
from twisted.internet.protocol import ProcessProtocol

# Custom imports
from errors import InternalError
from util import template
from util import path as pathUtil

#from util.ssl import createKeyCertPair, loadCertFile, loadKeyFile, \
#    writeCertToFile, writeKeyToFile


class _LXCProtocol(ProcessProtocol):
    """ Protocol which is used to handle the LXC commands.
    """
    def __init__(self, deferred, out=None, err=None):
        self._deferred = deferred
        
        if out and not callable(out):
            raise InternalError('stdout of LXC protocol has to be None or '
                                'callable')
        
        if err and not callable(err):
            raise InternalError('errout of LXC protocol has to be None or '
                                'callable')
        
        self._out = out
        self._err = err
    
    def outReceived(self, data):
        if self._out:
            self._out(data)
    
    def errReceived(self, data):
        if self._err:
            self._err(data)
    
    def processEnded(self, reason):
        if reason.value.exitCode == 0:
            self._deferred.callback(self)
        else:
            self._deferred.errback('Received non zero exit code from LXC: '
                                   '{0}'.format(reason.getErrorMessage()))


class Container(object):
    """ Class representing a single container.
    """
    def __init__(self, reactor, rootfs, conf):
        """ Initialize the Container.
            
            @param reactor:     Reference to the twisted::reactor
            @type  reactor:     twisted::reactor
            
            @param rootfs:      Filesystem path of the root directory of the
                                container filesystem.
            @type  rootfs:      str
            
            @param conf:        Filesystem path of folder where configuration
                                files for the container should be stored.
            @type  conf:        str
        """
        self._reactor = reactor
        self._rootfs = rootfs
        self._conf = pjoin(conf, 'config')
        self._fstab = pjoin(conf, 'fstab')
        
        pathUtil.checkPath(conf, 'Container Configuration')
        
        if os.path.exists(self._conf):
            raise ValueError('There is already a config file in given '
                             'configuration folder.')
        
        if os.path.exists(self._fstab):
            raise ValueError('There is already a fstab file in given '
                             'configuration folder.')
        
        self._fstabExt = []
    
    def extendFstab(self, src, fs, ro):
        """ Add a line to the Fstab file using bind.
            
            @param src:     Source path in host filesystem.
            @type  src:     str
            
            @param fs:      Path in container filesystem to which the source
                            should be bind.
            @type  fs:      str
            
            @param ro:      Flag to indicate whether bind should be read-only
                            or not.
            @type  ro:      bool
        """
        if ro:
            line = template.FSTAB_BIND_RO
        else:
            line = template.FSTAB_BIND
        
        self._fstabExt.append(line.format(srcDir=src,
                                          fsDir=pjoin(self._rootfs, fs)))
    
    def _setup(self, deferred, out=None, err=None):
        """ Setup necessary files.
        """
        with open(self._conf, 'w') as f:
            f.write(template.CONFIG.format(rootfs=self._rootfs,
                                           fstab=self._fstab))
        
        with open(self._fstab, 'w') as f:
            f.write(template.FSTAB_BASE.format(
                proc=pjoin(self._rootfs, 'proc'),
                devpts=pjoin(self._rootfs, 'dev/pts'),
                sysfs=pjoin(self._rootfs, 'sys')))
            f.writelines(self._fstabExt)
        
        return _LXCProtocol(deferred, out, err)
    
    def start(self, name, deferred):
        """ Start the container.
            
            @param name:        Name of the container which should be started.
            @type  name:        str
            
            @param command:     Deferred whose callback is triggered on success
                                with this Container instance as argument or
                                whose errback is triggered on failure with an
                                error message.
            @type  command:     twisted::Deferred
        """
        protocol = self._setup(deferred)
        
        try:
            log.msg("Start container '{0}'".format(name))
            cmd = ['/usr/bin/lxc-start', '-n', name, '-f', self._conf, '-d']
            self._reactor.spawnProcess(protocol, cmd[0], cmd, env=os.environ)
        except Exception as e:
            log.msg('Caught an exception when trying to start the container.')
            import traceback
            deferred.errback('\n'.join(traceback.format_exception_only(type(e),
                                                                       e)))
    
    def stop(self, name, deferred):
        """ Stop the container.
            
            @param name:        Name of the container which should be stopped.
            @type  name:        str
            
            @param command:     Deferred whose callback is triggered on success
                                or whose errback is triggered on failure with
                                an error message.
            @type  command:     twisted::Deferred
        """
        try:
            cmd = ['/usr/bin/lxc-stop', '-n', name]
            self._reactor.spawnProcess(_LXCProtocol(deferred), cmd[0], cmd,
                                       env=os.environ)
        except Exception as e:
            import traceback
            msg = '\n'.join(traceback.format_exception_only(type(e), e))
            log.msg(msg)
            deferred.errback(msg)
    
    def execute(self, name, command):
        """ Execute a command inside the container.
            
            @param name:        Name of the container which will execute the
                                command.
            @type  name:        str
            
            @param command:     Command which should be executed.
            @type  command:     [str]
        """
        def cb(_):
            print('\nSuccessful.')
            self._reactor.stop()
        
        def eb(err):
            print('\n{0}'.format(err.getErrorMessage()))
            self._reactor.stop()
        
        deferred = Deferred()
        deferred.addCallbacks(cb, eb)
        
        self.extendFstab('/usr/lib/lxc', pjoin(self._rootfs, 'usr/lib/lxc'),
                         False)
        
        protocol = self._setup(deferred, sys.stdout.write, sys.stderr.write)
        
        try:
            cmd = ['/usr/bin/lxc-execute', '-n', name, '-f', self._conf]
            cmd += command
            self._reactor.spawnProcess(protocol, cmd[0], cmd, env=os.environ)
        except Exception as e:
            import traceback
            print('Caught an exception when trying to execute a command in '
                  'the container.')
            print('\n'.join(traceback.format_exception_only(type(e), e)))


class DeploymentContainer(Container):
    """ Container representation for which is used to run a ROS environment
        inside it. It is used for normal RCE operation.
    """
    def __init__(self, manager, commID):
        """ Initialize the deployment container.
            
            @param manager:     Container Manager which will manage this
                                container.
            @type  manager:     ContainerManager
            
            @param commID:      Communication ID of the Environment Manager
                                which will be launched inside this container.
            @type  commID:      str
        """
        self._manager = manager
        self._commID = commID
        
        self._confDir = pjoin(manager.confDir, commID)
        self._dataDir = pjoin(manager.dataDir, commID)
        
        if os.path.isdir(self._confDir):
            raise ValueError('There is already a configuration directory for '
                             "'{0}'.".format(commID))
        
        if os.path.isdir(self._dataDir):
            raise ValueError('There is already a data directory for '
                             "'{0}'.".format(commID))
        
        os.mkdir(self._confDir)
        os.mkdir(self._dataDir)
        
        self._rceDir = pjoin(self._dataDir, 'rce')
        self._rosDir = pjoin(self._dataDir, 'ros')
        
        os.mkdir(self._rceDir)
        os.mkdir(self._rosDir)
        
        super(DeploymentContainer, self).__init__(manager.reactor,
                                                  manager.rootfs,
                                                  self._confDir)
    
    def _createFstabFile(self):
        """ Internally used method to create a fstab file based on the given
            parameters.
        """
        self.extendFstab(self._rosDir, 'home/ros', False)
        self.extendFstab(self._rceDir, 'opt/rce/data', False)
        self.extendFstab(self._manager.srcDir, 'opt/rce/src', True)
        self.extendFstab(pjoin(self._confDir, 'upstartComm'),
                         'etc/init/rceComm.conf', True)
        self.extendFstab(pjoin(self._confDir, 'upstartLauncher'),
                         'etc/init/rceLauncher.conf', True)
        
        for srcPath, destPath in self._manager.pkgDirIter:
            self.extendFstab(srcPath, destPath, True)
    
    def _createUpstartScripts(self):
        """ Internally used method to create an upstart script based on the
            given parameters.
        """
        with open(pjoin(self._confDir, 'upstartComm'), 'w') as f:
            f.write(template.UPSTART_COMM.format(
                commID=self._commID,
                serverID=self._manager.relayID
            ))
        
        with open(pjoin(self._confDir, 'upstartLauncher'), 'w') as f:
            f.write(template.UPSTART_LAUNCHER)
    
    def start(self, deferred):
        """ Start the deployment container.
            
            @param deferred:    Deferred whose callback is called with this
                                Container instance as argument when the start
                                was successful and whose errback is called with
                                an error message when there was a problem.
            @type  deferred:    twisted::Deferred
        """
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
        
        self._createFstabFile()
        self._createUpstartScripts()
        
        super(DeploymentContainer, self).start(self._commID, deferred)
    
    def stop(self, deferred):
        """ Stop the deployment container.
            
            @param deferred:    Deferred whose callback is called when the
                                stop was successful and whose errback is called
                                with an error message when there was a problem.
            @type  deferred:    twisted::Deferred
        """
        super(DeploymentContainer, self).stop(self._commID, deferred)
    
    def __del__(self):
        """ Destructor.
        """
        try:
            shutil.rmtree(self._confDir)
            shutil.rmtree(self._dataDir)
        except:
            pass


class CommandContainer(Container):
    """ Container representation which is used to execute a single command
        inside a container. Should be used for rosmake.
    """
    def __init__(self, reactor, rootfs, pkgDir):
        """ Initialize the command container.
            
            @param reactor:     Reference to the twisted::reactor
            @type  reactor:     twisted::reactor
            
            @param rootfs:      Filesystem path of the root directory of the
                                container filesystem.
            @type  rootfs:      str
            
            @param pkgDir:      List of tuples containing package source and
                                destination path for fstab file.
            @type  pkgDir:      [(str, str)]
        """
        self._confDir = mkdtemp(prefix='rce_cmd')
        self._homeDir = os.path.join(self._confDir, 'home')
        self._pkgDir = pkgDir
        
        os.mkdir(self._homeDir)
        
        super(CommandContainer, self).__init__(reactor, rootfs, self._confDir)
    
    def execute(self, command):
        """ Execute the command in the command container.
            
            @param command:     Command which should be executed.
            @type  command:     [str]
        """
        for srcPath, destPath in self._pkgDir:
            self.extendFstab(srcPath, destPath, False)
        
        self.extendFstab(self._homeDir, 'home/ros', False)
        
        super(CommandContainer, self).execute(os.path.basename(self._confDir),
                                              command)
    
    def __del__(self):
        """ Destructor.
        """
        try:
            shutil.rmtree(self._confDir)
        except:
            pass
