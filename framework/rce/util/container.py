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

pjoin = os.path.join

# twisted specific imports
from twisted.python import log
from twisted.python.failure import Failure
from twisted.internet.defer import Deferred
from twisted.internet.utils import getProcessValue

# Custom imports
from rce.util.path import checkPath


_CONFIG = """
lxc.utsname = {hostname}

lxc.tty = 4
lxc.pts = 1024
lxc.rootfs = {fs}
lxc.mount = {fstab}

lxc.network.type = veth
lxc.network.flags = up
lxc.network.name = eth0
lxc.network.link = lxcbr0
lxc.network.ipv4 = {ip}

lxc.cgroup.devices.deny = a
# /dev/null and zero
lxc.cgroup.devices.allow = c 1:3 rwm
lxc.cgroup.devices.allow = c 1:5 rwm
# consoles
lxc.cgroup.devices.allow = c 5:1 rwm
lxc.cgroup.devices.allow = c 5:0 rwm
lxc.cgroup.devices.allow = c 4:0 rwm
lxc.cgroup.devices.allow = c 4:1 rwm
# /dev/{{,u}}random
lxc.cgroup.devices.allow = c 1:9 rwm
lxc.cgroup.devices.allow = c 1:8 rwm
lxc.cgroup.devices.allow = c 136:* rwm
lxc.cgroup.devices.allow = c 5:2 rwm
# rtc
lxc.cgroup.devices.allow = c 254:0 rwm
"""


_FSTAB_BASE = """
proc    {proc}    proc    nodev,noexec,nosuid    0 0
devpts    {devpts}    devpts    defaults    0 0
sysfs    {sysfs}    sysfs    defaults    0 0
"""


_FSTAB_BIND = '{srcDir}    {fsDir}    none    bind{ro}    0 0\n'


class ContainerError(Exception):
    """ Exception is raised when a LXC command fails.
    """


class Container(object):
    """ Class representing a single container.
    """
    def __init__(self, reactor, rootfs, conf, hostname, ip):
        """ Initialize the Container.
            
            @param reactor:     Reference to the twisted::reactor
            @type  reactor:     twisted::reactor
            
            @param rootfs:      Filesystem path of the root directory of the
                                container filesystem.
            @type  rootfs:      str
            
            @param conf:        Filesystem path of folder where configuration
                                files for the container should be stored.
            @type  conf:        str
            
            @param hostname:    Host name of the container.
            @type  hostname:    str
            
            @param ip:          IP address which the container should use.
                                Use '0.0.0.0' for DHCP.
            @type  ip:          str
        """
        self._reactor = reactor
        self._rootfs = rootfs
        self._conf = pjoin(conf, 'config')
        self._fstab = pjoin(conf, 'fstab')
        self._hostname = hostname
        self._ip = ip
        
        checkPath(conf, 'Container Configuration')
        
        if os.path.exists(self._conf):
            raise ValueError('There is already a config file in the given '
                             'configuration folder.')
        
        if os.path.exists(self._fstab):
            raise ValueError('There is already a fstab file in the given '
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
        self._fstabExt.append(_FSTAB_BIND.format(srcDir=src,
                                                 fsDir=pjoin(self._rootfs, fs),
                                                 ro=',ro' if ro else ''))
    
    def _setup(self):
        """ Setup necessary files.
        """
        with open(self._conf, 'w') as f:
            f.write(_CONFIG.format(hostname=self._hostname, fs=self._rootfs,
                                   fstab=self._fstab, ip=self._ip))
        
        with open(self._fstab, 'w') as f:
            f.write(_FSTAB_BASE.format(
                proc=pjoin(self._rootfs, 'proc'),
                devpts=pjoin(self._rootfs, 'dev/pts'),
                sysfs=pjoin(self._rootfs, 'sys')))
            f.writelines(self._fstabExt)
    
    def start(self, name):
        """ Start the container.
            
            @param name:    Name of the container which should be started.
            @type  name:    str
            
            @return:        Deferred whose callback is triggered on success or
                            whose errback is triggered on failure with an
                            error message.
            @rtype:         twisted::Deferred
        """
        self._setup()
        
        log.msg("Start container '{0}'".format(name))
        deferred = Deferred()
        
        try:
            dfrd = getProcessValue('/usr/bin/lxc-start',
                                   ('-n', name, '-f', self._conf, '-d'),
                                   env=os.environ, reactor=self._reactor)
            
            def cb(retVal):
                if retVal == 0:
                    deferred.callback('Container successfully started.')
                else:
                    e = ContainerError('Container could not be started: '
                                       'Received exit code {0} from '
                                       'lxc-start.'.format(retVal))
                    deferred.errback(Failure(e))
            
            dfrd.addCallback(cb)
        except OSError:
            e = ContainerError('Insufficient system resources to start a new '
                               'process.')
            deferred.errback(Failure(e))
        
        return deferred
    
    def stop(self, name):
        """ Stop the container.
            
            @param name:        Name of the container which should be stopped.
            @type  name:        str
            
            @param command:     Deferred whose callback is triggered on success
                                or whose errback is triggered on failure with
                                an error message.
            @type  command:     twisted::Deferred
        """
        log.msg("Stop container '{0}'".format(name))
        deferred = Deferred()
        
        try:
            dfrd = getProcessValue('/usr/bin/lxc-stop', ('-n', name),
                                   env=os.environ, reactor=self._reactor)
            
            def cb(retVal):
                if retVal == 0:
                    deferred.callback('Container successfully stopped.')
                else:
                    e = ContainerError('Container could not be stopped: '
                                       'Received exit code {0} from '
                                       'lxc-stop.'.format(retVal))
                    deferred.errback(Failure(e))
            
            dfrd.addCallback(cb)
        except OSError:
            e = ContainerError('Insufficient system resources to stop a '
                               'process.')
            deferred.errback(Failure(e))
        
        return deferred
    
#    def execute(self, name, command):
#        """ Execute a command inside the container.
#            
#            @param name:        Name of the container which will execute the
#                                command.
#            @type  name:        str
#            
#            @param command:     Command which should be executed.
#            @type  command:     [str]
#        """
#        def cb(_):
#            print('\nSuccessful.')
#            self._reactor.stop()
#        
#        def eb(err):
#            print('\n{0}'.format(err.getErrorMessage()))
#            self._reactor.stop()
#        
#        deferred = Deferred()
#        deferred.addCallbacks(cb, eb)
#        
#        self.extendFstab('/usr/lib/lxc', pjoin(self._rootfs, 'usr/lib/lxc'),
#                         False)
#        
#        protocol = self._setup(deferred, sys.stdout.write, sys.stderr.write)
#        
#        try:
#            cmd = ['/usr/bin/lxc-execute', '-n', name, '-f', self._conf, '--']
#            cmd += command
#            self._reactor.spawnProcess(protocol, cmd[0], cmd, env=os.environ)
#        except Exception as e:
#            import traceback
#            print('Caught an exception when trying to execute a command in '
#                  'the container.')
#            print('\n'.join(traceback.format_exception_only(type(e), e)))
