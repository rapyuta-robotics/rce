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

# twisted specific imports
from twisted.python import log
from twisted.internet.defer import Deferred
from twisted.internet.protocol import ProcessProtocol

# Custom imports
from util import template

pjoin = os.path.join


def checkPath(path, description):
    """ Check if the path is valid.
        
        @param path:            Path which should be checked.
        @type  path:            str
        
        @param description:     Description which is used for the error
                                message if necessary.
        @type  description:     str
        
        @raise:                 ValueError, if path is not valid.
    """
    if not os.path.isabs(path):
        raise ValueError('{0} directory is not an absolute '
                         'path.'.format(description))
    
    if not os.path.isdir(path):
        raise ValueError('{0} directory does not exist: '
                         '{1}'.format(description, path))


def checkExe(folder, exe):
    """ Check if the executable is valid.
        
        @param folder:          Folder in which the executable is located.
        @type  folder:          str
        
        @param exe:             Executable which should be checked.
        @type  exe:             str
        
        @raise:                 ValueError, if executable is not valid.
    """
    path = pjoin(folder, exe)
    
    if not os.path.isfile(path):
        raise ValueError("'{0}' is not a file.".format(exe))
    
    if not os.access(path, os.X_OK):
        raise ValueError("'{0}' is not a executable.".format(exe))


class _LXCProtocol(ProcessProtocol):
    """ Protocol which is used to handle the LXC commands.
    """
    def __init__(self, deferred):
        self._deferred = deferred
    
    def processEnded(self, reason):
        self._deferred.callback(reason)


class Container(object):
    """ Class representing a single container.
    """
    def __init__(self, reactor, rootfs, conf):
        """ # TODO: Add description
        """
        self._rootfs = rootfs
        self._conf = os.path.join(conf, 'config')
        self._fstab = os.path.join(conf, 'fstab')
        
        checkPath(conf, 'Container Configuration')
        
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
    
    def _setup(self, deferred):
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
        
        def callback(reason):
            if reason.value.exitCode != 0:
                deferred.errback('Received non zero exit code from LXC: '
                                 '{0}'.format(reason.getErrorMessage()))
            else:
                deferred.callback(self)
        
        _deferred = Deferred()
        _deferred.addCallback(callback)
        
        return _LXCProtocol(_deferred)
    
    def start(self, name, deferred):
        """ Start the container.
            
            # TODO: Add description
        """
        protocol = self._setup(deferred)
        
        try:
            log.msg("Start container '{0}'".format(name))
            cmd = ['/usr/bin/lxc-start', '-n', name, '-f', self._conf, '-d']
            self.reactor.spawnProcess(protocol, cmd[0], cmd, env=os.environ)
        except:
            log.msg('Caught an exception when trying to start the container.')
            import sys, traceback
            etype, value, _ = sys.exc_info()
            deferred.errback('\n'.join(traceback.format_exception_only(etype,
                                                                       value)))
    
    def stop(self, name, deferred):
        """ Stop the container.
            
            # TODO: Add description
        """
        def callback(reason):
            if reason.value.exitCode == 0:
                deferred.callback(None)
            else:
                msg = ('Received non zero exit code after stop of '
                       'container: {0}'.format(reason.getErrorMessage()))
                deferred.errback(msg)
        
        _deferred = Deferred()
        _deferred.addCallback(callback)
        
        try:
            cmd = ['/usr/bin/lxc-stop', '-n', name]
            self.reactor.spawnProcess(_LXCProtocol(_deferred), cmd[0], cmd,
                                      env=os.environ)
        except:
            import sys, traceback
            etype, value, _ = sys.exc_info()
            msg = '\n'.join(traceback.format_exception_only(etype, value))
            log.msg(msg)
            deferred.errback(msg)
    
    def execute(self, name, command):
        """ Execute a command inside the container.
            
            # TODO: Add description
        """
        def cb(msg):
            print(msg)
        
        deferred = Deferred()
        deferred.addErrback(cb)
        
        protocol = self._setup(deferred)
        
        try:
            cmd = ['/usr/bin/lxc-execute', '-n', name, '-f', self._conf,
                   command]
            self.reactor.spawnProcess(protocol, cmd[0], cmd, env=os.environ)
        except:
            log.msg('Caught an exception when trying to execute a command in '
                    'the container.')
            import sys, traceback
            etype, value, _ = sys.exc_info()
            deferred.errback('\n'.join(traceback.format_exception_only(etype,
                                                                       value)))
