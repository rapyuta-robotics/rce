#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Manager.py
#       
#       Copyright 2012 dominique hunziker <dominique.hunziker@gmail.com>
#       
#       This program is free software; you can redistribute it and/or modify
#       it under the terms of the GNU General Public License as published by
#       the Free Software Foundation; either version 2 of the License, or
#       (at your option) any later version.
#       
#       This program is distributed in the hope that it will be useful,
#       but WITHOUT ANY WARRANTY; without even the implied warranty of
#       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#       GNU General Public License for more details.
#       
#       You should have received a copy of the GNU General Public License
#       along with this program; if not, write to the Free Software
#       Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#       MA 02110-1301, USA.
#       
#       

# twisted specific imports
from twisted.python import log
from twisted.internet.defer import Deferred, DeferredList
from twisted.internet.protocol import ProcessProtocol

# Python specific imports
import os
import shutil
from threading import Event

# Custom imports
import settings
from Comm.Message import MsgDef
from Type import StartContainerMessage, StopContainerMessage, ContainerStatusMessage
from Processor import StartContainerProcessor, StopContainerProcessor

class LXCProtocol(ProcessProtocol):
    """ Protocol which is used to handle the LXC commands.
    """
    def __init__(self, deferred):
        self._deferred = deferred
    
    def processEnded(self, reason):
        self._deferred.callback(reason)

class ContainerManager(object):
    """ Manager which handles container specific task.
    """
    def __init__(self, commMngr):
        """ Initialize the ContainerManager.
            
            @param commMngr:    CommManager which should be used to communicate.
            @type  commMngr:    CommManager
        """
        # References used by the manager
        self._commMngr = commMngr
        self._reactor = commMngr.reactor
        
        # Validate loaded directories from settings
        self._confDir = settings.CONF_DIR
        self._rootfs = settings.ROOTFS
        self._srcRoot = settings.ROOT_DIR
        
        if not os.path.isabs(self._confDir):
            raise ValueError('Configuration directory is not an absolute path.')
        
        if not os.path.isabs(self._rootfs):
            raise ValueError('Root file system directory is not an absolute path.')
        
        if not os.path.isabs(self._srcRoot):
            raise ValueError('Root source directory is not an absolute path.')
        
        # Storage of all CommIDs
        self._commIDs = []
        
        # Register Content Serializers
        self._commMngr.registerContentSerializers([ StartContainerMessage(),
                                                    StopContainerMessage(),
                                                    ContainerStatusMessage() ])
        
        # Register Message Processors
        self._commMngr.registerMessageProcessors([ StartContainerProcessor(self),
                                                   StopContainerProcessor(self) ])
    
    def _createConfigFile(self, commID):
        """ Create a config file based on the given parameters.
        """
        content = '\n'.join([ 'lxc.utsname = ros',
                              '',
                              'lxc.tty = 4',
                              'lxc.pts = 1024',
                              'lxc.rootfs = {rootfs}'.format(rootfs=self._rootfs),
                              'lxc.mount = {fstab}'.format(
                                  fstab=os.path.join(self._confDir, commID, 'fstab')
                              ),
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
        
        with open(os.path.join(self._confDir, commID, 'config'), 'w') as f:
            f.write(content)
    
    def _createFstabFile(self, commID, homeDir):
        """ Create a fstab file based on the given parameters.
        """
        if not os.path.isabs(homeDir):
            raise ValueError('Home directory is not an absolute path.')
        
        content = '\n'.join([ 'proc     {proc}      proc     nodev,noexec,nosuid 0 0'.format(
                                  proc=os.path.join(self._rootfs, 'proc')
                              ),
                              'devpts   {devpts}   devpts   defaults            0 0'.format(
                                  devpts=os.path.join(self._rootfs, 'dev/pts')
                              ),
                              'sysfs    {sysfs}       sysfs    defaults            0 0'.format(
                                  sysfs=os.path.join(self._rootfs, 'sys')
                              ),
                              '/opt/ros    {rootfsROS}    none    bind,ro 0 0'.format(
                                  rootfsROS=os.path.join(self._rootfs, 'opt/ros')
                              ),
                              '{pyPath}    {rootfs}{pyPath}    none    bind 0 0'.format(
                                  pyPath='/usr/lib/pymodules/python2.7/rospkg',
                                  rootfs=self._rootfs
                              ),
                              '{homeDir}    {rootfsHome}    none    bind 0 0'.format(
                                  homeDir=homeDir,
                                  rootfsHome=os.path.join(self._rootfs, 'home/ros')
                              ),
                              '{srcDir}    {rootfsLib}    none    bind,ro 0 0'.format(
                                  srcDir=self._srcRoot,
                                  rootfsLib=os.path.join(self._rootfs, 'opt/rce')
                              ),
                              '{upstart}    {initDir}    none    bind,ro 0 0'.format(
                                  upstart=os.path.join(self._confDir, commID, 'upstartComm'),
                                  initDir=os.path.join(self._rootfs, 'etc/init/rceComm.conf')
                              ),
                              '{upstart}   {initDir}   none   bind,ro 0 0'.format(
                                  upstart=os.path.join(self._confDir, commID, 'upstartLauncher'),
                                  initDir=os.path.join(self._rootfs, 'etc/init/rceLauncher.conf')
                              ),
                              '' ])
        
        with open(os.path.join(self._confDir, commID, 'fstab'), 'w') as f:
            f.write(content)
    
    def _createUpstartScripts(self, commID):
        """ Create an upstart script based on the given parameters.
        """
        content = '\n'.join([ '# description',
                              'author "Dominique Hunziker"',
                              'description "CommNode of reCloudEngine - Framework for managing and using ROS nodes"',
                              '',
                              '# start/stop conditions',
                              'start on runlevel [2345]',
                              'stop on runlevel [016]',
                              '',
                              'kill timeout 5',
                              '',
                              'script',
                              '\t# setup environment',
                              '\t. /etc/environment',
                              '\t. /opt/ros/fuerte/setup.sh',
                              '\t',
                              '\t# start environment node',
                              '\t'+' '.join([ 'start-stop-daemon',
                                              '--start',
                                              '-c', 'rce:rce',
                                              #'-d', '/opt/rce/framework/Administration/src',
                                              '--retry', '5',
                                              '--exec', '/usr/bin/python',
                                              '--',
                                              '/opt/rce/framework/Administration/src/Environment.py',
                                              commID,
                                              '{0}{1}'.format( MsgDef.PREFIX_SATELLITE_ADDR,
                                                               self._commMngr.commID[MsgDef.PREFIX_LENGTH_ADDR:])]),
                              'end script',
                              '' ])
       
        with open(os.path.join(self._confDir, commID, 'upstartComm'), 'w') as f:
            f.write(content)
        
        content = '\n'.join([ '# description',
                              'author "Dominique Hunziker"',
                              'description "Launcher of reCloudEngine - Framework for managing and using ROS nodes"',
                              '',
                              '# start/stop conditions',
                              'start on runlevel [2345]',
                              'stop on runlevel [016]',
                              '',
                              '# timeout before the process is killed; generous as a lot of processes have',
                              '# to be terminated by the launcher.',
                              'kill timeout 30',
                              '',
                              'script',
                              '\t# setup environment',
                              '\t. /etc/environment',
                              '\t. /opt/ros/fuerte/setup.sh',
                              '\t',
                              '\t# start launcher',
                              '\t'+' '.join([ 'start-stop-daemon',
                                              '--start',
                                              '-c', 'ros:ros',
                                              '-d', '/home/ros',
                                              '--retry', '5',
                                              '--exec', '/usr/bin/python',
                                              '--',
                                              '/opt/rce/framework/Administration/src/Launcher.py' ]),
                              'end script',
                              '' ])
       
        with open(os.path.join(self._confDir, commID, 'upstartLauncher'), 'w') as f:
            f.write(content)
    
    def _startContainer(self, deferred, commID, homeDir):
        """ Internally used method to start a container.
        """
        # Create folder for config and fstab file
        confDir = os.path.join(self._confDir, commID)
        
        if os.path.isdir(confDir):
            msg = 'There exists already a directory with the name "{0}".'.format(commID)
            log.msg(msg)
            deferred.errback(msg)
            return
        
        try:
            log.msg('Create files...')
            os.mkdir(confDir)
            
            # Construct config file
            self._createConfigFile(commID)
            
            # Construct fstab file
            self._createFstabFile(commID, homeDir)
            
            # Construct startup scripts
            self._createUpstartScripts(commID)
        except:
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
                log.msg(reason)
                deferred.callback(None)
            else:
                deferred.errback(reason.getErrorMessage())
        
        _deferred.addCallback(callback)
        
        try:
            log.msg('Start container...')
            cmd = [ '/usr/bin/lxc-start',
                    '-n', commID,
                    '-f', os.path.join(self._confDir, commID, 'config'),
                    '-d' ]
            self._reactor.spawnProcess(LXCProtocol(_deferred), cmd[0], cmd, env=os.environ)
        except:
            import sys, traceback #@Reimport
            etype, value, _ = sys.exc_info()
            msg = '\n'.join(traceback.format_exception_only(etype, value))
            log.msg(msg)
            deferred.errback(msg)
            return
    
    def startContainer(self, commID, homeDir):
        """ Callback for message processor to stop a container.
        """
        deferred = Deferred()
        # TODO: Add callback to deferred to get information about the creation of the container
        
        if commID in self._commIDs:
            log.msg('There is already a container registered under the same CommID.')
            deferred.errback('There is already a container registered under the same CommID.')
            return
        
        self._commIDs.append(commID)
        self._reactor.callInThread(self._startContainer, deferred, commID, homeDir)
    
    def _stopContainer(self, deferred, commID):
        """ Internally used method to stop a container.
        """
        # Stop container
        _deferred = Deferred()
        
        def callback(reason):
            error = None
            
            if reason.value.exitCode != 0:
                log.msg(reason)
                error = reason.getErrorMessage()
            
            try:
                # Delete config folder
                shutil.rmtree(os.path.join(self._confDir, commID))
                
                # Remove commID from internal list
                self._reactor.callFromThread(self._commIDs.remove, commID)
            except:
                import sys, traceback
                etype, value, _ = sys.exc_info()
                error = '\n'.join(traceback.format_exception_only(etype, value))
            
            if error:
                deferred.errback(error)
            else:
                deferred.callback(None)
        
        _deferred.addCallback(callback)
        
        try:
            cmd = ['/usr/bin/lxc-stop', '-n', commID]
            self._reactor.spawnProcess(LXCProtocol(_deferred), cmd[0], cmd, env=os.environ)
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
        # TODO: Add callback to deferred to get information about the destuction of the container
        
        if commID not in self._commIDs:
            log.msg('There is no container registered under this CommID.')
            deferred.errback('There is no container registered under this CommID.')
            return
        
        self._reactor.callInThread(self._stopContainer, deferred, commID)
    
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
