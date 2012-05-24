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
from twisted.internet.defer import Deferred
from twisted.internet.protocol import ProcessProtocol

# Python specific imports
import os
import shutil

# Custom imports
import settings
from Comm.Message import MsgDef
from Type import StartContainerMessage, StopContainerMessage, ContainerStatusMessage #@UnresolvedImport
from Processor import StartContainerProcessor, StopContainerProcessor #@UnresolvedImport

class LXCProtocol(ProcessProtocol):
    """ Protocol which is used to handle the LXC commands.
    """
    def __init__(self, deferred):
        self._deferred
    
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
    
    def _createConfigFile(self, commID, ip):
        """ Create a config file based on the given parameters.
        """
        with open(os.path.join(self._confDir, commID, 'config'), 'w') as f:
            f.writeline('lxc.utsname = ros')
            f.writeline('')
            f.writeline('lxc.tty = 4')
            f.writeline('lxc.pts = 1024')
            f.writeline(
                'lxc.rootfs = {rootfs}'.format(
                    rootfs=os.path.join(self._confDir, commID, 'fstab')
                )
            )
            f.writeline('')
            f.writeline('lxc.network.type = veth')
            f.writeline('lxc.network.flags = up')
            f.writeline('lxc.network.name = eth0')
            f.writeline('lxc.network.link = br0')
            f.writeline('lxc.network.ipv4 = {ip}'.format(ip=ip))
            f.writeline('')
            f.writeline('lxc.cgroup.devices.deny = a')
            f.writeline('# /dev/null and zero')
            f.writeline('lxc.cgroup.devices.allow = c 1:3 rwm')
            f.writeline('lxc.cgroup.devices.allow = c 1:5 rwm')
            f.writeline('# consoles')
            f.writeline('lxc.cgroup.devices.allow = c 5:1 rwm')
            f.writeline('lxc.cgroup.devices.allow = c 5:0 rwm')
            f.writeline('lxc.cgroup.devices.allow = c 4:0 rwm')
            f.writeline('lxc.cgroup.devices.allow = c 4:1 rwm')
            f.writeline('# /dev/{,u}random')
            f.writeline('lxc.cgroup.devices.allow = c 1:9 rwm')
            f.writeline('lxc.cgroup.devices.allow = c 1:8 rwm')
            f.writeline('lxc.cgroup.devices.allow = c 136:* rwm')
            f.writeline('lxc.cgroup.devices.allow = c 5:2 rwm')
            f.writeline('# rtc')
            f.writeline('lxc.cgroup.devices.allow = c 254:0 rwm')
    
    def _createFstabFile(self, commID, homeDir):
        """ Create a fstab file based on the given parameters.
        """
        if not os.path.isabs(homeDir):
            raise ValueError('Home directory is not an absoulte path.')
        
        with open(os.path.join(self._confDir, commID, 'fstab'), 'w') as f:
            f.writeline(
                'proc     {proc}      proc     nodev,noexec,nosuid 0 0'.format(
                    proc=os.path.join(self._rootfs, 'proc')
                )
            )
            f.writeline(
                'devpts   {devpts}   devpts   defaults            0 0'.format(
                    devpts=os.path.join(self._rootfs, 'dev/pts')
                )
            )
            f.writeline(
                'sysfs    {sysfs}       sysfs    defaults            0 0'.format(
                    sysfs=os.path.join(self._rootfs, 'sys')
                )
            )
            f.writeline(
                '{homeDir}   {rootfsHome}   none   bind 0 0'.format(
                    homeDir=homeDir,
                    rootfsHome=os.path.join(self._rootfs, 'home/ros')
                )
            )
            f.writeline(
                '{srcDir}   {rootfsLib}   none   bind,ro 0 0'.format(
                    srcDir=self._srcRoot,
                    rootfsLib=os.path.join(self._rootfs, 'home/ros/lib')
                )
            )
            f.writeline(
                '{upstart}   {initDir}   none   bind,ro 0 0'.format(
                    upstart=os.path.join(self._confDir, commID, 'upstart'),
                    initDir=os.path.join(self._rootfs, 'etc/init/reappengine.conf')
                )
            )
    
    def _createUpstartScript(self, commID, ip, key):
        """ Create an upstart script based on the given parameters.
        """
        with open(os.path.join(self._confDir, commID, 'upstart'), 'w') as f:
            f.writeline('# description')
            f.writeline('author "Dominique Hunziker"')
            f.writeline('description "reappengine - ROS framework for managing and using ROS nodes"')
            f.writeline('')
            f.writeline('# start/stop conditions')
            f.writeline('start on runlevel [2345]')
            f.writeline('stop on runlevel [016])')
            f.writeline('')
            f.writeline('# timeout before the process is killed; generous as a lot of processes have')
            f.writeline('# to be terminated by the reappengine')
            f.writeline('kill timeout 30')
            f.writeline('')
            f.writeline('script')
            f.writeline('\t# setup environment')
            f.writeline('\t. /etc/environment')
            f.writeline('\t. /opt/ros/fuerte/setup.sh')
            f.writeline('\t')
            f.writeline('\t# start environment node')
            f.writeline('\t'+' '.join([ 'start-stop-daemon',
                                        '-c', 'ros:ros',
                                        '-d', '/home/ros',
                                        '--retry', '5',
                                        '--exec', 'python',
                                        '--',
                                        '/home/ros/lib/framework/Administration/src/Environment.py',
                                        commID,
                                        ip,
                                        '{0}{1}'.format( MsgDef.PREFIX_SATELLITE_ADDR,
                                                         self._commMngr.commID[MsgDef.PREFIX_LENGTH_ADDR:]),
                                       key ]))
            f.writeline('end script')
    
    def _startContainer(self, commID, ip, homeDir, key):
        """ Internally used method to start a container.
        """
        # Create folder for config and fstab file
        confDir = os.path.abspath(os.path.join(settings.CONF_DIR, commID))
        
        if os.path.isdir(confDir):
            log.msg('There exists already a directory with the name "{0}".'.format(commID))
            return
        
        os.mkdir(confDir)
        
        # Construct config file
        self._createConfigFile(commID, ip)
        
        # Construct fstab file
        self._createFstabFile(commID, homeDir)
        
        # Construct startup script
        self._createUpstartScript(commID, ip, key)
        
        # Start container
        deferred = Deferred()
        
        def callback(reason):
            if reason.value.exitCode != 0:
                log.msg(reason)
        
        deferred.addCallback(callback)
        
        cmd = [ '/usr/bin/lxc-start',
                '-n', commID,
                '-f', os.path.abspath(os.path.join(settings.CONF_DIR, commID, 'config')) ]
        self._reactor.spawnProcess(LXCProtocol(deferred), cmd[0], cmd, env=os.environ)
    
    def startContainer(self, commID, ip, homeDir, key):
        """
        """
        if commID in self._commIDs:
            log.msg('There is already a container registered under the same CommID.')
            return
        
        self._commIDs.append(commID)
        self._reactor.deferToThread(self._startContainer, commID, ip, homeDir, key)
    
    def _stopContainer(self, commID):
        """ Internally used method to stop a container.
        """
        # Stop container
        deferred = Deferred()
        
        def callback(reason):
            if reason.value.exitCode != 0:
                log.msg(reason)
        
        deferred.addCallback(callback)
        
        cmd = ['/usr/bin/lxc-stop', '-n', commID]
        self._reactor.spawnProcess(LXCProtocol(deferred), cmd[0], cmd, env=os.environ)
        
        # Delete config folder
        shutil.rmtree(os.path.join(self._confDir, commID))
    
    def stopContainer(self, commID):
        """
        """
        if commID not in self._commIDs:
            log.msg('There is no container registered under this CommID.')
            return
        
        self._reactor.deferToThread(self._stopContainer, commID)
        self._commIDs.remove(commID)
