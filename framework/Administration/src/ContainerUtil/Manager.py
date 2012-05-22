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

# Custom imports
from Comm.Message.ContainerType import StartContainerMessage, StopContainerMessage, ContainerStatusMessage #@UnresolvedImport
from Comm.Message.ContainerProcessor import StartContainerProcessor, StopContainerProcessor #@UnresolvedImport
import settings

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
        # References used by the container manager
        self._commMngr = commMngr
        self._reactor = commMngr.reactor
        
        # Register Content Serializers
        self.registerContentSerializer(StartContainerMessage())
        self.registerContentSerializer(StopContainerMessage())
        self.registerContentSerializer(ContainerStatusMessage())
        
        # Register Message Processors
        self._commMngr.registerMessageProcessor(StartContainerProcessor(self))
        self._commMngr.registerMessageProcessor(StopContainerProcessor(self))
    
    def _createConfigFile(self, commID, ip):
        """ Create a config file based on the given parameters.
        """
        with open(os.path.abspath(os.path.join(settings.CONF_DIR, commID, 'config')), 'w') as f:
            f.writeline('lxc.utsname = ros')
            f.writeline('')
            f.writeline('lxc.tty = 4')
            f.writeline('lxc.pts = 1024')
            f.writeline('lxc.rootfs = {rootfs}'.format(rootfs=settings.ROOTFS))
            f.writeline('lxc.mount  = {fstab}'.format(fstab=os.path.abspath(os.path.join(settings.CONF_DIR, commID, 'fstab'))))
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
        with open(os.path.abspath(os.path.join(settings.CONF_DIR, commID, 'fstab')), 'w') as f:
            f.writeline('proc     {rootfs}/proc      proc     nodev,noexec,nosuid 0 0'.format(rootfs=settings.ROOTFS))
            f.writeline('devpts   {rootfs}/dev/pts   devpts   defaults            0 0'.format(rootfs=settings.ROOTFS))
            f.writeline('sysfs    {rootfs}/sys       sysfs    defaults            0 0'.format(rootfs=settings.ROOTFS))
            f.writeline('{homeDir}   {rootfs}/home/ros   none   bind 0 0'.format(homeDir=os.path.abspath(homeDir), rootfs=settings.ROOTFS))
    
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
        
        # Start container
        deferred = Deferred()
        
        def callback(reason):
            if reason.value.exitCode != 0:
                log.msg(reason)
        
        deferred.addCallback(callback)
        
        cmd = ['/usr/bin/lxc-start', '-n', commID, '-f', os.path.abspath(os.path.join(settings.CONF_DIR, commID, 'config'))]
        self._reactor.spawnProcess(LXCProtocol(deferred), cmd[0], cmd, env=os.environ)
    
    def startContainer(self, commID, ip, homeDir, key):
        """
        """
        self._reactor.deferToThread(self._startContainer, commID, ip, homeDir, key)
    
    def _stopContainer(self, commID):
        """
        """
        # Stop container
        deferred = Deferred()
        
        def callback(reason):
            if reason.value.exitCode != 0:
                log.msg(reason)
        
        deferred.addCallback(callback)
        
        cmd = ['/usr/bin/lxc-stop', '-n', commID]
        self._reactor.spawnProcess(LXCProtocol(deferred), cmd[0], cmd, env=os.environ)
    
    def stopContainer(self, commID):
        """
        """
        self._reactor.deferToThread(self._stopContainer, commID)
