#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       NodeMonitor.py
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
from twisted.internet.protocol import ProcessProtocol

# Python specific imports
import os

# Custom imports
from Exceptions import InternalError, InvalidRequest
from ROSUtil import ResourceNotFound

class ROSNodeProtocol(ProcessProtocol):
    """ Protocol which is used to handle the ROS nodes.
    """
    def __init__(self, nodeMonitor):
        self._nodeMonitor = nodeMonitor
    
    def connectionMade(self):
        self._nodeMonitor.started(self)
    
    def processEnded(self, reason):
        self._nodeMonitor.stopped()

class NodeMonitor(object):
    """ Class which is used to launch and monitor a node.
    """
    _STOP_ESCALATION = [ ( 'INT',   15),
                         ('TERM',    2),
                         ('KILL', None) ]
    
    def __init__(self, manager, node):
        """ Initialize the NodeMonitor instance.
            
            @param manager:     Manager instance used in this node which
                                has references to the ROS component loader
                                and the twisted reactor.
            
            @param node:    Node instance which should be monitored.
            @type  node:    Node
        """
        self._manager = manager
        self._node = node
        
        self._process = None
        self._started = False
        self._running = False
        
        self._escalationLvl = 0
    
    @property
    def tag(self):
        """ Node tag to identify the node. """
        return self._node.tag
    
    def start(self):
        """ Launch the node.

            @raise:     InternalError if Node is already launched.
        """
        if self._started:
            raise InternalError('Can not launch an already running node.')
        
        # Find and validate executable
        try:
            cmd = [self._manager.loader.findNode(self._node.pkg, self._node.exe)]
        except ResourceNotFound as e:
            raise InvalidRequest('Could not identify which node to launch: {0}'.format(e))
        
        # Process namespace argument
        namespace = self._node.namespace
        
        if namespace:
            cmd.append('__ns:={0}'.format(namespace))
        
        # Start node
        self._started = True
        log.msg('Start Node {0}/{2} [pkg: {1}].'.format( self._node.namespace,
                                                         self._node.pkg,
                                                         self._node.exe ))
        self._manager.reactor.spawnProcess(ROSNodeProtocol(self), cmd[0], cmd, env=os.environ)
    
    def started(self, process):
        """ Callback for ROSProcessProtocol to signal that the process
            has been launched.
        """
        self._process = process
        self._running = True
        self._manager.registerNode(self)
    
    def isAlive(self):
        """ Check whether the node/process is alive.
        """
        if not self._started:
            return True
        
        return self._running
    
    def stopped(self):
        """ Callback for ROSProcessProtocol to signal that the process has died.
        """
        self._manager.unregisterNode(self)
        self._running = False
        self._process = None
    
    def stop(self):
        """ Stop the node.
        """
        if not self._running:
            return
        
        if self._escalationLvl == 0:
            log.msg('Stop Node {0}/{1}.'.format( self._node.namespace,
                                                 self._node.exe ))
        escalation = self._STOP_ESCALATION[self._escalationLvl]
        
        self._process.transport.signalProcess(escalation[0])
        
        if escalation[1]:
            self._escalationLvl += 1
            self._manager.reactor.callLater(escalation[1], self.stop)
