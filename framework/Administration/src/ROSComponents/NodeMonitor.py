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

# ROS specific imports
import roslib.packages      # TODO: Find replacement function for roslib.packages function

# twisted specific imports
from twisted.internet.protocol import ProcessProtocol

# Python specific imports
import os

# Custom imports
from Exceptions import InternalError
from ParameterMonitor import IntMonitor, StrMonitor, FloatMonitor, BoolMonitor, FileMonitor #@UnresolvedImport

class ROSNodeProtocol(ProcessProtocol):
    """ Protocol which is used to handle the ROS nodes.
    """
    def __init__(self, nodeMonitor):
        self._nodeMonitor = nodeMonitor
    
    def connectionMade(self):
        self._nodeMonitor.running = True
        self._nodeMonitor.process = self
    
    def processEnded(self, reason):
        self._nodeMonitor.process = None
        self._nodeMonitor.running = False
        self._deferred.callback(reason)

class NodeMonitor(object):
    """ Class which is used to launch and monitor a node.
    """
    _STOP_ESCALATION = [ ( 'INT',   15),
                         ('TERM',    2),
                         ('KILL', None) ]
    
    _PARAMS = { IntMonitor.IDENTIFIER   : IntMonitor,
                StrMonitor.IDENTIFIER   : StrMonitor,
                FloatMonitor.IDENTIFIER : FloatMonitor,
                BoolMonitor.IDENTIFIER  : BoolMonitor,
                FileMonitor.IDENTIFIER  : FileMonitor }
    
    def __init__(self, reactor, node):
        """ Initialize the NodeMonitor instance.
            
            @param reactor:     twisted reactor instance
            
            @param node:    Node instance which should be monitored.
            @type  node:    Node
        """
        self._reactor = reactor
        self._node = node
        self.process = None
        
        self.started = False
        self.running = False
        
        self._escalationLvl = 0
    
    def start(self):
        """ Add the necessary parameters to the parameter server, launch the
            node and start the necessary handlers for the interfaces.

            @raise:     InternalError if Node can not be launched.
        """
        if self.started:
            raise InternalError('Can not launch an already running node.')
        
        self.started = True
        self._paramMonitors = [self._PARAMS[param.IDENTIFIER](param.name, param.value) for param in self._node.params]
        
        cmd = [ roslib.packages.find_node(self._node.pkg, self._node.exe) ]
        self._reactor.spawnProcess(ROSNodeProtocol(self), cmd[0], cmd, env=os.environ)

    def isAlive(self):
        """ Check whether the node/process is alive.
        """
        if not self.started:
            return True
        
        return self.running

    def stop(self):
        """ Stop the node and remove the parameters from the parameter server.
        """
        if not self.running:
            return
        
        escalation = self._STOP_ESCALATION[self._escalationLvl]
        
        self.process.transport.signalProcess(escalation[0])
        
        if escalation[1]:
            self._escalationLvl += 1
            self._reactor.callLater(self.stop, escalation[1])
    
    def __del__(self):
        """ Destructor.
        """
        if self.running:
            self.stop()
