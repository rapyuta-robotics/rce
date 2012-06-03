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

# ROS specific imports
import roslaunch.core

# Custom imports
from Exceptions import InvalidRequest, InternalError
from Comm.Message.Base import Message
from Comm.Message import MsgTypes
from Type import ROSAddMessage, ROSRemoveMessage, ROSMsgMessage #, ROSResponseMessage, ROSGetMessage #@UnresolvedImport
from Processor import ROSAddProcessor, ROSRemoveProcessor, ROSMessageContainerProcessor #, ROSGetProcessor #@UnresolvedImport
from MiscUtility import generateID

from Node import NodeForwarder #@UnresolvedImport
from Interface import ServiceInterface, PublisherInterface, SubscriberInterface #@UnresolvedImport

class ROSManager(object):
    """ Manager which handles ROS specific tasks.
    """
    def __init__(self, launcher, commMngr):
        """ Initialize the ROSManager.
            
            @param launcher:    Launcher for the ROS nodes
            @type  launcher:    roslaunch.scriptapi.Launcher
            
            @param commMngr:    CommManager which should be used to communicate.
            @type  commMngr:    CommManager
        """
        # References used by the manager
        self._launcher = launcher
        self._commMngr = commMngr
        
        # Storage for references required to process messages
        self._runningNodes = {}
        self._interfaces = {}
        
        # Flag to indicate shutdown
        self.isShutdown = False
        
        rosAdd = ROSAddMessage()
        rosAdd.registerComponents([ NodeForwarder,
                                    ServiceInterface,
                                    PublisherInterface,
                                    SubscriberInterface ])
        
        # Register Content Serializers
        self._commMngr.registerContentSerializers([ rosAdd,
                                                    ROSRemoveMessage(),
                                                    ROSMsgMessage(),
                                                    # ROSResponseMessage(),
                                                    # ROSGetMessage()
                                                   ])
        
        # Register Message Processors
        self._commMngr.registerMessageProcessors([ ROSAddProcessor(self, commMngr),
                                                   ROSRemoveProcessor(self, commMngr),
                                                   ROSMessageContainerProcessor(self, commMngr),
                                                   # ROSGetProcessor(self, commMngr)
                                                  ])
    
    def addInterface(self, interface):
        """ Callback for Interface instance to register the interface.

            @param interface:   Interface which should be registered.
            @type  interface:   InterfaceBase

            @raise:     InternalError if there is already an interface
                        with the same name registered.
        """
        name = interface.interfaceName

        if name in self._interfaces:
            raise InternalError('There is already an interface with the same ID.')

        self._interfaces[name] = interface
    
    def getInterface(self, name):
        """ Get the interface matching the given name.
            
            @raise:     InternalError if there is no interface matching the given name.
        """
        if name not in self._interfaces:
            raise InternalError('There is no interface with the name "{0}".'.format(name))
        
        return self._interfaces[name]
    
    def removeInterface(self, interface):
        """ Callback for Interface instance to unregister the interface.
        """
        self._interfaces.pop(interface.interfaceName, None)
    
    def sendROSMessage(self, rosMsg, dest, name, uid):
        """ Send a ROS message to the specified destination.
                            
            @param rosMsg:  ROS Message which should be sent in serialized form.
            @type  rosMsg:  str
            
            @param dest:    Communication ID of destination.
            @type  dest:    str
            
            @param name:    Interface name which should be used to send the message
            @type  name:    str
            
            @param uid:     Unique ID to identify the message
            @type  uid:     str
        """
        msg = Message()
        msg.msgType = MsgTypes.ROS_MSG
        msg.dest = dest
        msg.content = { 'msg' : rosMsg, 'name' : name, 'uid' : uid, 'push' : False }
        self._commMngr.sendMessage(msg)
    
    def addNode(self, node):
        """ Add a new node to the list of managed nodes. This method takes
            care of initializing and starting the node and its necessary
            data.

            @param node:    Node which should be added.
            @type  node:    Node instance

            @return:    ID with which the added Node can be identified.
            @rtype:     str

            @raise:     InternalError if Node can not be added.
        """
        if self.isShutdown:
            return

        while 1:
            uid = generateID()

            if uid not in self._runningNodes.keys():
                self._runningNodes[uid] = node
                break

        try:
            node.start(self, uid)
        except InternalError:
            del self._runningNodes[uid]
            raise

        return uid

    def removeNode(self, uid):
        """ Remove and terminate a node matching the given ID and clean up
            after it. All messages from and to this node will be removed as
            well.

            @param uid:     Key which is used to identify the node which
                            should be removed.
            @type  uid:     str

            @raise:     InvalidRequest if the ID does not match any node.
        """
        if self.isShutdown:
            return

        if uid not in self._runningNodes:
            raise InvalidRequest('Node ID is invalid.')

        self._runningNodes[uid].stop()
        del self._runningNodes[uid]

    def startNodeProcess(self, node):
        """ Start the given node.
             - Callback method from Node
             - Has to be executed in main thread

            @param node:    Node which should be added to the list.
            @type  node:    roslaunch.core.Node

            @return:        Process instance
            @rtype:         roslaunch.Process instance

            @raise: InternalError if the node could not be launched.
        """
        try:
            return self._launcher.launch(node)
        except roslaunch.core.RLException as e:
            raise InternalError(str(e))

    def runTaskInSeparateThread(self, func, *args, **kw):
        """ Convenience method to run any function in a separate thread.

            This method should not be overwritten.

            @param func:    Callable which should be executed in separate
                            thread.
            @type  func:    callable

            @param *args:   Any positional arguments which will be passed
                            to 'func'.

            @param *kw:     Any keyworded arguments which will be passed
                            to 'func'.
        """
        self._commMngr.reactor.callInThread(func, args, kw)

    def shutdown(self):
        """ Method is called when the manager is stopped.
        """
        if self.isShutdown:
            return
        
        self.isShutdown = True
        
        for node in self._runningNodes:
            node.stop()
        
        self._commMngr.shutdown()
