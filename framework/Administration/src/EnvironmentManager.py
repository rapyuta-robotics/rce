#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       EnvironmentManager.py
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
import rospy
import roslaunch.scriptapi
import roslaunch.core

# twisted specific imports
from twisted.python import log
from twisted.internet.ssl import ClientContextFactory
from twisted.internet.task import LoopingCall

# Python specific imports
import sys

# Custom imports
from Exceptions import InvalidRequest, InternalError
from Comm.Message.Base import Message, validateAddress
from Comm.Message.TypeBase import MessageTypes as MsgTypes
from Comm.Message.ROSProcessor import ROSAddNode, ROSRemoveNode, ROSMessageContainer, ROSGet
from Comm.Manager import ReappengineManager
from Comm.Factory import ReappengineClientFactory
from MiscUtility import generateID

class EnvironmentFactory(ReappengineClientFactory):
    """ Specialized ReappengineClientFactory to filter the incoming messages.
    """
    def filterMessage(self, msgType):
        return msgType not in [ MsgTypes.ROUTE_INFO,
                                MsgTypes.ROS_ADD,
                                MsgTypes.ROS_REMOVE,
                                MsgTypes.ROS_GET,
                                MsgTypes.ROS_MSG ]

class EnvironmentManager(ReappengineManager):
    """ Manager which is used for the nodes inside the containers.
    """
    def __init__(self, reactor, launcher, commID):
        """ Initialize the necessary variables for the environment manager.

            @param reactor:     TODO: Add description
            @type  reactor:     reactor
            
            @param launcher:    Launcher for the ROS nodes
            @type  launcher:    roslaunch.scriptapi.Launcher

            @param commID:  CommID of this node which can be used by other
                            nodes to identify this node.
            @type  commID:  str
        """
        super(EnvironmentManager, self).__init__(reactor, commID)
        
        # References used by the manager
        self._launcher = launcher
        
        # Storage for references required to process messages
        self._runningNodes = {}
        self._interfaces = {}
        
        # Setup list of valid message processors
        self.msgProcessors.extend([ ROSAddNode(self),
                                    ROSRemoveNode(self),
                                    ROSMessageContainer(self),
                                    ROSGet(self) ])

    def registerInterface(self, interface):
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
    
    def unrgisterInterface(self, interface):
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
        self.sendMessage(msg)
    
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

        while True:
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

    def shutdown(self):
        """ Method is called when the manager/factory is stopped.
        """
        super(EnvironmentManager, self).shutdown()
        
        for node in self._runningNodes:
            node.stop()

def main(reactor, ip, port, commID, satelliteID, key):
    # Start logger
    log.startLogging(sys.stdout)
    
    log.msg('Start initialization...')

    # Init ROS
    log.msg('Initialize ROS node')
    rospy.init_node('Administration')

    # Init ROS Launcher
    log.msg('Initialize ROS launcher')
    launcher = roslaunch.scriptapi.ROSLaunch()

    # Create Manager
    manager = EnvironmentManager(reactor, launcher, commID)

    # Initialize twisted
    log.msg('Initialize twisted')
    reactor.connectSSL(ip, port, EnvironmentFactory(manager, satelliteID, key), ClientContextFactory())

    # Add shutdown hooks
    log.msg('Add shutdown hooks')
    def terminate():
        log.msg('Shutdown hook "terminate" called...')
        reactor.callFromThread(manager.shutdown)
        reactor.callFromThread(reactor.stop)
        log.msg('Shutdown hook "terminate" leaving')

    rospy.on_shutdown(terminate)

    # Start ROS Launcher
    log.msg('Start ROS launcher')
    launcher.start()

    # Setup periodic calling of ROS spin_once in main thread
    log.msg('Add periodic call for ROS spin_once')
    LoopingCall(launcher.spin_once).start(0.1)

    # Start twisted (without signal handles as ROS also registers signal handlers)
    log.msg('Initialization completed')
    log.msg('Enter mainloop')
    reactor.run(installSignalHandlers=False)

    # Stop ROS Launcher (should normally already be done as ROS handles the signals)
    launcher.stop()
    log.msg('Leaving Administrator')

def _get_argparse():
    from argparse import ArgumentParser

    parser = ArgumentParser(prog='Administrator',
                            description='Administrator of App Nodes in Linux Container for the reappengine.')

    parser.add_argument('commID', help='Communication address of this node.')
    parser.add_argument('ip', type=str, help='IP address of the satellite node to which the connection should be established.')
    parser.add_argument('port', type=int, help='Port of the satellite node to which the connection should be established.')
    parser.add_argument('satelliteID', help='Communication address of the node to which a connection should be established.')
    parser.add_argument('key', help='Key which is used to identify this node with the satellite node.')

    return parser

if __name__ == '__main__':
    from twisted.internet import reactor

    parser = _get_argparse()
    args = parser.parse_args()
    
    if not validateAddress(args.commID):
        print 'CommID is not a valid address.'
        exit(1)
     
    if not validateAddress(args.satelliteID):
        print 'SatelliteID is not a valid address.'
        exit(1)
    
    main(reactor, args.ip, args.port, args.commID, args.satelliteID, args.key)
