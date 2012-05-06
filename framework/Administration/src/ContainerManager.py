#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       ContainerManager.py
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
from twisted.internet.endpoints import SSL4ClientEndpoint
from twisted.internet.ssl import ClientContextFactory
from twisted.internet.task import LoopingCall

# Custom imports
from Exceptions import InvalidRequest, InternalError
from Comm.MessageDefinition import MessageTypesDefinition as MsgType, validateAddress
from Comm.Manager import ReappengineManager
from MiscUtility import generateID

class ContainerManager(ReappengineManager):
    """
    """
    def __init__(self, reactor, launcher, commID, satelliteID):
        """
        """
        # super(ContainerManager, self).__init__(reactor, commID)
        ReappengineManager.__init__(self, reactor, commID)
        
        # References used by the manager
        self._launcher = launcher
        self._satelliteID = satelliteID

        # Storage for references required to process messages
        self._runningNodes = {}
        self._interfaces = {}

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

    def unrgisterInterface(self, interface):
        """ Callback for Interface instance to unregister the interface.
        """
        self._interfaces.pop(interface.interfaceName, None)

    def resolveDest(self, dest):
        """ Resolve the received destination ID and decide what to do next.

            @param dest:    CommID of the destination node.
            @type  dest:    str

            @return:    The correct communication ID to where the message
                        should be forwarded.

            @raise:     DestinationError if no route for the ID can be found .
        """
        return self._satelliteID

    def messageReceived(self, msg):
        """ Process the received message.

            @param msg:     Message in from of a dictionary
            @type  msg:     dict
        """
        if self.isShutdown:
            return

        if msg.msgType == MsgType.ROS_ADD:
            for node in msg.content:
                self.addNode(node)

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

    def stopFactory(self):
        """ Method is called when the manager/factory is stopped.
        """
        # super(ContainerManager, self).stopFactory()
        ReappengineManager.stopFactory(self)

        for node in self._runningNodes:
            node.stop()

def main(reactor, ip, port, commID, satelliteID):
    print 'Start initialization...'

    # Init ROS
    print 'Initialize ROS node'
    rospy.init_node('Administration')

    # Init ROS Launcher
    print 'Initialize ROS launcher'
    launcher = roslaunch.scriptapi.ROSLaunch()

    # Create Manager
    manager = ContainerManager(reactor, launcher, commID, satelliteID)

    # Initialize twisted
    print 'Initialize twisted'
    endpoint = SSL4ClientEndpoint(reactor, ip, port, ClientContextFactory())
    d = endpoint.connect(manager)

    def sendMessages(p):
        reactor.callLater(0.5, p.sendMessage, 'Wie gehts?')
        reactor.callLater(1.5, p.sendMessage, 'Was gibts?')
        reactor.callLater(2.5, p.sendMessage, 'Tschau')

    d.addCallback(sendMessages)

    # Add shutdown hooks
    print 'Add shutdown hooks'
    def terminate():
        print 'Shutdown hook "terminate" called...'
        reactor.callFromThread(reactor.stop)
        print 'Shutdown hook "terminate" leaving'

    rospy.on_shutdown(terminate)

    # Start ROS Launcher
    print 'Start ROS launcher'
    launcher.start()

    # Setup periodic calling of ROS spin_once in main thread
    print 'Add periodic call for ROS spin_once'
    LoopingCall(launcher.spin_once).start(0.1)

    # Start twisted (without signal handles as ROS also registers signal handlers)
    print 'Initialization completed'
    print 'Enter mainloop'
    reactor.run(installSignalHandlers=False)

    # Stop ROS Launcher (should normally already be done as ROS handles the signals)
    launcher.stop()
    print 'Leaving Administrator'

def _get_argparse():
    from argparse import ArgumentParser

    parser = ArgumentParser(prog='Administrator',
                            description='Administrator of App Nodes in Linux Container for the reappengine.')

    parser.add_argument('commID', help='Communication address of this node.')
    parser.add_argument('ip', type=str, help='IP address of the satellite node to which the connection should be established.')
    parser.add_argument('port', type=int, help='Port of the satellite node to which the connection should be established.')
    parser.add_argument('satelliteID', help='Communication address of the node to which a connection should be established.')

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
    
    main(reactor, args.ip, args.port, args.commID, args.satelliteID)
