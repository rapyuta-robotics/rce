#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Environment.py
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

# twisted specific imports
from twisted.python import log
from twisted.internet.ssl import ClientContextFactory
from twisted.internet.task import LoopingCall

# Python specific imports
import sys

# Custom imports
from Comm.Message.Base import validateAddress
from Comm.Message import MsgTypes
from Comm.Factory import ReappengineClientFactory
from Comm.CommManager import CommManager
from ROSUtil.Manager import ROSManager #@UnresolvedImport

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
    commManager = CommManager(reactor, commID)
    rosManager = ROSManager(launcher, commManager)

    # Initialize twisted
    log.msg('Initialize twisted')
    factory = ReappengineClientFactory(commManager, satelliteID, key)
    factory.addApprovedMessageTypes([ MsgTypes.ROUTE_INFO,
                                      MsgTypes.ROS_ADD,
                                      MsgTypes.ROS_REMOVE,
                                      MsgTypes.ROS_GET,
                                      MsgTypes.ROS_MSG ])
    reactor.connectSSL(ip, port, factory, ClientContextFactory())

    # Add shutdown hooks
    log.msg('Add shutdown hooks')
    def terminate():
        log.msg('Shutdown hook "terminate" called...')
        reactor.callFromThread(rosManager.shutdown)
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
