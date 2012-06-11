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

# twisted specific imports
from twisted.python import log
from twisted.internet.ssl import ClientContextFactory

# Python specific imports
import sys

# Custom imports
import settings
from Comm.CommUtil import validateAddress
from Comm.Message import MsgDef
from Comm.Message import MsgTypes
from Comm.Factory import RCEClientFactory
from Comm.CommManager import CommManager
from EnvironmentUtil.Manager import ROSManager

def main(reactor, commID, serverID):
    # Start logger
    f = open('/opt/rce/data/env.log', 'w')
    log.startLogging(f)
    
    log.msg('Start initialization...')

    # Init ROS
    log.msg('Initialize ROS node')
    rospy.init_node('Administration')

    # Create Manager
    commManager = CommManager(reactor, commID)
    rosManager = ROSManager(commManager)

    # Initialize twisted
    log.msg('Initialize twisted')
    
    # Client for connection to server
    factory = RCEClientFactory( commManager,
                                        serverID )
    factory.addApprovedMessageTypes([ MsgTypes.ROUTE_INFO,
                                      MsgTypes.ROS_ADD,
                                      MsgTypes.ROS_REMOVE,
                                      MsgTypes.ROS_USER,
                                      # MsgTypes.ROS_GET,    # Only push valid; no pull
                                      MsgTypes.ROS_MSG ])
    #reactor.connectSSL(ip, settings.PORT_SATELLITE_ENVIRONMENT, factory, ClientContextFactory())
    reactor.connectTCP(settings.IP_SATELLITE, settings.PORT_SATELLITE_ENVIRONMENT, factory)
    
    # Client for connection to launcher
    factory = RCEClientFactory( commManager,
                                        MsgDef.LAUNCHER_ADDR )
    #reactor.connectSSL('localhost', settings.PORT_LAUNCHER, factory, ClientContextFactory())
    reactor.connectTCP('localhost', settings.PORT_LAUNCHER, factory)

    # Add shutdown hooks
    log.msg('Add shutdown hooks')
    def terminate():
        log.msg('Shutdown hook "terminate" called...')
        reactor.callFromThread(rosManager.shutdown)
        reactor.callFromThread(reactor.stop)
        log.msg('Shutdown hook "terminate" leaving')

    rospy.on_shutdown(terminate)

    # Start twisted (without signal handles as ROS also registers signal handlers)
    log.msg('Initialization completed')
    log.msg('Enter mainloop')
    reactor.run(installSignalHandlers=False)
    log.msg('Leaving Administrator')

def _get_argparse():
    from argparse import ArgumentParser

    parser = ArgumentParser(prog='Environment',
                            description='Administrator of App Nodes in Linux Container for the reappengine.')

    parser.add_argument('commID', help='Communication address of this node.')
    parser.add_argument('serverID', help='Communication address of the node to which a connection should be established.')
    
    return parser

if __name__ == '__main__':
    from twisted.internet import reactor

    parser = _get_argparse()
    args = parser.parse_args()
    
    if not validateAddress(args.commID):
        print 'CommID is not a valid address.'
        exit(1)
     
    if not validateAddress(args.serverID):
        print 'ServerID is not a valid address.'
        exit(1)
    
    main(reactor, args.commID, args.serverID)
