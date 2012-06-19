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

# Custom imports
import settings
from Comm.CommUtil import validateAddress
from Comm.Message import MsgDef
from Comm.Message import MsgTypes
from Comm.Factory import RCEClientFactory
from Comm.CommManager import CommManager
from EnvironmentUtil.Manager import ROSManager
from SSLUtil import RCEClientContext

def main(reactor, commID, serverID):
    # Start logger
    f = open('/opt/rce/data/env.log', 'w')
    log.startLogging(f)
    
    log.msg('Start initialization...')
    
    if settings.USE_SSL:
        ctx = RCEClientContext( '/opt/rce/data/ca.pem',
                                '/opt/rce/data/cert.pem',
                                '/opt/rce/data/key.pem' )
    
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
    if settings.USE_SSL:
        reactor.connectSSL(settings.IP_SERVER, settings.PORT_SERVER_ENVIRONMENT, factory, ctx)
    else:
        reactor.connectTCP(settings.IP_SERVER, settings.PORT_SERVER_ENVIRONMENT, factory)
    
    # Client for connection to launcher
    factory = RCEClientFactory( commManager,
                                MsgDef.LAUNCHER_ADDR )
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
    
    f.close()

def _get_argparse():
    from argparse import ArgumentParser

    parser = ArgumentParser(prog='Environment',
                            description='Communication interface of ROS Apps and the Server node of the framework.')

    parser.add_argument('commID', help='Communication address of this node.')
    parser.add_argument('serverID', help='Communication address of the server node to which a connection should be established.')
    
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
