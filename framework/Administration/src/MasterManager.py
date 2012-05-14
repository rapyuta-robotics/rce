#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       MasterManager.py
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
from twisted.internet.ssl import DefaultOpenSSLContextFactory

# Python specific imports
import sys

# Custom imports
from Exceptions import InvalidRequest, InternalError
from Comm.Message.Base import Message, validateAddress
from Comm.Message.TypeBase import MessageTypes as MsgTypes
from Comm.Manager import ReappengineManager
from Comm.Factory import ReappengineServerFactory
from MiscUtility import generateID

class MasterServerFactory(ReappengineServerFactory):
    """ Factory which is used in the master node for the connection to the satellite nodes.
    """
    def filterMessage(self, msgType):
        return msgType not in [ MsgTypes.ROUTE_INFO,    # TODO: <- Necessary in master node?
                                MsgTypes.LOAD_INFO,
                                MsgTypes.ROS_RESPONSE,
                                MsgTypes.ROS_MSG ]
    
    def getResponse(self, origin):
        return { 'origin' : self.manager.commID, 'dest' : origin, 'key' : '' }  # TODO: Assign a new destination ID to dest?
    
    def postInitTrigger(self, origin):
        ### TODO: Send MsgTypes.CONNECT
        #         ROUTE_INFO what exactly?
        
        msg = Message()
        msg.msgType = MsgTypes.ROUTE_INFO
        msg.dest = origin
        msg.content = self.manager.getSatelliteRouting()
        self.manager.sendMessage(msg)

class MasterManager(ReappengineManager):
    """ Manager which is used for master node who is responsible for the management
        of the satellites nodes and is the connection point for the outside world.
    """
    def __init__(self, reactor, commID):
        """ Initialize the necessary variables for the base manager.

            @param reactor:     TODO: Add description
            @type  reactor:     reactor

            @param commID:  CommID of this node which can be used by other
                            nodes to identify this node.
            @type  commID:  str
        """
        super(MasterManager, self).__init__(reactor, commID)
        
        # Setup list of valid message processors
        self.msgProcessors.extend([ ])

    def shutdown(self):
        """ Method is called when the manager/factory is stopped.
        """
        super(MasterManager, self).shutdown()
        
        # Stop all satellites nodes

def main(reactor, port, commID):
    # Start logger
    log.startLogging(sys.stdout)
    
    log.msg('Start initialization...')

    # Create Manager
    manager = MasterManager(reactor, commID)

    # Initialize twisted
    log.msg('Initialize twisted')
    reactor.listeSSL(port, MasterServerFactory(manager), DefaultOpenSSLContextFactory('Comm/key.pem', 'Comm/cert.pem'))

    # Start twisted (without signal handles as ROS also registers signal handlers)
    log.msg('Initialization completed')
    log.msg('Enter mainloop')
    reactor.run()       # TODO: Call shutdown on manager before reactor is terminated.
    log.msg('Leaving Master')

def _get_argparse():
    from argparse import ArgumentParser

    parser = ArgumentParser(prog='Master',
                            description='Master Node of the Satellite Nodes of the reappengine.')

    parser.add_argument('commID', help='Communication address of this node.')
    parser.add_argument('port', type=int, help='Port of the satellite node to which the connection should be established.')
    
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
    
    main(reactor, args.port, args.commID)
