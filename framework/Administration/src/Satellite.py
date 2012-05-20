#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Satellite.py
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
from zope.interface import implements
from twisted.python import log
from twisted.internet.ssl import DefaultOpenSSLContextFactory
from twisted.internet.task import LoopingCall

# Python specific imports
import sys

# Custom imports
import settings
from Comm.Message import MsgDef
from Comm.Message import  MsgTypes
from Comm.Message.Base import validateSuffix
from Comm.Message.Interfaces import IServerImplementation
from Comm.Factory import ReappengineClientFactory, ReappengineServerFactory
from Comm.CommManager import CommManager
from SatelliteUtil.Manager import SatelliteManager #@UnresolvedImport
from SatelliteUtil.Triggers import DefaultRoutingTrigger, SatelliteRoutingTrigger #@UnresolvedImport
from MiscUtility import generateID

class EnvironmentServerImplementation(object):
    """ ServerImplementation which is used in the satellite node for the connections to the
        environment nodes.
    """
    implements(IServerImplementation)
    
    def __init__(self, commMngr):
        """ Initialize the EnvironmentServerImplementation.

            @param commMngr:    CommManager which is responsible for handling the communication
                                in this node.
            @type  commMngr:    CommManager
        """
        self.commManager = commMngr
        self._expectedConnections = {}
        self._upConnections = {}
    
    def addExpectedConnection(self, commID, key):
        """ Add a communication ID to the dictionary of expected incoming
            connections.
            
            @param commID:  CommID of expected connection.
            @type  commID:  str
            
            @param key:     Key which is used to authenticate the connection.
            @type  key:     str
        """
        self._expectedConnections[commID] = key
    
    def authOrigin(self, origin, key):
        if origin not in self._expectedConnections:
            log.msg('Received a initialization request from an unexpected source.')
            return False
        
        if self._expectedConnections[origin] != key:
            log.msg('Tried to initialize a connection without a valid key.')
            return False
        
        return True
    
    def getResponse(self, origin):    
        return { 'origin' : self.commManager.commID, 'dest' : origin, 'key' : generateID() }
    
    def saveState(self, content):
        dest = content['dest']
        del self._expectedConnections[dest]
        self._upConnections[dest] = content['key']
    
    def unregisterConnection(self, conn):
        dest = conn.dest
        self._expectedConnections[dest] = self._upConnections[dest]
        del self._upConnections[dest]

class SatelliteServerImplementation(object):
    """ ServerImplementation which is used in the satellite node for the connection to the
        other satellite nodes.
    """
    implements(IServerImplementation)
    
    def __init__(self, commMngr):
        """ Initialize the EnvironmentServerImplementation.

            @param commMngr:    CommManager which is responsible for handling the communication
                                in this node.
            @type  commMngr:    CommManager
        """
        self.commManager = commMngr
    
    def authOrigin(self, origin, key):
        return True
    
    def getResponse(self, origin):
        return { 'origin' : self.commmManager.commID, 'dest' : origin, 'key' : '' }
    
    def saveState(self, content):
        pass
    
    def unregisterConnection(self, conn):
        pass

def main(reactor, ip, port, uid):
    log.startLogging(sys.stdout)
    
    log.msg('Start initialization...')
    commID = MsgDef.PREFIX_SATELLITE_ADDR + uid
    ctx = DefaultOpenSSLContextFactory('Comm/key.pem', 'Comm/cert.pem') # TODO: ???
    
    # Create Manager
    commManager = CommManager(reactor, commID)
    satelliteManager = SatelliteManager(ctx.getContext(), commManager)
    
    # Create triggers
    defaultTrigger = DefaultRoutingTrigger(commManager)
    satelliteTrigger = SatelliteRoutingTrigger(commManager, satelliteManager)
    
    # Initialize twisted
    log.msg('Initialize twisted')
    
    # Server for connections from the containers
    factory = ReappengineServerFactory(commManager, EnvironmentServerImplementation(commManager), defaultTrigger)
    factory.addApprovedMessageTypes([ MsgTypes.ROUTE_INFO,
                                      MsgTypes.ROS_RESPONSE,
                                      MsgTypes.ROS_MSG ])
    reactor.listenSSL(settings.PORT_SATELLITE_ENVIRONMENT, factory, ctx)
    
    # Server for connections from other satellites
    factory = ReappengineServerFactory(commManager, SatelliteServerImplementation(commManager, satelliteManager), satelliteTrigger)
    factory.addApprovedMessageTypes([ MsgTypes.ROUTE_INFO,
                                      MsgTypes.ROS_MSG ])
    reactor.listenSSL(settings.PORT_SATELLITE_SATELLITE, factory, ctx)
    
    # Client for connection to ContainerLauncher
    factory = ReappengineClientFactory(commManager, MsgDef.PREFIX_CONTAINER_ADDR + uid, '', satelliteTrigger)
    factory.addApprovedMessageTypes([ MsgTypes.ROUTE_INFO,
                                      MsgTypes.CONTAINER_STATUS ])
    reactor.connectSSL('localhost', settings.PORT_CONTAINER_MNGR, factory, ctx)
    
    # Client for connection to Master
    factory = ReappengineClientFactory(commManager, MsgDef.MASTER_ADDR, '', satelliteTrigger, True)
    factory.addApprovedMessageTypes([ MsgTypes.ENV_CREATE,
                                      MsgTypes.ENV_DESTROY,
                                      MsgTypes.ROUTE_INFO,
                                      MsgTypes.CONNECT,
                                      MsgTypes.ROS_MSG ])
    reactor.connectSSL(ip, port, factory, ctx)
    
    # Setup periodic calling of Load Balancer Updater
    log.msg('Add periodic call for Load Balancer Update.')
    LoopingCall(satelliteManager.updateLoadInfo).start(settings.LOAD_INFO_UPDATE)
    
    # Start twisted
    log.msg('Initialization completed')
    log.msg('Enter mainloop')
    reactor.run()       # TODO: Call shutdown on manager before reactor is terminated.
    log.msg('Leaving Satellite')

def _get_argparse():
    from argparse import ArgumentParser

    parser = ArgumentParser(prog='Satellite',
                            description='# TODO: Add description')

    parser.add_argument('uid', help='Unique ID which is used to identify this machine.')
    parser.add_argument('ip', type=str, help='IP address of the Master node.')
    parser.add_argument('port', type=int, help='Port of the Master node.')

    return parser

if __name__ == '__main__':
    from twisted.internet import reactor

    parser = _get_argparse()
    args = parser.parse_args()
    
    if not validateSuffix(args.uid):
        print 'UID is not a valid.'
        exit(1)
    
    main(reactor, args.ip, args.port, args.uid)
