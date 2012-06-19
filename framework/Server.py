#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Server.py
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
from twisted.internet.task import LoopingCall
from autobahn.websocket import listenWS

# Python specific imports
import os, sys

# Custom imports
import settings
from Comm.Message import MsgDef
from Comm.Message import  MsgTypes
from Comm.Factory import RCEClientFactory, RCEServerFactory
from Comm.CommManager import CommManager
from Comm.CommUtil import validateSuffix
from ServerUtil.Manager import ServerManager
from ServerUtil.Triggers import BaseRoutingTrigger, ServerRoutingTrigger, EnvironmentRoutingTrigger
from ServerUtil.ClientServer import WebSocketCloudEngineFactory

if settings.USE_SSL:
    from SSLUtil import RCEClientContext, RCEServerContext

class EnvironmentServerFactory(RCEServerFactory):
    """ RCEServerFactory which is used in the server node for the connections
        to the environment nodes.
    """
    def __init__(self, commMngr, serverMngr):
        """ Initialize the EnvironmentServerFactory.
            
            @param commMngr:    CommManager instance which should be used with this
                                factory and its build protocols.
            @type  commMngr:    CommManager
            
            @param serverMngr:   ServerManager which is used in this node.
            @type  serverMngr:   ServerManager
        """
        super(EnvironmentServerFactory, self).__init__(commMngr, EnvironmentRoutingTrigger(commMngr, serverMngr))
        self._serverManager = serverMngr
    
    def authOrigin(self, origin):
        return self._serverManager.authenticateContainerConnection(origin)
    
    def unregisterConnection(self, conn):
        super(EnvironmentServerFactory, self).unregisterConnection(conn)
        self._serverManager.setConnectedFlagContainer(conn.dest, False)

def main(reactor, ip, uid):
    log.startLogging(sys.stdout)
    
    log.msg('Start initialization...')
    commID = MsgDef.PREFIX_PUB_ADDR + uid
    
    if settings.USE_SSL:
        pathToRCE = os.path.join(settings.SSL_DIR, 'RCE.cert')
        path = os.path.join(settings.SSL_DIR, 'server')
        
        # SSL ContextFactory for connections from other server nodes
        serverContext = RCEServerContext( pathToRCE,
                                          os.path.join(path, 'toMaster.cert'),
                                          os.path.join(path, 'toMaster.key') )
        
        # SSL ContextFactory for connections to other server nodes and to master node
        clientContext = RCEClientContext( pathToRCE,
                                          os.path.join(path, 'toMaster.cert'),
                                          os.path.join(path, 'toMaster.key') )
        
        # SSL ContextFactory for connection to container node
        containerContext = RCEClientContext( os.path.join(settings.SSL_DIR, 'Machine.cert'),
                                             os.path.join(path, 'toContainer.cert'),
                                             os.path.join(path, 'toContainer.key') )
        
        # SSL ContextFactory for connections from environment nodes
        environmentContext = RCEServerContext( os.path.join(settings.SSL_DIR, 'Container.cert'),
                                               os.path.join(path, 'toEnv.cert'),
                                               os.path.join(path, 'toEnv.key') )
    else:
        clientContext = None
    
    # Create Manager
    commManager = CommManager(reactor, commID)
    serverManager = ServerManager(commManager, clientContext)
    
    # Initialize twisted
    log.msg('Initialize twisted')
    
    # Server for connections from the containers
    factory = EnvironmentServerFactory( commManager,
                                        serverManager )
    factory.addApprovedMessageTypes([ # MsgTypes.ROUTE_INFO,
    #                                  MsgTypes.ROS_RESPONSE, # Only used in pull implementation
                                      MsgTypes.ROS_MSG ])
    if settings.USE_SSL:
        reactor.listenSSL(settings.PORT_SERVER_ENVIRONMENT, factory, environmentContext)
    else:
        reactor.listenTCP(settings.PORT_SERVER_ENVIRONMENT, factory)
    
    # Server for connections from other servers
    factory = RCEServerFactory( commManager,
                                ServerRoutingTrigger(commManager, serverManager) )
    factory.addApprovedMessageTypes([ MsgTypes.ROUTE_INFO,
                                      MsgTypes.ROS_MSG ])
    if settings.USE_SSL:
        reactor.listenSSL(settings.PORT_SERVER_SERVER, factory, serverContext)
    else:
        reactor.listenTCP(settings.PORT_SERVER_SERVER, factory)
    
    # Client for connection to Container Node
    factory = RCEClientFactory( commManager,
                                MsgDef.PREFIX_PRIV_ADDR + uid,
                                BaseRoutingTrigger(commManager) )
    #factory.addApprovedMessageTypes([ MsgTypes.ROUTE_INFO,
    #                                  MsgTypes.CONTAINER_STATUS ])
    if settings.USE_SSL:
        reactor.connectSSL('localhost', settings.PORT_CONTAINER_MNGR, factory, containerContext)
    else:
        reactor.connectTCP('localhost', settings.PORT_CONTAINER_MNGR, factory)
    
    # Client for connection to Master
    factory = RCEClientFactory( commManager,
                                MsgDef.MASTER_ADDR,
                                BaseRoutingTrigger(commManager) )
    factory.addApprovedMessageTypes([ # MsgTypes.ROUTE_INFO,
                                      MsgTypes.CONNECT,
                                      MsgTypes.ID_RESPONSE ])
    if settings.USE_SSL:
        reactor.connectSSL(ip, settings.PORT_MASTER, factory, clientContext)
    else:
        reactor.connectTCP(ip, settings.PORT_MASTER, factory)
    
    # Server for connections from robots, i.e. the web, brokered by the master node
    listenWS(WebSocketCloudEngineFactory(serverManager, "ws://localhost:9000"))
    
    # Setup periodic calling of Load Balancer Updater
    log.msg('Add periodic call for Load Balancer Update.')
    LoopingCall(serverManager.updateLoadInfo).start(settings.LOAD_INFO_UPDATE)
    
    # Setup shutdown hooks
    #reactor.addSystemEventTrigger('before', 'shutdown', serverManager.shutdown)
    reactor.addSystemEventTrigger('before', 'shutdown', commManager.shutdown)
    
    # Start twisted
    log.msg('Initialization completed')
    log.msg('Enter mainloop')
    reactor.run()
    log.msg('Leaving Server')

def _get_argparse():
    from argparse import ArgumentParser

    parser = ArgumentParser(prog='Server',
                            description='Responsible for communication in one machine for the framework.')

    parser.add_argument('uid', help='Unique ID which is used to identify this machine.')
    parser.add_argument('ip', type=str, help='IP address of the Master node.')

    return parser

if __name__ == '__main__':
    from twisted.internet import reactor

    parser = _get_argparse()
    args = parser.parse_args()
    
    if not validateSuffix(args.uid):
        print 'UID is not a valid.'
        exit(1)
    
    main(reactor, args.ip, args.uid)
