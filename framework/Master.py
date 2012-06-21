#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Master.py
#       
#       This file is part of the RoboEarth Cloud Engine framework.
#       
#       This file was originally created for RoboEearth - http://www.roboearth.org/
#       The research leading to these results has received funding from the European Union 
#       Seventh Framework Programme FP7/2007-2013 under grant agreement no248942 RoboEarth.
#       
#       Copyright 2012 RoboEarth
#       
#       Licensed under the Apache License, Version 2.0 (the "License");
#       you may not use this file except in compliance with the License.
#       You may obtain a copy of the License at
#       
#       http://www.apache.org/licenses/LICENSE-2.0
#       
#       Unless required by applicable law or agreed to in writing, software
#       distributed under the License is distributed on an "AS IS" BASIS,
#       WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#       See the License for the specific language governing permissions and
#       limitations under the License.
#       
#       \author/s: Dominique Hunziker <dominique.hunziker@gmail.com> 
#       
#       

# zope specific imports
from zope.interface import implements

# twisted specific imports
from twisted.python import log
from twisted.internet.task import LoopingCall

# Python specific imports
import os, sys

# Custom imports
import settings
from Comm.Message import MsgDef
from Comm.Message import  MsgTypes
from Comm.Message.Base import Message
from Comm.Factory import RCEServerFactory
from Comm.CommManager import CommManager
from Comm.Interfaces import IPostInitTrigger
from MasterUtil.Manager import MasterManager
from MasterUtil.UIDServer import UIDServerFactory

if settings.USE_SSL:
    from SSLUtil import createKeyCertPair, writeCertToFile, RCEServerContext

class MasterTrigger(object):
    """ PostInitTrigger which is used to send the available server as connection directive.
    """
    implements(IPostInitTrigger)
    
    def __init__(self, commMngr, masterMngr):
        """ Initialize the DefaultRoutingTrigger.

            @param commMngr:    CommManager which is responsible for handling the communication
                                in this node.
            @type  commMngr:    CommManager
            
            @param masterMngr:  MasterManager which is responsible for the handling of this node.
            @type  masterMngr:  MasterManager
        """
        self.commManager = commMngr
        self.masterManager = masterMngr
    
    def trigger(self, origin, ip):
        msg = Message()
        msg.msgType = MsgTypes.CONNECT
        msg.dest = origin
        msg.content = self.masterManager.getServers()
        self.commManager.sendMessage(msg)
        
        self.masterManager.addServer(origin, ip)

class MasterServerFactory(RCEServerFactory):
    """ RCEServerFactory which is used in the master node for the connections to the
        server nodes.
    """
    def __init__(self, commMngr, masterMngr):
        """ Initialize the MasterServerFactory.

            @param commMngr:    CommManager which is responsible for handling the communication
                                in this node.
            @type  commMngr:    CommManager
            
            @param masterMngr:  MasterManager which is used in this node.
            @type  masterMngr:  MasterManager
        """
        super(MasterServerFactory, self).__init__(commMngr, MasterTrigger(commMngr, masterMngr))
        self._masterManager = masterMngr
    
    def authOrigin(self, origin):
        return self._masterManager.checkUID(origin[MsgDef.PREFIX_LENGTH_ADDR:])
    
    def unregisterConnection(self, conn):
        super(MasterServerFactory, self).unregisterConnection(conn)
        self._masterManager.removeServer(conn.dest)

def main(reactor):
    # Start logger
    log.startLogging(sys.stdout)
    
    log.msg('Start initialization...')
    
    if settings.USE_SSL:
        # First check if the path is valid
        if not os.path.exists(settings.SSL_DIR):
            log.msg('SSL directory does not exist.')
            exit(1)
        
        if not os.path.isdir(settings.SSL_DIR):
            log.msg('Path to SSL directory is not a directory.')
            exit(1)
        
        nodeName = 'Master'
        caFile = os.path.join(settings.SSL_DIR, 'RCE.cert')
        
        (caCert, caKey) = createKeyCertPair('RCE')
        writeCertToFile(caCert, caFile)
        
        (cert, key) = createKeyCertPair(nodeName, caCert, caKey)
        
        # SSL ContextFactory for connections from initServer script
        uidContext = RCEServerContext(caFile, cert, key)
        
        # SSL ContextFactory for connections from server nodes
        serverContext = RCEServerContext(caFile, cert, key)
    else:
        caCert = None
        caKey = None

    # Create Manager
    commManager = CommManager(reactor, MsgDef.MASTER_ADDR)
    masterManager = MasterManager(commManager, caCert, caKey)
    
    # Initialize twisted
    log.msg('Initialize twisted')
    
    # Server for UID distribution
    if settings.USE_SSL:
        reactor.listenSSL(settings.PORT_UID, UIDServerFactory(masterManager), uidContext)
    else:
        reactor.listenTCP(settings.PORT_UID, UIDServerFactory(masterManager))
    
    # Server for connections from the servers
    factory = MasterServerFactory( commManager,
                                   masterManager )
    factory.addApprovedMessageTypes([ MsgTypes.ROUTE_INFO,  # TODO: <- Necessary?
                                      MsgTypes.LOAD_INFO,
                                      MsgTypes.ID_REQUEST,
                                      MsgTypes.ID_DEL ])
    if settings.USE_SSL:
        reactor.listenSSL(settings.PORT_MASTER, factory, serverContext)
    else:
        reactor.listenTCP(settings.PORT_MASTER, factory)
    
    # Server for initial outside connections
    ### 
    ### TODO: Add outside connection here!
    ###
    
    # Setup periodic calling of Clean Up
    log.msg('Add periodic call for Clean Up.')
    LoopingCall(masterManager.clean).start(settings.UID_TIMEOUT / 2)
    
    # Setup shutdown hooks
    #reactor.addSystemEventTrigger('before', 'shutdown', masterManager.shutdown)
    reactor.addSystemEventTrigger('before', 'shutdown', commManager.shutdown)
    
    # Start twisted (without signal handles as ROS also registers signal handlers)
    log.msg('Initialization completed')
    log.msg('Enter mainloop')
    reactor.run()
    log.msg('Leaving Master')

def _get_argparse():
    from argparse import ArgumentParser

    parser = ArgumentParser(prog='Environment',
                            description='Communication interface of ROS Apps and the Server node of the framework.')

    return parser

if __name__ == '__main__':
    from twisted.internet import reactor
    
    parser = _get_argparse()
    args = parser.parse_args()
    
    main(reactor)
