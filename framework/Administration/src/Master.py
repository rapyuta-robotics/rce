#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Master.py
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

# Python specific imports
import sys

# Custom imports
import settings
from Comm.Message import MsgDef
from Comm.Message import  MsgTypes
from Comm.UIDServer import UIDServerFactory #@UnresolvedImport
from Comm.Factory import ReappengineServerFactory, BaseServerImplementation
from Comm.CommManager import CommManager
from MasterUtil.Manager import MasterManager #@UnresolvedImport
from SatelliteUtil.Triggers import DefaultRoutingTrigger, SatelliteRoutingTrigger #@UnresolvedImport
from MiscUtility import generateID

### TODO : v
def postInitTrigger(self, origin):
    ### TODO: Send MsgTypes.CONNECT
    #         ROUTE_INFO what exactly?
    
    msg = Message()
    msg.msgType = MsgTypes.ROUTE_INFO
    msg.dest = origin
    msg.content = self.manager.getSatelliteRouting()
    self.manager.sendMessage(msg)

def main(reactor):
    # Start logger
    log.startLogging(sys.stdout)
    
    log.msg('Start initialization...')
    ctx = DefaultOpenSSLContextFactory('Comm/key.pem', 'Comm/cert.pem')

    # Create Manager
    commManager = CommManager(reactor, MsgDef.MASTER_ADDR)
    manager = MasterManager(commManager)

    # Initialize twisted
    log.msg('Initialize twisted')
    reactor.listenTCP(settings.PORT_UID, UIDServerFactory(manager))
    
    factory = ReappengineServerFactory( commManager,
                                        BaseServerImplementation(commManager),
                                        defaultTrigger )
    factory.addApprovedMessageTypes([ MsgTypes.ROUTE_INFO,  # TODO: <- Necessary?
                                      MsgTypes.LOAD_INFO,
                                      MsgTypes.ROS_RESPONSE,
                                      MsgTypes.ROS_MSG ])
    reactor.listenSSL(settings.PORT_MASTER, factory, ctx)

    # Setup shutdown hooks
    reactor.addSystemEventTrigger('before', 'shutdown', manager.shutdown)
    reactor.addSystemEventTrigger('before', 'shutdown', commManager.shutdown)
    
    # Start twisted (without signal handles as ROS also registers signal handlers)
    log.msg('Initialization completed')
    log.msg('Enter mainloop')
    reactor.run()
    log.msg('Leaving Master')

if __name__ == '__main__':
    from twisted.internet import reactor
    main(reactor)
