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

# zope specific imports
from zope.interface import implements

# twisted specific imports
from twisted.python import log
from twisted.internet.ssl import DefaultOpenSSLContextFactory
from twisted.internet.task import LoopingCall

# Python specific imports
import sys

# Custom imports
import settings
from Comm.Message import MsgDef
from Comm.Message import  MsgTypes
from Comm.Message.Base import Message
from Comm.Interfaces import IServerImplementation #@UnresolvedImport
from Comm.UIDServer import UIDServerFactory #@UnresolvedImport
from Comm.Factory import ReappengineServerFactory
from Comm.CommManager import CommManager
from Comm.Interfaces import IPostInitTrigger #@UnresolvedImport
from MasterUtil.Manager import MasterManager #@UnresolvedImport

def main(reactor):
    # Start logger
    log.startLogging(sys.stdout)
    
    log.msg('Start initialization...')
    #ctx = DefaultOpenSSLContextFactory('Comm/key.pem', 'Comm/cert.pem')

    # Create Manager
    commManager = CommManager(reactor, MsgDef.MASTER_ADDR)
    masterManager = MasterManager(commManager)
    
    # Create Trigger
    trigger = MasterTrigger(commManager, masterManager)
    
    # Initialize twisted
    log.msg('Initialize twisted')
    
    # Server for UID distribution
    reactor.listenTCP(settings.PORT_UID, UIDServerFactory(masterManager))
    
    # Server for connections from the satellites
    factory = ReappengineServerFactory( commManager,
                                        MasterServerImplementation(commManager, masterManager),
                                        trigger )
    factory.addApprovedMessageTypes([ MsgTypes.ROUTE_INFO,  # TODO: <- Necessary?
                                      MsgTypes.LOAD_INFO ])
    #reactor.listenSSL(settings.PORT_MASTER, factory, ctx)
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
    
    ### TODO: Debugging only
    def test1():
        log.msg('Start test 1...')
        uid = masterManager.getSatellites()[0]['commID']
        log.msg('test 1: Start container commID: "{0}", satelliteID: "{1}".'.format('TESTID', uid))
        masterManager.addContainer('TESTID', '/reappengine/home', uid)
    
    def test2():
        log.msg('Start test 2...')
        uid = masterManager.getSatellites()[0]['commID']
        log.msg('test 2: Stop container commID: "{0}", satelliteID: "{1}".'.format('TESTID', uid))
        masterManager.removeContainer('TESTID', uid)
    
    reactor.callLater(10, test1)
    #reactor.callLater(20, test2)
    #reactor.callLater(30, test1)
    
    # Start twisted (without signal handles as ROS also registers signal handlers)
    log.msg('Initialization completed')
    log.msg('Enter mainloop')
    reactor.run()
    log.msg('Leaving Master')

if __name__ == '__main__':
    from twisted.internet import reactor
    main(reactor)
