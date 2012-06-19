#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Container.py
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

# Python specific imports
import os, sys

# Custom imports
import settings
from Comm.Message import MsgDef
from Comm.Message import  MsgTypes
from Comm.Factory import RCEServerFactory
from Comm.CommManager import CommManager
from Comm.CommUtil import validateSuffix
from ContainerUtil.Manager import ContainerManager
from SSLUtil import RCEServerContext

def main(reactor, uid):
    log.startLogging(sys.stdout)
    
    log.msg('Start initialization...')
    commID = MsgDef.PREFIX_PRIV_ADDR + uid
    
    if settings.USE_SSL:
        path = os.path.join(settings.SSL_DIR, 'container')
        ctx = RCEServerContext( os.path.join(settings.SSL_DIR, 'Machine.cert'),
                                os.path.join(path, 'toServer.cert'),
                                os.path.join(path, 'toServer.key') )
    
    # Create Manager
    commManager = CommManager(reactor, commID)
    containerManager = ContainerManager(commManager)
    
    # Initialize twisted
    log.msg('Initialize twisted')
    
    # Server for connection(s) from the server
    factory = RCEServerFactory(commManager) # , trigger)
    factory.addApprovedMessageTypes([ # MsgTypes.ROUTE_INFO,
                                      MsgTypes.CONTAINER_START,
                                      MsgTypes.CONTAINER_STOP ])
    if settings.USE_SSL:
        reactor.listenSSL(settings.PORT_CONTAINER_MNGR, factory, ctx)
    else:
        reactor.listenTCP(settings.PORT_CONTAINER_MNGR, factory)

    # Setup shutdown hooks
    reactor.addSystemEventTrigger('before', 'shutdown', containerManager.shutdown)
    reactor.addSystemEventTrigger('before', 'shutdown', commManager.shutdown)
    
    # Start twisted
    log.msg('Initialization completed')
    log.msg('Enter mainloop')
    reactor.run() 
    log.msg('Leaving ContainerManager')

def _get_argparse():
    from argparse import ArgumentParser

    parser = ArgumentParser(prog='ContainerManager',
                            description='Manager of the containers in one machine for the framework.')

    parser.add_argument('uid', help='Unique ID which is used to identify this machine.')

    return parser

if __name__ == '__main__':
    from twisted.internet import reactor

    parser = _get_argparse()
    args = parser.parse_args()
    
    if not validateSuffix(args.uid):
        print 'UID is not a valid.'
        exit(1)
    
    main(reactor, args.uid)
