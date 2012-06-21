#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Container.py
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

if settings.USE_SSL:
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
