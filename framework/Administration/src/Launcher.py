#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Launcher.py
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
from twisted.internet.ssl import ClientContextFactory

# Python specific imports
import sys

# Custom imports
import settings
from Comm.Message import MsgDef
from Comm.Message import MsgTypes
from Comm.Factory import ReappengineServerFactory
from Comm.CommManager import CommManager
from LauncherUtil.Manager import LauncherManager

def main(reactor):
    # Start logger
    f = open('launcher.log', 'w')
    log.startLogging(f)
    
    log.msg('Start initialization...')

    # Create Manager
    commManager = CommManager(reactor, MsgDef.LAUNCHER_ADDR)
    LauncherManager(commManager)

    # Initialize twisted
    log.msg('Initialize twisted')
    
    # Server for connection from the environment
    factory = ReappengineServerFactory( commManager )
    factory.addApprovedMessageTypes([ MsgTypes.ROS_ADD,
                                      MsgTypes.ROS_REMOVE ])
    #reactor.listenSSL(settings.PORT_LAUNCHER, factory, ClientContextFactory())
    reactor.listenTCP(settings.PORT_LAUNCHER, factory)
    
    # Start twisted
    log.msg('Initialization completed')
    log.msg('Enter mainloop')
    reactor.run()
    log.msg('Leaving Launcher')

def _get_argparse():
    from argparse import ArgumentParser

    parser = ArgumentParser(prog='Launcher',
                            description='Launcher of App Nodes in Linux Container for the reappengine.')

    return parser

if __name__ == '__main__':
    from twisted.internet import reactor

    #parser = _get_argparse()
    #args = parser.parse_args()
    
    main(reactor)
