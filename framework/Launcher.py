#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Launcher.py
#       
#       This file is part of the RoboEarth Cloud Engine framework.
#       
#       This file was originally created for RoboEearth
#       http://www.roboearth.org/
#       
#       The research leading to these results has received funding from
#       the European Union Seventh Framework Programme FP7/2007-2013 under
#       grant agreement no248942 RoboEarth.
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
#       \author/s: Dominique Hunziker 
#       
#       

# twisted specific imports
from twisted.python import log

# Custom imports
import settings
from Comm.Message import MsgDef
from Comm.Message import MsgTypes
from Comm.Factory import RCEServerFactory
from Comm.CommManager import CommManager
from LauncherUtil.Manager import LauncherManager

def main(reactor):
    # Start logger
    f = open('/home/ros/launcher.log', 'w')
    log.startLogging(f)
    
    log.msg('Start initialization...')
    
    # Create Manager
    commManager = CommManager(reactor, MsgDef.LAUNCHER_ADDR)
    LauncherManager(commManager)

    # Initialize twisted
    log.msg('Initialize twisted')
    
    # Server for connection from the environment
    factory = RCEServerFactory( commManager )
    factory.addApprovedMessageTypes([ MsgTypes.ROS_ADD,
                                      MsgTypes.ROS_REMOVE ])
    reactor.listenTCP(settings.PORT_LAUNCHER, factory)
    
    # Start twisted
    log.msg('Initialization completed')
    log.msg('Enter mainloop')
    reactor.run()
    log.msg('Leaving Launcher')
    
    f.close()

def _get_argparse():
    from argparse import ArgumentParser

    parser = ArgumentParser(prog='Launcher',
                            description='Launcher of App Nodes in Linux Container for the reappengine.')

    return parser

if __name__ == '__main__':
    from twisted.internet import reactor

    parser = _get_argparse()
    args = parser.parse_args()
    
    main(reactor)
