#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       DB.py
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
import settings
from Comm.Message import MsgDef
from Comm.Message import  MsgTypes
from Comm.Factory import RCEServerFactory
from Comm.CommManager import CommManager

from DBUtil.DBBase import DBBase
from DBUtil.DB.DjangoDB import Django

class DBManager(DBBase):
    """ Manager which is used to handle the database requests.
    """
    def __init__(self, commMngr):
        """ Initialize the DBManager.

            @param commMngr:    CommManager which is responsible for handling the communication
                                in this node.
            @type  commMngr:    CommManager
        """
        super(DBManager, self).__init__(commMngr)
        
        self._django = Django(commMngr.reactor)
        self._django.start()
    
    def getRobotSpecs(self, robotID):
        """ # TODO: Add description.
            
            @return:    Path to the robot's home folder.
            @rtype:     str
        """
        return '/reappengine/home' # TODO: Add actual code here
    
    def getNodeSpecs(self, nodeID):
        """ # TODO: Add description
            
            
        """
        return self._django.getNode(nodeID)

def main(reactor):
    # Start logger
    log.startLogging(sys.stdout)
    
    log.msg('Start initialization...')
    #ctx = DefaultOpenSSLContextFactory('Comm/key.pem', 'Comm/cert.pem')

    # Create Manager
    commManager = CommManager(reactor, MsgDef.MASTER_ADDR)
    DBManager(commManager)
    
    # Initialize twisted
    log.msg('Initialize twisted')
    
    # Server for connections from the servers
    factory = RCEServerFactory( commManager )
    factory.addApprovedMessageTypes([ MsgTypes.DB_REQUEST ])
    #reactor.listenSSL(settings.PORT_DB, factory, ctx)
    reactor.listenTCP(settings.PORT_DB, factory)
    
    # Setup shutdown hooks
    reactor.addSystemEventTrigger('before', 'shutdown', commManager.shutdown)
    
    # Start twisted (without signal handles as ROS also registers signal handlers)
    log.msg('Initialization completed')
    log.msg('Enter mainloop')
    reactor.run()
    log.msg('Leaving Master')

if __name__ == '__main__':
    from twisted.internet import reactor
    main(reactor)
