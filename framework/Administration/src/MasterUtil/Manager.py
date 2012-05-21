#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       MasterManager.py
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
from Exceptions import InvalidRequest, InternalError
from Comm.Message.Base import Message, validateAddress
from Comm.Message import MsgTypes
from Comm.CommManager import CommManager
from Comm.Factory import ReappengineServerFactory
from Comm.Interfaces import IUIDServer #@UnresolvedImport
from MiscUtility import generateID

class MasterManager(object):
    """ Manager which is used for master node who is responsible for the management
        of the satellites nodes and is the connection point for the outside world.
    """
    implements(IUIDServer)
    
    def __init__(self, reactor, commID):
        """ Initialize the necessary variables for the base manager.

            @param reactor:     TODO: Add description
            @type  reactor:     reactor

            @param commID:  CommID of this node which can be used by other
                            nodes to identify this node.
            @type  commID:  str
        """
        super(MasterManager, self).__init__(reactor, commID)
        
        # Setup list of valid message processors
        self.msgProcessors.extend([ ])
    
    def getUID(self):
        """ Callback method which provides a new UID for a machine.
            
            @return:    Unique ID which should be used to build the CommID for Satellite and
                        Container Node.
            @rtype:     str
        """
        # TODO: Return a UID...
    
    def shutdown(self):
        """ Method is called when the manager/factory is stopped.
        """
        super(MasterManager, self).shutdown()
        
        # Stop all satellites nodes
