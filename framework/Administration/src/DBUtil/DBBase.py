#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       DBBase.py
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
from twisted.internet.defer import maybeDeferred

# Python specific imports
import json

# Custom imports
from Comm.Message.Interfaces import IMessageProcessor #@UnresolvedImport
from Comm.Message import MsgTypes
from Comm.Message.Base import Message

from Type import DBRequestMessage, DBResponseMessage #@UnresolvedImport

class DBBase(object):
    """ Provides the base class which can be subclassed and to which the remotely callable
        methods can be added.
    """
    implements(IMessageProcessor)
    
    IDENTIFIER = MsgTypes.DB_REQUEST
    
    def __init__(self, commMngr):
        """ Initialize the DBBase.

            @param commMngr:    CommManager which is responsible for handling the communication
                                in this node.
            @type  commMngr:    CommManager
        """
        self._commManager = commMngr
        
        # Register Content Serializers
        self._commMngr.registerContentSerializers([ DBRequestMessage(),
                                                    DBResponseMessage() ])
        
        # Register Message Processors
        self._commMngr.registerMessageProcessors([ self ])
    
    def _send(self, resp, status, uid, dest):
        """ Internally used method to send a response.
        """
        msg = Message()
        msg.msgType = MsgTypes.DB_RESPONSE
        msg.dest = dest
        msg.content = { 'id'     : uid,
                        'status' : status,
                        'resp'   : json.dumps(resp) }
        self._commManager.sendMessage(msg)
    
    def processMessage(self, msg):
        """ Callback method for Message.Handler to process the received message.
        """
        msg = msg.content
        
        try:
            f = getattr(self, msg['method'])
        except (AttributeError, TypeError):
            self._send('DBInterface has not method called "{0}".'.format(msg['method']), False, msg['id'])
        else:
            deferred = maybeDeferred(f(*json.loads(msg['args'], **json.loads(['kw']))))
            deferred.addCallbacks( self._send,
                                   self._send,
                                   (True, msg['id'], msg.origin),
                                   None,
                                   (False, msg['id'], msg.origin),
                                   None )
