#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       DBBase.py
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

# zope specific imports
from zope.interface import implements

# twisted specific imports
from twisted.internet.defer import maybeDeferred

# Python specific imports
import json

# Custom imports
from Comm.Message.Interfaces import IMessageProcessor
from Comm.Message import MsgTypes
from Comm.Message.Base import Message

from Type import DBRequestMessage, DBResponseMessage

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
