#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       DBInterface.py
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
from twisted.internet.defer import Deferred

# Python specific imports
import json

# Custom imports
from Comm.Message.Interfaces import IMessageProcessor
from Comm.Message import MsgDef
from Comm.Message import MsgTypes
from Comm.Message.Base import Message

class _Method(object):
    """ Representation of a remote method object.
    """
    def __init__(self, deferredSend, name):
        self._deferredSend = deferredSend
        self._name = name
    
    def __call__(self, *args, **kw):
        return self._deferredSend(self._name, args, kw)

class DBInterface(object):
    """ Interface to the database which allows calling all methods which are defined
        in the remote subclass of DBBase and which are not starting with '_'.
    """
    implements(IMessageProcessor)
    
    IDENTIFIER = MsgTypes.DB_RESPONSE
    
    def __init__(self, commMngr):
        """ Initialize the DBInterface.

            @param commMngr:    CommManager which is responsible for handling the communication
                                in this node.
            @type  commMngr:    CommManager
        """
        self._commManager = commMngr
        
        self._counter = 0
        self._pending = {}
    
    def __getattr__(self, attr):
        """ Modified getter method to return _Method instances which are used for the
            remote calls.
        """
        try:
            super(DBInterface, self).__getattr__(attr)
        except AttributeError:
            if attr[0] == '_':
                raise
            
            return _Method(self._deferredSend, attr)
    
    def _deferredSend(self, name, args, kw):
        """ Internally used callback for _Method which is used to send a request.
        """
        deferred = Deferred()
        
        self._counter = (self._counter + 1) % 1000
        self._pending[str(self._counter)] = deferred
        
        msg = Message()
        msg.msgType = MsgTypes.DB_REQUEST
        msg.dest = MsgDef.DB_ADDR
        msg.content = { 'id'     : str(self._counter),
                        'method' : name,
                        'args'   : json.dumps(args),
                        'kw'     : json.dumps(kw) }
        self._commManager.sendMessage(msg)
        
        return deferred
    
    def processMessage(self, msg):
        """ Callback method for Message.Handler to process the received message.
        """
        msg = msg.content
        deferred = self._pending.pop(msg['id'])
        
        if msg['status']:
            deferred.callback(json.loads(msg['resp']))
        else:
            deferred.errback(json.loads(msg['resp']))
