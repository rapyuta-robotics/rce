#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Type.py
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
#       \author/s: 
#       
#       

# zope specific imports
from zope.interface import implements

# Custom imports
from Exceptions import SerializationError
from Comm.Message.Interfaces import IContentSerializer
from Comm.Message import MsgTypes

class ConnectDirectiveMessage(object):
    """ Message type to provide node with connection directives.
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.CONNECT
    
    def serialize(self, s, data):
        s.addInt(len(data))
        
        for element in data:
            try:
                s.addElement(element['commID'])
                s.addElement(element['ip'])
            except KeyError as e:
                raise SerializationError('Could not serialize message of type ConnectDirective. Missing key: {0}'.format(e))
    
    def deserialize(self, s):
        return [{ 'commID' : s.getElement(), 'ip' : s.getElement() } for _ in xrange(s.getInt())]

class GetCommIDRequestMessage(object):
    """ Message type to request a new unique communication ID.
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.ID_REQUEST
    
    def serialize(self, s, data):
        pass
    
    def deserialize(self, s):
        pass

class GetCommIDResponseMessage(object):
    """ Message type to respond to a request for a new unique communication ID.
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.ID_RESPONSE
    
    def serialize(self, s, data):
        s.addElement(data['commID'])
    
    def deserialize(self, s):
        return { 'commID' : s.getElement() }

class DelCommIDRequestMessage(object):
    """ Message type to remove a communication ID from the list of currently used IDs.
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.ID_DEL
    
    def serialize(self, s, data):
        try:
            s.addElement(data['commID'])
        except KeyError as e:
            raise SerializationError('Could not serialize message of type DelCommIDRequestMessage. Missing key: {0}'.format(e))
    
    def deserialize(self, s):
        return { 'commID' : s.getElement() }

# TODO: At the moment not used!
class LoadInfoMessage(object):
    """ Message type to provide the load balancer with the necessary information.
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.LOAD_INFO
    
    def serialize(self, s, data):
        pass
    
    def deserialize(self, data):
        pass
