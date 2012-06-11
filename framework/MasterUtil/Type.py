#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Type.py
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
    
    IDENTIFIER = MsgTypes.ID_REQUEST
    
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

class LoadInfoMessage(object):
    """ Message type to provide the load balancer with the necessary information.
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.LOAD_INFO
    
    def serialize(self, s, data):
        pass # TODO: Add
    
    def deserialize(self, data):
        pass # TODO: Add
