#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       StdType.py
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
from Interfaces import IContentSerializer #@UnresolvedImport
import MsgTypes

class InitMessage(object):
    """ Message type to initialize the communication.
        
        The fields are:
            dest    CommID of destination
            origin  CommID of origin
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.INIT_REQUEST
    
    def serialize(self, s, data):
        try:
            s.addElement(data['remoteID'])
        except KeyError as e:
            raise SerializationError('Could not serialize message of type InitMessage: {0}'.format(e))
    
    def deserialize(self, s):
        return { 'remoteID' : s.getElement() }

class RouteMessage(object):
    """ Message type to provide other nodes with routing information.
        
        Message format is a list of 2-tuples (DestID, Add (True) / Remove (False) flag)
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.ROUTE_INFO
    
    def serialize(self, s, data):
        if not isinstance(data, list):
            raise SerializationError('Content of the message has to be a list.')
        
        s.addInt(len(data))
        
        for element in data:
            if len(element) != 2:
                raise SerializationError('List element is not a 2-tuple.')
            
            try:
                s.addElement(element[0])
                s.addBool(element[1])
            except IndexError as e:
                raise SerializationError('Could not serialize element: {0}'.format(e))
    
    def deserialize(self, s):
        return [(s.getElement(), s.getBool()) for _ in xrange(s.getInt())]
