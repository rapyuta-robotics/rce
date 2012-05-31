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
from Comm.Message.Interfaces import IContentSerializer #@UnresolvedImport
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
                raise SerializationError('Could not serialize message of type ConnectDirective: {0}'.format(e))
    
    def deserialize(self, s):
        return [{ 'commID' : s.getElement(), 'ip' : s.getElement() } for _ in xrange(s.getInt())]

class LoadInfoMessage(object):
    """ Message type to provide the load balancer with the necessary information.
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.LOAD_INFO
    
    def serialize(self, s, data):
        pass # TODO: Add
    
    def deserialize(self, data):
        pass # TODO: Add
