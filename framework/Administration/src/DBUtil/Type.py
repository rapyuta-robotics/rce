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

class DBRequestMessage(object):
    """ Message type to request a database query.
        
        The fields are:
            method  Method which should be called in the DBHandler
            args    Keyworded arguments as a serialized json string
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.DB_REQUEST
    
    def serialize(self, s, data):
        try:
            s.addElement(data['id'])
            s.addElement(data['method'])
            s.addElement(data['args'])
            s.addElement(data['kw'])
        except KeyError as e:
            raise SerializationError('Could not serialize message of type DBRequestMessage: {0}'.format(e))
    
    def deserialize(self, s):
        return { 'id'     : s.getElement(),
                 'method' : s.getElement(),
                 'args'   : s.getElement(),
                 'kw'     : s.getElement() }

class DBResponseMessage(object):
    """ Message type to send back a response for a database query.
        
        The fields are:
            resp    Response of the database query as a serialized json string
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.DB_RESPONSE
    
    def serialize(self, s, data):
        try:
            s.addElement(data['id'])
            s.addBool(data['status'])
            s.addElement(data['resp'])
        except KeyError as e:
            raise SerializationError('Could not serialize message of type DBResponseMessage: {0}'.format(e))
    
    def deserialize(self, s):
        return { 'id'     : s.getElement(),
                 'status' : s.getBool(),
                 'resp'   : s.getElement() }
