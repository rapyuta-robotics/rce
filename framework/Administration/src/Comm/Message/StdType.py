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

# Python specific imports
from struct import error as StructError

try:
    from cStringIO import StringIO
except ImportError:
    from StringIO import StringIO

# Custom imports
from Exceptions import SerializationError
from TypeBase import ContentBase, serializeDict, deserializeDict, serializeList, deserializeList, MessageTypes as MsgTypes
import Definition as MsgDef

class InitMessage(ContentBase):
    """ Message type to initialize the communication.
        
        The fields are:
            dest    CommID of destination
            origin  CommID of origin
            key     Key to authenticate the node
    """
    IDENTIFIER = MsgTypes.INIT_REQUEST
    
    def serialize(self, data):
        buf = StringIO()
        
        try:
            buf.write(MsgDef.I_STRUCT.pack(len(data['dest'])))
            buf.write(data['dest'])
            
            buf.write(MsgDef.I_STRUCT.pack(len(data['origin'])))
            buf.write(data['origin'])
            
            buf.write(MsgDef.I_STRUCT.pack(len(data['key'])))
            buf.write(data['key'])
        except KeyError as e:
            raise SerializationError('Could not serialize message of type InitMessage: {0}'.format(e))
        
        return buf.getvalue()
    
    def deserialize(self, data):
        msg = {}
        
        try:
            start = 0
            end = MsgDef.I_LEN
            length, = MsgDef.I_STRUCT.unpack(data[start:end])
            start = end
            end += length
            msg['dest'] = data[start:end]
            
            start = end
            end += MsgDef.I_LEN
            length, = MsgDef.I_STRUCT.unpack(data[start:end])
            start = end
            end += length
            msg['origin'] = data[start:end]
            
            start = end
            end += MsgDef.I_LEN
            length, = MsgDef.I_STRUCT.unpack(data[start:end])
            start = end
            end += length
            msg['key'] = data[start:end]
        except StructError as e:
            raise SerializationError('Could not deserialize message of type InitMessage: {0}'.format(e))
        
        return msg

class RouteMessage(ContentBase):
    """ Message type to provide other nodes with routing information.
    """
    IDENTIFIER = MsgTypes.ROUTE_INFO
    
    def serialize(self, data):
        if not isinstance(data, dict):
            raise SerializationError('Content of the message has to be a dictionary.')
        
        return serializeDict(data)
    
    def deserialize(self, data):
        return deserializeDict(data)

class ConnectDirectiveMessage(ContentBase):
    """ Message type to provide node with connection directives.
    """
    IDENTIFIER = MsgTypes.CONNECT
    
    def serialize(self, data):
        satellites = []
        
        try:
            for element in data:
                buf = StringIO()
                
                buf.write(MsgDef.I_STRUCT.pack(len(element['ip'])))
                buf.write(element['ip'])
                
                buf.write(MsgDef.I_STRUCT.pack(len(element['port'])))
                buf.write(element['port'])
                
                buf.write(MsgDef.I_STRUCT.pack(len(element['commID'])))
                buf.write(element['commID'])
                
                satellites.append(buf.getvalue())
        except KeyError as e:
            raise SerializationError('Could not serialize message of type ConnectDirective: {0}'.format(e))
        
        return serializeList(satellites)
    
    def deserialize(self, data):
        satellites = []
        
        try:
            for element in deserializeList(data):
                msg = {}
                
                start = 0
                end = MsgDef.I_LEN
                length, = MsgDef.I_STRUCT.unpack(element[start:end])
                start = end
                end += length
                msg['ip'] = element[start:end]
                
                start = end
                end += MsgDef.I_LEN
                length, = MsgDef.I_STRUCT.unpack(element[start:end])
                start = end
                end += length
                msg['port'] = element[start:end]
                
                start = end
                end += MsgDef.I_LEN
                length, = MsgDef.I_STRUCT.unpack(element[start:end])
                start = end
                end += length
                msg['commID'] = element[start:end]
                
                satellites.append(msg)
        except StructError as e:
            raise SerializationError('Could not deserialize message of type ConnectDirective: {0}'.format(e))
        
        return satellites

class LoadInfoMessage(ContentBase):
    """ Message type to provide the load balancer with the necessary information.
    """
    IDENTIFIER = MsgTypes.LOAD_INFO
    
    def serialize(self, data):
        pass # TODO: Add
    
    def deserialize(self, data):
        pass # TODO: Add
