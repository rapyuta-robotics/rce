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

# Python specific imports
from struct import error as StructError

try:
    from cStringIO import StringIO
except ImportError:
    from StringIO import StringIO

# Custom imports
from Exceptions import SerializationError
from Comm.Message.Interfaces import IContentSerializer #@UnresolvedImport
from Comm.Message.SerializerUtil import serializeList, deserializeList
from Comm.Message import MsgDef
from Comm.Message import MsgTypes
from Serializer import serialize as rosSerialize, deserialize as rosDeserialize

class ROSAddNodeMessage(object):
    """ Message type to add a/multiple node(s).
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.ROS_ADD
    
    def serialize(self, data):
        if not isinstance(data, list):
            raise SerializationError('Content of the message ROSAddNode has to be a list.')
        
        data = map(rosSerialize, data)
        return serializeList(data)
    
    def deserialize(self, data):
        data = deserializeList(data)
        return map(rosDeserialize, data)

class ROSRemoveNodeMessage(object):
    """ Message type to remove a/multiple node(s).
        
        Should be a list of ROSUtil.Node
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.ROS_REMOVE
    
    def serialize(self, data):
        if not isinstance(data, list):
            raise SerializationError('Content of the message ROSRemoveNode has to be a list.')
        
        return serializeList(data)
    
    def deserialize(self, data):
        return map(rosDeserialize, data)

class ROSMsgMessage(object):
    """ Message type for a single ROS message.
        
        The fields are:
            msg     Serialized ROS Message
            name    Interface name which should be used to send the message
                    or an empty string if the message was received
            uid     Unique ID to identify the message
            push    Flag to indicate whether the response message, if it exists,
                    should be pushed back or stored for polling
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.ROS_MSG
    
    def serialize(self, data):
        if not isinstance(data, dict):
            raise SerializationError('Content of the message ROSMessage has to be a dictionary.')
        
        buf = StringIO()
        
        try:
            buf.write(MsgDef.I_STRUCT.pack(len(data['msg'])))
            buf.write(data['msg'])
            
            buf.write(MsgDef.I_STRUCT.pack(len(data['name'])))
            buf.write(data['name'])
            
            buf.write(MsgDef.I_STRUCT.pack(len(data['uid'])))
            buf.write(data['uid'])
            
            if data['push']:
                buf.write(1)
            else:
                buf.write(0)
        except KeyError as e:
            raise SerializationError('Could not serialize message of type ROSMessage: {0}'.format(e))
        
        return buf.getvalue()
    
    def deserialize(self, data):
        msg = {}
        
        try:
            start = 0
            end = MsgDef.I_LEN
            length, = MsgDef.I_STRUCT.unpack(data[start:end])
            start = end
            end += length
            msg['msg'] = data[start:end]
            
            start = end
            end += MsgDef.I_LEN
            length, = MsgDef.I_STRUCT.unpack(data[start:end])
            start = end
            end += length
            msg['name'] = data[start:end]
            
            start = end
            end += MsgDef.I_LEN
            length, = MsgDef.I_STRUCT.unpack(data[start:end])
            start = end
            end += length
            msg['uid'] = data[start:end]
            
            start = end
            end += 1
            msg['push'] = bool(data[start:end])
        except StructError as e:
            raise SerializationError('Could not deserialize message of type ROSMessage: {0}'.format(e))
        
        return msg

class ROSResponseMessage(object):
    """ Message type to send back an ID which can be used to retrieve a result later.
        
        The fields are:
            msg     String which should be sent; if no error occurred it should contain
                    the ID and if an error occurred it should contain the error message
            error   Flag to indicate whether msg contains an error message or not
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.ROS_RESPONSE
    
    def serialize(self, data):
        if not isinstance(data, dict):
            raise SerializationError('Content of the message ROSResponse has to be a dictionary.')
        
        buf = StringIO()
        
        try:
            buf.write(MsgDef.I_STRUCT.pack(len(data['msg'])))
            buf.write(data['msg'])
            
            if data['error']:
                buf.write(1)
            else:
                buf.write(0)
        except KeyError as e:
            raise SerializationError('Could not serialize message of type ROSResponse: {0}'.format(e))
        
        return buf.getvalue()
    
    def deserialize(self, data):
        msg = {}
        
        try:
            start = 0
            end = MsgDef.I_LEN
            length, = MsgDef.I_STRUCT.unpack(data[start:end])
            start = end
            end += length
            msg['msg'] = data[start:end]
            
            start = end
            end += 1
            msg['error'] = bool(data[start:end])
        except StructError as e:
            raise SerializationError('Could not deserialize message of type ROSResponse: {0}'.format(e))
        
        return msg

class ROSGetMessage(object):
    """ Message type to request a result using an ID.
        
        The fields are:
            name    Interface name from where the message should be retrieved
            uid     ID which identifies the message which should be retrieved
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.ROS_GET
    
    def serialize(self, data):
        if not isinstance(data, dict):
            raise SerializationError('Content of the message ROSResponse has to be a dictionary.')
        
        buf = StringIO()
        
        try:
            buf.write(MsgDef.I_STRUCT.pack(len(data['name'])))
            buf.write(data['name'])
            
            buf.write(MsgDef.I_STRUCT.pack(len(data['uid'])))
            buf.write(data['uid'])
        except KeyError as e:
            raise SerializationError('Could not serialize message of type ROSGet: {0}'.format(e))
        
        return buf.getvalue()
    
    def deserialize(self, data):
        msg = {}
        
        try:
            start = 0
            end = MsgDef.I_LEN
            length, = MsgDef.I_STRUCT.unpack(data[start:end])
            start = end
            end += length
            msg['name'] = data[start:end]
            
            start = 0
            end = MsgDef.I_LEN
            length, = MsgDef.I_STRUCT.unpack(data[start:end])
            start = end
            end += length
            msg['uid'] = data[start:end]
        except StructError as e:
            raise SerializationError('Could not deserialize message of type ROSGet: {0}'.format(e))
        
        return msg
