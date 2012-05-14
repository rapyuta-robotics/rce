#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       TypeBase.py
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
import Definition as MsgDef

class MessageTypes(object):
    """ Message Types of Reappengine Protocol:
            
            AR  Initialization request
            AI  Routing information
            AC  Connection directives
            
            CS  Start container
            CH  Stop container
            CO  Container Status
            
            EC  Create environment
            ED  Destroy environment
            
            LI  Load Information
            
            RA  Add ROS component
            RR  Remove ROS component
            RM  ROS Message
            RB  Response from ROS
            RG  Get ROS Message
    """
    __slots__ = [ 'INIT_REQUEST', 'ROUTE_INFO', 'CONNECT',
                  'CONTAINER_START', 'CONTAINER_STOP', 'CONTAINER_STATUS',
                  'ENV_CREATE', 'ENV_DESTROY',
                  'LOAD_INFO',
                  'ROS_ADD', 'ROS_REMOVE', 'ROS_MSG', 'ROS_RESPONSE', 'ROS_GET' ]
    
    INIT_REQUEST = 'AR'
    ROUTE_INFO = 'AI'
    CONNECT = 'AC'
    
    CONTAINER_START = 'CS'
    CONTAINER_STOP = 'CH'
    CONTAINER_STATUS = 'CO'
    
    ENV_CREATE = 'EC'
    ENV_DESTROY = 'ED'
    
    LOAD_INFO = 'LI'
    
    ROS_ADD = 'RA'
    ROS_REMOVE = 'RR'
    ROS_MSG = 'RM'
    ROS_RESPONSE = 'RB'
    ROS_GET = 'RG'

_NONE = '__None__'
_NONE_LEN = len(_NONE)

def serializeDict(data):
    """ Serialize the dictionary.

        @param data:    Dictionary which should be serialized.
        @type  data:    { str : str }

        @return:        Serialized dictionary.
        @rtype:         str

        @raise:         SerializationError if content is not a dictionary.
    """
    buf = StringIO()
    buf.write(MsgDef.I_STRUCT.pack(len(data)))
    
    for (key, value) in data.iteritems():
        if key is None:
            key = _NONE
        
        if value is None:
            value = _NONE
        
        if not isinstance(key, str):
            raise SerializationError('Key of dictionary is not a string.')
        
        if not isinstance(value, str):
            raise SerializationError('Value of dictionary is not serialized to a string.')
        
        buf.write(MsgDef.I_STRUCT.pack(len(key)))
        buf.write(key)
        
        buf.write(MsgDef.I_STRUCT.pack(len(value)))
        buf.write(value)

    return buf.getvalue()

def deserializeDict(data):
    """ Deserialize the data.

        @param data:    Serialized data.
        @type  data:    str

        @return:        Dictionary of the message.
        @rtype:         { str : str }
    """
    msg = {}

    try:
        start = 0
        end = MsgDef.I_LEN
        size, = MsgDef.I_STRUCT.unpack(data[start:end])

        for _ in xrange(size):
            start = end
            end += MsgDef.I_LEN
            length, = MsgDef.I_STRUCT.unpack(data[start:end])
            start = end
            end += length
            key = data[start:end]
            
            if length == _NONE_LEN and key == _NONE:
                key = None
            
            start = end
            end += MsgDef.I_LEN
            length, = MsgDef.I_STRUCT.unpack(data[start:end])
            start = end
            end += length
            value = data[start:end]
            
            if length == _NONE_LEN and value == _NONE:
                value = None
            
            msg[key] = value
    except StructError as e:
        raise SerializationError('Could not deserialize Dictionary: {0}'.format(e))
    
    return msg

def serializeList(data):
    """ Serialize the list.

        @param data:    List which should be serialized.
        @type  data:    [ ]

        @return:        Serialized list.
        @rtype:         str

        @raise:         SerializationError if content is not a list.
    """
    buf = StringIO()
    buf.write(MsgDef.I_STRUCT.pack(len(data)))

    for ele in data:
        if not isinstance(ele, str):
            raise SerializationError('List element is not serialized to a string.')
        
        buf.write(MsgDef.I_STRUCT.pack(len(ele)))
        buf.write(ele)

    return buf.getvalue()

def deserializeList(data):
    """ Deserialize the data.

        @param data:    Serialized data.
        @type  data:    str

        @return:        List of the message.
        @rtype:         [ ]
        
        @raise:     SerializationError if the format does not match that of a
                    serialized list.
    """
    data = []

    try:
        start = 0
        end = MsgDef.I_LEN
        size, = MsgDef.I_STRUCT.unpack(data[start:end])

        for _ in xrange(size):
            start = end
            end += MsgDef.I_LEN
            length, = MsgDef.I_STRUCT.unpack(data[start:end])
            start = end
            end += length
            data.append(data[start:end])
    except StructError as e:
        raise SerializationError('Could not deserialize List: {0}'.format(e))
    
    return data

class ContentBase(object):
    """ Base class which declares the necessary methods which all content subclasses
        have to overwrite.
        
        The Content classes will be instantiated once and will be used for all received
        and sent messages. 
    """
    IDENTIFIER = None
    
    def serialize(self, data):
        """ Serialize the data and return it.
            
            @raise:     SerializationError
        """
    
    def deserialize(self, data):
        """ Deserialize the data and return it.
            
            @raise:     SerializationError
        """
