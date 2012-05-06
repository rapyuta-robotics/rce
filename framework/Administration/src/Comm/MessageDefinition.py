#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       MessageDefintion.py
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
import string
from struct import Struct, error as StructError

try:
    from cStringIO import StringIO
except ImportError:
    from StringIO import StringIO

# Custom imports
import settings
from Exceptions import SerializationError
from ROSUtil.Serializer import serialize as rosAddSerialize, deserialize as rosAddDeserialize

class MessageDefinition(object):
    """ Message Format of Reappengine Protocol:

            Message Length:                 int     (32 bit)
            Destination Address:            str     (6 chars/bytes)
            Origin Address:                 str     (6 chars/bytes)
            Message Type:                   str     (2 chars/bytes)
            Message Number:                 int     (32 bit)
            Content (JSON encoded string):  str     (variable length)
    """
    __slots__ = ['CHUNK_SIZE', 'I_STRUCT', 'I_LEN', 'MAX_INT', 'MAX_LENGTH',
                 'ADDRESS_LENGTH', 'MSG_TYPE_LENGTH', 'POS_DEST', 'POS_ORIGIN',
                 'POS_MSG_TYPE', 'POS_MSG_NUMBER', 'HEADER_LENGTH', 'NO_AUTH_ADDR']

    # Chunk size into which the message is split
    CHUNK_SIZE = 8192

    # Constants which are used to (de-)serialize integers
    I_STRUCT = Struct('!I')
    I_LEN = I_STRUCT.size
    MAX_INT = 2 ** (I_LEN * 8) - 1

    # To prevent overly large messages
    # Absolute maximum is MAX_INT
    # else the message length header field has an overflow
    MAX_LENGTH = min(settings.MAX_LENGTH_MESSAGE, MAX_INT)

    # Necessary constants for the message
    ADDRESS_LENGTH = 6
    MSG_TYPE_LENGTH = 2

    POS_DEST = I_LEN
    POS_ORIGIN = POS_DEST + ADDRESS_LENGTH
    POS_MSG_TYPE = POS_ORIGIN + ADDRESS_LENGTH
    POS_MSG_NUMBER = POS_MSG_TYPE + MSG_TYPE_LENGTH
    HEADER_LENGTH = POS_MSG_NUMBER + I_LEN

    # Special address which is used to signal an unauthenticated connection
    NO_AUTH_ADDR = '-' * ADDRESS_LENGTH

def _nop(data):
    """ No Operation Performed, i.e. return input unchanged.
    """
    return data

def _listSerializer(serializer, deserializer):
    """ Function to create a function tuple (Serializer, Deserializer)
        which expects a list to serialize and deserialize. Each list
        element is serialized using the function 'serializer' and
        deserialized using the function 'deserializer'.
    """
    def serialize(data):
        """ Serialize the content.

            @param data:    Content which should be serialized.
            @type  data:    [ ]

            @return:        Serialized data.
            @rtype:         str

            @raise:         SerializationError if content is not a list.
        """
        if not isinstance(data, list):
            raise SerializationError('Content of a ROS message has to be a list.')

        buf = StringIO()
        buf.write(MessageDefinition._I_STRUCT.pack(len(data)))

        for ele in data:
            ele = serializer(ele)
            buf.write(MessageDefinition._I_STRUCT.pack(len(data)))
            buf.write(ele)

        return buf.getvalue()

    def deserialize(data):
        """ Deserialize the content.

            @param data:    Serialized data.
            @type  data:    [ ]

            @return:        Content of the message.
            @rtype:         str
        """
        data = []

        try:
            start = 0
            end = MessageDefinition.INT_LENGTH
            size, = MessageDefinition._STRUCT_I.unpack(data[start:end])

            for _ in xrange(size):
                start = end
                end += MessageDefinition.INT_LENGTH
                length, = MessageDefinition._STRUCT_I.unpack(data[start:end])
                start = end
                end += length
                data.append(deserialize(data[start:end]))
        except StructError as e:
            raise SerializationError('Could not deserialize List: {0}'.format(e))

    return (serialize, deserialize)

class MessageTypesDefinition(object):
    """ Message Types of Reappengine Protocol:

            RA  Add ROS component
            RR  Remove ROS component

        MAP:    Contains as values a tuple with the functions
                (serializer, deserializer). This map is used to identify
                whether a message contains a valid message type or not.
    """
    __slots__ = ['ROS_ADD', 'ROS_REMOVE', 'MAP']

    ROS_ADD = 'RA'
    ROS_REMOVE = 'RR'

    MAP = { ROS_ADD    : _listSerializer(rosAddSerialize, rosAddDeserialize),
            ROS_REMOVE : _listSerializer(_nop, _nop) }

def validateAddress(addr):
    """ Validate the given address.
    """
    if len(addr) != MessageDefinition.ADDRESS_LENGTH:
        return False
    
    for c in addr:
        if c not in string.ascii_uppercase:
            return False
    
    return True
