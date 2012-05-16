#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Base.py
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

try:
    from cStringIO import StringIO
except ImportError:
    from StringIO import StringIO

# Custom imports
from Exceptions import InternalError, SerializationError
import MsgDef
import Serializer

def validateAddress(addr):
    """ Validate the given address.
    """
    if len(addr) != MsgDef.ADDRESS_LENGTH:
        return False
    
    for c in addr:
        if c not in string.ascii_uppercase:
            return False
    
    return True

class Message(object):
    """ This class represents a message which should be sent from one
        communication node to another.
    """
    __slots__ = ['msgType', 'msgNumber', 'dest', 'origin', 'content']

    def __init__(self):
        """ Initialize the message fields with None.
        """
        self.msgType = None
        self.msgNumber = None
        self.dest = None
        self.origin = None
        self.content = None

    def serialize(self, msgNrCallback, origin):
        """ Serialize the message.

            The fields msgType, dest and depending on msgType content has to be
            set before this method is called.

            @param msgNrCallback:   Callback function which can be used to get a new message
                                    number. The function should take no arguments and return
                                    an integer.
            @type  msgNrCallback:   Callable
            
            @param origin:  CommID of this node.
            @type  origin:  str
            
            @raise:     InternalError, SerializationError
        """
        if not callable(msgNrCallback):
            raise InternalError('Message number callback is not a callable object.')
        
        if self.msgType is None:
            raise SerializationError('The message is missing the message type.')

        if self.dest is None:
            raise SerializationError('The message is missing the destination.')
        
        if self.msgType not in Serializer.SERIALIZERS:
            raise SerializationError('The message has an invalid message type.')

        if len(self.dest) != MsgDef.ADDRESS_LENGTH:
            raise SerializationError('The destination address is not valid.')

        self.msgNumber = msgNrCallback()
        self.origin = origin
        
        content = Serializer.SERIALIZERS[self.msgType].serialize(self.content)
        
        buf = StringIO()
        buf.write(MsgDef.I_STRUCT.pack(len(content) + MsgDef.HEADER_LENGTH))
        buf.write(self.dest)
        buf.write(self.origin)
        buf.write(self.msgType)
        buf.write(MsgDef.I_STRUCT.pack(self.msgNumber))
        buf.write(content)
        return buf.getvalue()

    @classmethod
    def deserialize(cls, data):
        """ Deserialize the message.

            @return:    New message instance which contains the deserialized
                        message.

            @raise:     SerializationError
        """
        if MsgDef.HEADER_LENGTH > len(data):
            raise SerializationError('The received message does not contain a complete header.')

        msgLen, = MsgDef.I_STRUCT.unpack(data[:MsgDef.I_LEN])

        if msgLen != len(data):
            raise SerializationError('The received message does not match the specified length.')

        msg = cls()

        msg.dest = data[MsgDef.POS_DEST:MsgDef.POS_DEST + MsgDef.ADDRESS_LENGTH]
        msg.origin = data[MsgDef.POS_ORIGIN:MsgDef.POS_ORIGIN + MsgDef.ADDRESS_LENGTH]
        msg.msgType = data[MsgDef.POS_MSG_TYPE:MsgDef.POS_MSG_TYPE + MsgDef.MSG_TYPE_LENGTH]
        msg.msgNumber = MsgDef.I_STRUCT.unpack(data[MsgDef.POS_MSG_NUMBER:MsgDef.POS_MSG_NUMBER + MsgDef.I_LEN])
        
        if msg.msgType not in Serializer.SERIALIZERS:
            raise SerializationError('The message has an invalid message type.')
        
        msg.content = Serializer.SERIALIZERS[msg.msgType].deserialize(data[MsgDef.HEADER_LENGTH:])
        
        return msg
