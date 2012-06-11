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

# Custom imports
from Exceptions import SerializationError
import MsgDef
from SerializerUtil import Serializer, Deserializer

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

    def serialize(self, manager):
        """ Serialize the message.

            The fields msgType, dest and depending on msgType content has to be
            set before this method is called.
            
            @param manager:     CommManager which has all ContentSerializer registered.
            @type  manager:     CommManager
            
            @return:    Message FIFO containing serialized message.
            @rtype:     MessageFIFO
            
            @raise:     SerializationError
        """
        if self.msgType is None:
            raise SerializationError('The message is missing the message type.')

        if self.dest is None:
            raise SerializationError('The message is missing the destination.')

        if len(self.dest) != MsgDef.ADDRESS_LENGTH:
            raise SerializationError('The destination address is not valid.')
        
        serializer = manager.getSerializer(self.msgType)
        
        if not serializer:
            raise SerializationError('The message can not be serialized (Unknown message type: "{0}").'.format(self.msgType))
        
        s = Serializer()
        serializer.serialize(s, self.content)
        
        self.msgNumber = manager.getMessageNr()
        self.origin = manager.commID
        
        buf = s.getMsg()
        s.addHeader( len(buf) + MsgDef.HEADER_LENGTH,
                     self.dest,
                     self.origin,
                     self.msgType,
                     self.msgNumber )
        return buf

    @classmethod
    def deserialize(cls, fifo, manager):
        """ Deserialize the message.
            
            @param fifo:    Message FIFO containing the serialized message.
            @type  fifo:    MessageFIFO
            
            @param manager:     CommManager which has all ContentSerializer registered.
            @type  manager:     CommManager

            @return:    New message instance which contains the deserialized
                        message.

            @raise:     SerializationError
        """
        bufLen = len(fifo)
        
        if MsgDef.HEADER_LENGTH > bufLen:
            raise SerializationError('The received message does not contain a complete header.')
        
        s = Deserializer(fifo)
        
        msg = cls()
        msgLen, msg.dest, msg.origin, msg.msgType, msg.msgNumber = s.getHeader()

        if msgLen != bufLen:
            raise SerializationError('The received message does not match the specified length.')
        
        serializer = manager.getSerializer(msg.msgType)
        
        if not serializer:
            raise SerializationError('The message can not be serialized (Unknown message type: "{0}").'.format(msg.msgType))
        
        msg.content = serializer.deserialize(s)
        
        return msg
