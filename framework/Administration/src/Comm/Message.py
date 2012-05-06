#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Message.py
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

# twisted specific imports
from zope.interface import implements
from twisted.internet.interfaces import IPushProducer, IConsumer

# Python specific imports
try:
    from cStringIO import StringIO
except ImportError:
    from StringIO import StringIO

# Custom imports
from Exceptions import InternalError, SerializationError
from MessageDefinition import MessageDefinition as MsgDef, MessageTypesDefinition as MsgType

class DestinationError(Exception):
    """ This class is used to signal a problem with the destination
        of a message.
    """
    pass

class MessageLengthError(Exception):
    """ This class is used to signal a message which is too long.
    """
    pass

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

            @param manager:     Manager which is used to get the message number
                                and the origin of the message.
            @type  manager:     ReappengineManager
        """
        if self.msgType is None:
            raise SerializationError('The message is missing the message type.')

        if self.dest is None:
            raise SerializationError('The message is missing the destination.')

        if not self.msgType in MsgType.MAP:
            raise SerializationError('The message has an invalid message type.')

        if len(self.dest) != MsgDef.ADDRESS_LENGTH:
            raise SerializationError('The destination address is not valid.')

        self.msgNumber = manager.getMessageNr()
        self.origin = manager.getCommID()

        content = MsgType.MAP[self.msgType][0](self.content)

        buf = StringIO()
        buf.write(MsgDef.I_STRUCT.pack(len(self.content) + MsgDef.HEADER_LENGTH))
        buf.write(self.dest)
        buf.write(self.origin)
        buf.write(self.msgType)
        buf.write(MsgDef.I_STRUCT.pack(len(self.msgNumber)))
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
        msg.msgNumber = data[MsgDef.POS_MSG_NUMBER:MsgDef.POS_MSG_NUMBER + MsgDef.I_LEN]

        if msg.msgType not in MsgType.MAP:
            raise SerializationError('The message has an invalid message type.')

        msg.content = MsgType.MAP[msg.msgType][1](data[MsgDef.HEADER_LENGTH:])

        return msg

class MessageSender(object):
    """ This class is used to provide a startpoint for the communication.
        It also takes care of the serialization of the data.
    """
    implements(IPushProducer)

    def __init__(self, manager, msg):
        """ Initialize the Message object.

            @param manager:     Manager which has the node specific handling
                                for Protocol callbacks.
            @type  manager:     Class which implements IReappengineManager

            @param msg:     Message which should be sent.
            @type  msg:     Message

            @raise:     InternalError, MessageLengthError
        """
        self._manager = manager
        self._consumer = None

        self._sendBuf = msg.serialize(manager)
        self._pos = 0
        self.paused = False

    def pauseProducing(self):
        """ Method for the consumer to signal that he can't accept (more)
            data at the moment.
        """
        self.paused = True

    def resumeProducing(self):
        """ Method for the consumer to signal that he can accept (again)
            data.
        """
        self.paused = False
        self._send()

    def stopProducing(self):
        """ Method for the consumer to signal that the producer should
            stop sending data.
        """
        self.paused = True
        self._consumer.unregisterProducer()

    def _send(self):
        """ This method sends the data to the specified consumer.
        """
        msgLen = len(self._sendBuf)

        if msgLen < MsgDef.HEADER_LENGTH:
            self._consumer.unregisterProducer()
            raise MessageLengthError('The message does not contain a valid message.')

        while not self.paused:
            self._consumer.write(self._sendBuf[self._pos:self._pos + MsgDef.CHUNK_SIZE])
            self._pos += MsgDef.CHUNK_SIZE

            if self._pos > msgLen:
                self._consumer.unregisterProducer()
                break

    def send(self, consumer):
        """ Send the previously parsed message.
        """
        if self._consumer:
            raise InternalError('This message sender is already sending a message.')

        self._consumer = consumer
        self._consumer.registerProducer(self, True)
        self._send()

class MessageReceiver(object):
    """ This class is used to provide an endpoint for the communication.
        It also takes care of the deserialization of the data.
    """
    implements(IConsumer)

    def __init__(self, manager, conn, authenticated):
        """ Initialize the Message Receiver object.

            @param manager:     Manager which has the node specific handling
                                for Protocol callbacks.
            @type  manager:     Class which implements IReappengineManager

            @param conn:    Connection from with the message originated.
                            (This is only used for the authentication process.)
            @type  conn:    ReappengineProtocol

            @param authenticated:   Flag to indicate whether the message was
                                    received from an authenticated source.
            @type  authenticated:   bool

            @raise:     InternalError, MessageLengthError
        """
        self._manager = manager
        self._conn = conn
        self._authenticated = authenticated

        self._recvBuf = ''

    def registerProducer(self, producer, streaming):
        """ Register a producer which be used to send a message. The
            producer should be a push producer, i.e. streaming should
            be true. If this is not the case an exception is raised.
        """
        if not streaming:
            raise NotImplementedError('Pull Producer are not supported; use Push Producer instead.')

        if self._conn != producer:
            raise InternalError('Producer does not match the specified connection.')

        if self._recvBuf:
            raise InternalError('Can not use the same Message Receiver for more than one message.')

    def write(self, data):
        """ Method which is used by the producer to send data of this
            protocol instance.
        """
        self._recvBuf += data

    def unregisterProducer(self):
        """ Method which is used by the producer to signal that he has
            finished sending data.
        """
        self._manager.processMessage(Message.deserialize(self._recvBuf), self._conn, self._authenticated)

class MessageForwarder(object):
    """ This class is used to forward a message.
    """
    implements(IPushProducer, IConsumer)
    
    def __init__(self):
        """ Forward a received message to the specified node.
        """
        self._producer = None
        self._consumer = None
        
        self._buf = ''
        self._sent = 0
        self._recv = 0
        
        self.paused = False
        self._connected = False
        self._completed = False
    
    ##################################################
    ### Producer
    
    def pauseProducing(self):
        """ Method for the consumer to signal that he can't accept (more)
            data at the moment.
        """
        self.paused = True

    def resumeProducing(self):
        """ Method for the consumer to signal that he can accept (again)
            data.
        """
        self.paused = False
        self._send()

    def stopProducing(self):
        """ Method for the consumer to signal that the producer should
            stop sending data.
        """
        self.paused = True
        self._completed = True
        self._consumer.unregisterProducer()
        
        if self._producer:
            self._producer.stopProducing()
    
    def _send(self):
        """ This method sends the data to the specified consumer.
        """
        while not self.paused and self._buf:
            sendSize = min(len(self._buf), MsgDef.CHUNK_SIZE)
            
            self._consumer.write(self._buf[:sendSize])
            self._buf = self._buf[sendSize:]
            self._sent += sendSize
        
        if self._completed and (self._sent == self._recv or not self._buf):
            self._consumer.unregisterProducer()

    def send(self, consumer):
        """ Send the previously parsed message.
        """
        if self._consumer:
            raise InternalError('This message sender is already sending a message.')
        
        self._consumer = consumer
        self._consumer.registerProducer(self, True)
        self._send()
    
    ##################################################
    ### Consumer
    
    def registerProducer(self, producer, streaming):
        """ Register a producer which be used to send a message. The
            producer should be a push producer, i.e. streaming should
            be true. If this is not the case an exception is raised.
        """
        if not streaming:
            raise NotImplementedError('Pull Producer are not supported; use Push Producer instead.')
        
        self._producer = producer

    def write(self, data):
        """ Method which is used by the producer to send data of this
            protocol instance.
        """
        self._buf += data
        self._recv += len(data)
        
        if self._consumer:
            self._send()

    def unregisterProducer(self):
        """ Method which is used by the producer to signal that he has
            finished sending data.
        """
        self._completed = True
    
    ##################################################

class Sink(object):
    """ This class provides a consumer which can be used to
        discard the received message (equivalent to /dev/null).
    """
    implements(IConsumer)

    def registerProducer(self, producer, streaming):
        """ Register a producer which be used to send a message. The
            producer should be a push producer, i.e. streaming should
            be true. If this is not the case an exception is raised.
        """
        if not streaming:
            raise NotImplementedError('Pull Producer are not supported; use Push Producer instead.')

    def write(self, data):
        """ Method which is used by the producer to send data of this
            protocol instance.
        """
        pass

    def unregisterProducer(self):
        """ Method which is used by the producer to signal that he has
            finished sending data.
        """
        pass
