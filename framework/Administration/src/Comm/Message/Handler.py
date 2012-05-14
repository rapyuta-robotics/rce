#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Handler.py
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
from twisted.python import log
from twisted.internet.interfaces import IPushProducer, IConsumer

# Custom imports
from Exceptions import InternalError, SerializationError
import Definition as MsgDef
from Base import MessageLengthError, Message

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
                log.msg('Message Sender: {0} bytes sent'.format(msgLen))
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

    # TODO: Check if conn and authenticated is still necessary
    def __init__(self, manager, authenticated):
        """ Initialize the Message Receiver object.

            @param manager:     Manager which has the node specific handling
                                for Protocol callbacks.
            @type  manager:     Class which implements IReappengineManager

            @param authenticated:   Flag to indicate whether the message was
                                    received from an authenticated source.
            @type  authenticated:   bool

            @raise:     InternalError, MessageLengthError
        """
        self._manager = manager
        self._authenticated = authenticated
        self._conn = None

        self._recvBuf = ''

    def registerProducer(self, producer, streaming):
        """ Register a producer which be used to send a message. The
            producer should be a push producer, i.e. streaming should
            be true. If this is not the case an exception is raised.
        """
        if not streaming:
            raise NotImplementedError('Pull Producer are not supported; use Push Producer instead.')

        if self._recvBuf:
            raise InternalError('Can not use the same Message Receiver for more than one message.')
        
        self._conn = producer

    def write(self, data):
        """ Method which is used by the producer to send data of this
            protocol instance.
        """
        self._recvBuf += data

    def unregisterProducer(self):
        """ Method which is used by the producer to signal that he has
            finished sending data.
        """
        log.msg('Message Receiver: {0} bytes received'.format(len(self._recvBuf)))
        
        try:
            if self._authenticated:
                self._manager.messageReceived(Message.deserialize(self._recvBuf))
            else:
                self._conn.receivedInitMessage(Message.deserialize(self._recvBuf))
        except SerializationError as e:
            log.msg('Could not deserialize message: {0}'.format(e))

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
            log.msg('Message Forwarder: {0} of {1} bytes forwarded'.format(self._sent, self._recv))
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
