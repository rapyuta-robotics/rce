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
from twisted.internet.interfaces import IConsumer

# Custom imports
from Exceptions import InternalError, MessageLengthError
import MsgDef
from Interfaces import IReappengineProducer

class MessageSender(object):
    """ This class is used to provide a startpoint for the communication.
        It also takes care of the serialization of the data.
    """
    implements(IReappengineProducer)

    def __init__(self, manager, msg):
        """ Initialize the Message object.

            @param manager:     CommManager which has the current CommID and the
                                message number.
            @type  manager:     CommManager

            @param msg:     Message which should be sent.
            @type  msg:     Message

            @raise:     InternalError, MessageLengthError
        """
        self._origin = manager.commID
        self._consumer = None

        self._sendBuf = msg.serialize(manager)
        self._pos = 0
        self._paused = False
    
    @property
    def origin(self):
        """ Communication ID of this node. """
        return self._origin
    
    @property
    def paused(self):
        """ Flag which indicates whether the producer is paused or not. """
        return self._paused

    def pauseProducing(self):
        """ Method for the consumer to signal that he can't accept (more)
            data at the moment.
        """
        self._paused = True

    def resumeProducing(self):
        """ Method for the consumer to signal that he can accept (again)
            data.
        """
        self._paused = False
        self._send()

    def stopProducing(self):
        """ Method for the consumer to signal that the producer should
            stop sending data.
        """
        self._paused = True
        self._consumer.unregisterProducer()

    def _send(self):
        """ This method sends the data to the specified consumer.
        """
        msgLen = len(self._sendBuf)

        if msgLen < MsgDef.HEADER_LENGTH:
            self._consumer.unregisterProducer()
            raise MessageLengthError('The message does not contain a valid message.')

        while not self._paused:
            self._consumer.write(self._sendBuf[self._pos:self._pos + MsgDef.CHUNK_SIZE])
            self._pos += MsgDef.CHUNK_SIZE

            if self._pos > msgLen:
                log.msg('Message Sender: {0} bytes sent'.format(msgLen))
                self._consumer.unregisterProducer()
                break

    def send(self, consumer):
        """ Callback from consumer to register himself and request that the Producer starts
            to send the previously parsed message.
                                
            @param consumer:    Protocol instance which should consume this message.
                                It is also possible to use any consumer which implements the
                                IConsumer interface; however, in this case the method send has
                                to be called manually.
            @type  consumer:    ReappengineProtocol (or any IConsumer)
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

    def __init__(self, commManager, processMsg):
        """ Initialize the Message Receiver object.

            @param processMsg:  Callback which should be used when the message
                                has been completely received and successfully
                                deserialized. The callback function should take
                                as a single argument a Message instance.
            @type  processMsg:  Callable
        """
        self._commManager = commManager
        self._processMsg = processMsg

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

    def write(self, data):
        """ Method which is used by the producer to send data of this
            protocol instance.
        """
        self._recvBuf += data

    def unregisterProducer(self):
        """ Method which is used by the producer to signal that he has
            finished sending data.
        """
        self._commManager.messageReceived(self._recvBuf, self._processMsg)

class MessageForwarder(object):
    """ This class is used to forward a message.
    """
    implements(IReappengineProducer, IConsumer)
    
    def __init__(self, origin):
        """ Initialize the Message Forwarder.
            
            @param origin:  Communication ID of originating connection.
            @type  origin:  str
        """
        self._origin = origin
        
        self._producer = None
        self._consumer = None
        
        self._buf = ''
        self._sent = 0
        self._recv = 0
        
        self._paused = False
        self._connected = False
        self._completed = False
    
    @property
    def origin(self):
        """ Communication ID of this node. """
        return self._origin
    
    @property
    def paused(self):
        """ Flag which indicates whether the producer is paused or not. """
        return self._paused
    
    ##################################################
    ### Producer
    
    def pauseProducing(self):
        """ Method for the consumer to signal that he can't accept (more)
            data at the moment.
        """
        self._paused = True

    def resumeProducing(self):
        """ Method for the consumer to signal that he can accept (again)
            data.
        """
        self._paused = False
        self._send()

    def stopProducing(self):
        """ Method for the consumer to signal that the producer should
            stop sending data.
        """
        self._paused = True
        self._completed = True
        self._consumer.unregisterProducer()
        
        if self._producer:
            self._producer.stopProducing()
    
    def _send(self):
        """ This method sends the data to the specified consumer.
        """
        while not self._paused and self._buf:
            sendSize = min(len(self._buf), MsgDef.CHUNK_SIZE)
            
            self._consumer.write(self._buf[:sendSize])
            self._buf = self._buf[sendSize:]
            self._sent += sendSize
        
        if self._completed and (self._sent == self._recv or not self._buf):
            log.msg('Message Forwarder: {0} of {1} bytes forwarded'.format(self._sent, self._recv))
            self._consumer.unregisterProducer()

    def send(self, consumer):
        """ Callback from consumer to register himself and request that the Producer starts
            to send the previously parsed message.
                                
            @param consumer:    Protocol instance which should consume this message.
                                It is also possible to use any consumer which implements the
                                IConsumer interface; however, in this case the method send has
                                to be called manually.
            @type  consumer:    ReappengineProtocol (or any IConsumer)
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
