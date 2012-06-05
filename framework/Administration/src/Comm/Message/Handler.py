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

# zope specific imports
from zope.interface import implements

# twisted specific imports
from twisted.python import log
from twisted.internet.interfaces import IConsumer

# Custom imports
from Exceptions import InternalError, SerializationError
from Interfaces import IReappengineProducer
import MsgDef
from Base import Message
from FIFO import MessageFIFO

class _Sink(object):
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

class _Base(object):
    """ Common Base Class.
    """
    def __init__(self, msgLen, origin, dest, buf):
        # Message length
        self._msgLen = msgLen
        
        # Addresses
        self._origin = origin
        self._dest = dest
        
        # FIFO
        self._buf = buf
    
    @property
    def origin(self):
        """ Communication ID of the originating connection of this message. """
        return self._origin
    
    @property
    def dest(self):
        """ Communication ID of the next destination of this message. """
        return self._dest

class _Receiver(_Base):
    """ Receiver for messages which implements the necessary interface such that it can
        be used by the ReappengineProtocol.
    """
    implements(IConsumer)
    
    def __init__(self, msgLen, origin, dest, buf):
        super(_Receiver, self).__init__(msgLen, origin, dest, buf)
        
        self._recv = 0
        self._producer = None
    
    def registerProducer(self, producer, streaming):
        """ Register a producer which is used to send the message. The
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
        self._buf.push(data)
        self._recv += len(data)

    def unregisterProducer(self):
        """ Method which is used by the producer to signal that he has
            finished sending data.
        """
        log.msg('Message Handler: {0} of {1} bytes received.'.format(self._recv, self._msgLen))

class _Sender(_Base):
    """ Sender for messages which implements the necessary interface such that it can
        be used by the ReappengineProtocol.
    """
    implements(IReappengineProducer)
    
    def __init__(self, msgLen, origin, dest, buf):
        super(_Sender, self).__init__(msgLen, origin, dest, buf)
        
        self._sent = 0
        self._consumer = None
        self._paused = False
        self._aborted = False
    
    @property
    def paused(self):
        """ Flag which indicates whether the producer of this message handler is paused or not. """
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
        self._aborted = True
        self._consumer.unregisterProducer()
    
    def _send(self):
        """ This method sends the data to the specified consumer.
        """
        while not self._paused and self._buf:
            data, length = self._buf.pop(MsgDef.CHUNK_SIZE)
            self._consumer.write(data)
            self._sent += length
        
        if (self._sent == self._msgLen or not self._buf) or self._aborted:
            log.msg('Message Handler: {0} of {1} bytes sent.'.format(self._sent, self._msgLen))
            self._consumer.unregisterProducer()

    def send(self, consumer):
        """ Callback from consumer to register himself and request that the Producer starts
            to send the previously parsed message.
                                
            @param consumer:    Protocol instance which should consume this message.
                                It is also possible to use any consumer which implements the
                                IConsumer interface; however, in this case this method has
                                to be called manually.
            @type  consumer:    ReappengineProtocol (or any IConsumer)
        """
        if self._consumer:
            raise InternalError('This message sender is already sending a message.')
        
        self._consumer = consumer
        self._consumer.registerProducer(self, True)
        self._send()

class _Forwarder(_Receiver, _Sender):
    """ Forwarder for messages which implements the necessary interfaces such that it can
        be used by the ReappengineProtocol.
    """
    implements(IConsumer, IReappengineProducer)
    
    def __init__(self, msgLen, origin, dest):
        """ Initialize the MessageHandler.
            
            @param msgLen:  Message length.
            @type  msgLen:  int
            
            @param origin:  Communication ID of originating connection.
            @type  origin:  str
            
            @param dest:    Communication ID of next destination.
            @type  dest:    str
        """
        super(_Forwarder, self).__init__(msgLen, origin, dest, MessageFIFO())

    def stopProducing(self):
        """ Method for the consumer to signal that the producer should
            stop sending data.
        """
        super(_Forwarder, self).stopProducing()
        
        if self._producer:
            self._producer.stopProducing()

    def write(self, data):
        """ Method which is used by the producer to send data of this
            protocol instance.
        """
        super(_Forwarder, self).write(data)
        
        if self._consumer:
            self._send()

class _EndReceiver(_Receiver):
    """ Receiver for messages with endpoint in the current node which implements
        the necessary interface such that it can be used by the ReappengineProtocol.
    """
    def __init__(self, manager, handler, msgLen, origin, dest):
        super(_EndReceiver, self).__init__(msgLen, origin, dest, MessageFIFO())
        
        self._handler = handler
        self._manager = manager
    
    def unregisterProducer(self):
        """ Method which is used by the producer to signal that he has
            finished sending data.
        """
        super(_EndReceiver, self).unregisterProducer()
        
        try:
            self._handler.processMessage(Message.deserialize(self._buf, self._manager))
        except SerializationError as e:
            log.msg('Could not deserialize message: {0}'.format(e))

def send(manager, msg):
    """ Function which is used to construct a new message handler for an outgoing message.
        
        @param manager:     CommManger which is responsible for this node.
        @type  manager:     CommManager
        
        @param msg:     Message which should be sent.
        @type  msg:     Message
    """
    try:
        buf = msg.serialize(manager)
        sender = _Sender(len(buf), manager.commID, msg.dest, buf)
    except SerializationError as e:
        log.msg('Message could not be sent: {0}'.format(str(e)))
    else:
        manager.reactor.callFromThread(manager.router.registerProducer, sender, msg.dest)

def receive(factory, msgType, msgLen, origin, dest, init):
    """ Function which is used to construct a new message handler for an incoming message.
        
        @param factory:     Factory which is responsible for the connection from which the
                            message originated.
        @type  factory:     ReappengineFactory
        
        @param msgType:     Message type.
        @type  msgType:     str
        
        @param msgLen:  Message Length
        @type  msgLen:  int
        
        @param origin:  Communication ID of the connection from which the message originated.
        @type  origin:  str
        
        @param dest:    Communication ID of message destination.
        @type  dest:    str
        
        @param init:    Protocol instance which has to be initialized.
                        If the protocol is already initialized None should be used.
        @type  init:    ReappengineProtocol / None
    """
    if msgLen > MsgDef.MAX_LENGTH:
        # TODO: What to do with message which is too long?
        #       At the moment the data is read but not saved or parsed.
        log.msg('Message is too long and will be dropped.')
        return _Sink()
    
    if factory.filterMessage(msgType):
        # Message should be filtered out
        log.msg('Message of type "{0}" has been filtered out.'.format(msgType))
        return _Sink()
    
    commManager = factory.commManager
    commID = commManager.commID
    
    if init:
        # Other side is not yet authenticated
        log.msg('Received a message in uninitialized state.')
        return _EndReceiver(commManager, init, msgLen, origin, commID)
    
    # Everything ok; get the next message consumer
    log.msg('Received a message in initialized state.')
    
    if dest == commID or dest == MsgDef.NEIGHBOR_ADDR:
        # Message is for this node
        return _EndReceiver(commManager, commManager, msgLen, origin, commID)
    
    # Message is not for this node; therefore, forward it, but first
    # check if the forwarding is allowed.
    if dest[:MsgDef.PREFIX_LENGTH_ADDR] == MsgDef.PREFIX_CONTAINER_ADDR:
        # Tried to send a message to Container Manager via this node
        # which is not allowed for security reasons.
        log.msg('Received a message to a container manager via this node. Message is dropped.')
        return _Sink()
    
    handler = _Forwarder(msgLen, origin, dest)
    commManager.router.registerProducer(handler, dest)
    return handler
