#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       CommManager.py
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
from zope.interface.verify import verifyObject
from twisted.python import log

# Custom imports
from Exceptions import InternalError, SerializationError
from Message.Interfaces import IContentSerializer, IMessageProcessor
from Message import MsgDef
from Message.Base import Message
from Message.Handler import MessageReceiver, MessageForwarder, MessageSender, Sink
from Message.StdType import InitMessage, RouteMessage
from Message.StdProcessor import RouteProcessor
from Router import Router

class CommManager(object):
    """ Class which is responsible for handling the communication.
    """
    def __init__(self, reactor, commID):
        """ Initialize the CommManager.

            @param reactor:     Reference to used reactor (from twisted).
            @type  reactor:     reactor

            @param commID:  CommID of this node which can be used by other
                            nodes to identify this node.
            @type  commID:  str
        """
        # Communication ID of this node
        self._commID = commID
        
        # twisted reactor which is used in this manager
        self._reactor = reactor
        
        # Reference to Router
        self._router = Router()
        
        # Message number counter
        self._msgNr = 0
        
        # Content serializers
        self._contentSerializer = {}
        self.registerContentSerializers([ InitMessage(),
                                          RouteMessage() ])
        
        # Message processors
        self._processors = {}
        self.registerMessageProcessors([ RouteProcessor(self) ])
    
    @property
    def commID(self):
        """ Communication ID of this node. """
        return self._commID
    
    @commID.setter
    def commID(self, value):
        if self._commID != MsgDef.NEED_ADDR:
            raise InternalError('Can not assign a new CommID to node which already has a CommID.')
        
        self._commID = value
    
    @property
    def reactor(self):
        """ Reactor instance of this node. """
        return self._reactor
    
    @property
    def router(self):
        """ Router instance of this node. """
        return self._router
    
    def registerContentSerializers(self, serializers):
        """ Method to register a content serializer which are used for serializing/deserializing
            the messages. If there already exists a serializer for the same message type the
            old serializer is dropped.
            
            @param serializers:     Content serializers which should be registered.
            @type  serializers:     [ IContentSerializer ]
        """
        for serializer in serializers:
            if not verifyObject(IContentSerializer, serializer):
                raise InternalError('Can only add serializers which implement IContentSerializer.')
            
            self._contentSerializer[serializer.IDENTIFER] = serializer
    
    def registerMessageProcessors(self, processors):
        """ Method to register a message processor which are used for the incoming messages.
            If there already exists a message processor for the same message type the old
            message processor is dropped.
                                
            @param processors:  Processor which should be registered.
            @type  processors:  [ IMessageProcessor ]
        """
        for processor in processors:
            if not verifyObject(IMessageProcessor, processor):
                raise InternalError('Can only add processors which implement IMessageProcessor.')
            
            self._processors.append[processor.IDENTIFER] = processor

    def getNextConsumer(self, origin, dest=None, callback=None):
        """ This method is called by the protocol instances to get the next destination
            point for the received message.
            
            Either the argument 'dest' or the argument 'callback' has to be provided. If both
            or none of the two are provided an error is raised.
            If 'dest' is provided the standard message handling is used. The returned value is
            either a MessageForwarder instance or a MessageReceiver instance.
            If 'callback' is provided the returned MessageReceiver instance gets the argument
            as callback function.
            
            @param origin:  The communication ID from which the received message originated.
            @type  origin:  str
            
            @param dest:    The destination address of the received message.
            @type  dest:    str
            
            @param callback:    Custom callback which should be used for the MessageReceiver.
            @type  callback:    Callable

            @return:    The correct Message handler to where the message should be forwarded.
        """
        if callable(callback):
            if dest:
                raise InternalError('Both arguments "dest" and "callback" are supplied.')
            
            return MessageReceiver(self, callback)
        
        if not dest:
            raise InternalError('None of the arguments "dest" and "callback" are supplied.')
        
        # Check if the message is for this communication node
        if dest == self._commID or dest == MsgDef.NEIGHBOR_ADDR:
            return MessageReceiver(self, self._processMessage)
        
        # Message is not for this node; therefore, it should probably be forwarded
        
        if dest[:MsgDef.PREFIX_LENGTH_ADDR] == MsgDef.PREFIX_CONTAINER_ADDR:
            # Tried to send a message to Container Manager via this node which is not allowed for security reasons.
            log.msg('Received a message to a container manager via this node. Message is dropped.')
            return Sink()
        
        forwarder = MessageForwarder(origin)
        self._router.registerProducer(forwarder, dest)
        return forwarder
    
    def getMessageNr(self):
        """ Returns a new message number.
        """
        self._msgNr = (self._msgNr + 1) % MsgDef.MAX_INT
        return self._msgNr
    
    def getSerializer(self, msgType):
        """ Returns the ContentSerializer matching the given msgType or None if there
            was no match.
        """
        return self._contentSerializer.get(msgType, None)
    
    def _sendMessage(self, msg):
        """ Send a message.
        """
        try:
            self._router.registerProducer(MessageSender(self, msg), msg.dest)
        except SerializationError as e:
            log.msg('Message could not be sent: {0}'.format(str(e)))
    
    def sendMessage(self, msg):
        """ Send a message.

            @param msg:     Message which should be sent to destination.
            @type  msg:     Message
        """
        self._reactor.callFromThread(self._sendMessage, msg)
    
    def _processMessage(self, msg):
        """ Process the message using the registered message processors.
            
            @param msg:     Received message.
            @type  msg:     Message
        """
        if msg.msgType in self._processors:
            self._processors[msg.msgType].processMessage(msg)
        else:
            log.msg('Received a message whose type can not be handled by this CommManager.')

    def messageReceived(self, msg, processMsg):
        """ This method is called with the received still serialized message.
            
            @param msg:     Serialized message.
            @type  msg:     str

            @param processMsg:  Callback which should be used when the message
                                has been completely received and successfully
                                deserialized. The callback function should take
                                as a single argument a Message instance.
            @type  processMsg:  Callable
        """
        log.msg('Message Receiver: {0} bytes received'.format(len(self._recvBuf)))
        
        if not callable(processMsg):
            raise InternalError('Process Message callback is not a callable object.')
        
        try:
            self._processMsg(Message.deserialize(self._recvBuf, self._contentSerializer))
        except SerializationError as e:
            log.msg('Could not deserialize message: {0}'.format(e))
    
    def shutdown(self):
        """ Method which should be used when the CommManager is terminated.
            It is used to remove circular references.
        """
        self._contentSerializer = {}
        self._processors = {}
