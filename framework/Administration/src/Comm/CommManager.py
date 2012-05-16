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
from Message.Interfaces import IMessageProcessor
from Message import MsgDef
from Message.Handler import MessageReceiver, MessageForwarder, MessageSender
from Message.StdProcessor import RouteProcessor

class CommManager(object):
    """ Class which is responsible for handling the communication.
    """
    def __init__(self, reactor, commID):
        """ Initialize the CommManager.

            @param reactor:     TODO: Add description
            @type  reactor:     reactor

            @param commID:  CommID of this node which can be used by other
                            nodes to identify this node.
            @type  commID:  str
        """
        # Communication ID of this node
        self.commID = commID
        
        # twisted reactor which is used in this manager
        self.reactor = reactor
        
        # Reference to Router
        self.router = None # TODO: Add Router instance here
        
        # Message number counter
        self._msgNr = 0
        
        # Message processors
        self._processors = []
        self.registerMessageProcessor(RouteProcessor(self))
    
    def registerMessageProcessor(self, processor):
        """ Method to register a message producer which are used for the incoming messages.
            
            @param processor:   Processor which should be registered.
            @type  processor:   IMessageProcessor
        """
        if not verifyObject(IMessageProcessor, processor):
            raise InternalError('Can only add processors which are a subclass of ProcssorBase.')
        
        self._processors.append(processor)

    def getNextConsumer(self, dest, origin):
        """ This method is called by the protocol instances to get the next destination
            point for the received message.

            @param dest:    The destination address of the received message.
            @type  dest:    str

            @param origin:  The communication ID from which the received message originated.
            @type  origin:  str

            @return:    The correct Message handler to where the message should be forwarded.
        """
        # Check if the message is for this communication node
        if dest == self.commID or dest == MsgDef.NEIGHBOR_ADDR:
            return MessageReceiver(self.messageReceived)

        forwarder = MessageForwarder(origin)
        self.router.registerProducer(forwarder, dest)
        return forwarder
    
    def getMessageNr(self):
        """ Returns a new message number.
        """
        self._msgNr = (self._msgNr + 1) % MsgDef.MAX_INT
        return self._msgNr
    
    def _sendMessage(self, msg):
        """ Send a message.
        """
        try:
            self.router.registerProducer(MessageSender(self, msg), msg.dest)
        except SerializationError as e:
            log.msg('Message could not be sent: {0}'.format(str(e)))
    
    def sendMessage(self, msg):
        """ Send a message.

            @param msg:     Message which should be sent to destination.
            @type  msg:     Message
        """
        self.reactor.callFromThread(self._sendMessage, msg)

    def messageReceived(self, msg):
        """ This method is called with the received message.
            
            For handling the message the first match from the msgProcessors list
            is used.

            @param msg:     Received message.
            @type  msg:     Message
        """
        for processor in self._processors:
            if processor.IDENTIFIER == msg.msgType:
                processor.processMessage(msg)
                break
        else:
            log.msg('Received a message whose type can not be handled by this CommManager.')
