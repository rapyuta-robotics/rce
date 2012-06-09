#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Protocol.py
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
from twisted.internet.error import ConnectionDone
from twisted.internet.protocol import Protocol
from twisted.internet.interfaces import IPushProducer, IConsumer

# Custom imports
from Exceptions import InternalError
from Message import MsgDef
from Message.Handler import receive

class ReappengineProtocol(Protocol, object):
    """ Reappengine Protocol.

        To send a message using this protocol a push producer should be
        registered with this protocol. The direct usage of the transport
        object is no permitted.
    """
    implements(IPushProducer, IConsumer)

    def __init__(self, factory, addr):
        """ Instantiate the Reappengine Protocol.
            
            @param factory:     Factory which created this connection.
            @type  factory:     ReappengineFactory
            
            @param addr:    Address object which is given as argument to buildProtocol of
                            the factory.
            @type  addr:    twisted::address
        """
        # Reference to parent for using persistent data
        self.factory = factory

        # Protocol variables
        self._destIP = addr.host
        self._dest = None
        self._initialized = False
        self._paused = False
        
        # Input buffer which is used for data between message FIFO changes
        self._recvBuf = ''
        
        # Variables to store current message
        self._msgHandler = None
        self._currentMsgLength = -1
        self._parsedBytes = 0
        
        # Reference on current producer
        self._producer = None
    
    @property
    def ip(self):
        """ IP address of node on the other end of this connection. """
        return self._destIP
    
    @property
    def dest(self):
        """ Communication ID of node on the other end of this connection. """
        return self._dest
    
    @dest.setter
    def dest(self, value):
        if self._dest:
            raise InternalError('There is already a destination address assigned to this connection.')
        
        self._dest = value
    
    @property
    def initialized(self):
        """ Flag which indicates whether the connection is initialized. """
        return self._initialized
    
    @initialized.setter
    def initialized(self, value):
        if not value:
            raise InternalError('Can not set a connection to uninitialized.')
        
        self._initialized = value
    
    @property
    def paused(self):
        """ Flag which indicates whether the producer is paused or not. """
        return self._paused
    
    def connectionMade(self):
        """ This method is called by twisted once the connection is established.
        """
        self.factory.startInit(self)
    
    def dataReceived(self, data):
        """ Process received raw data. (Called by twisted.)
        """
        # If there is any data stored in receive buffer add it to newly received data
        if self._recvBuf:
            data = self._recvBuf + data
            self._recvBuf = ''
        
        # Run processing as long as there is no pause requested
        while not self._paused:
            lenBuf = len(data)
            
            # If there is no more data to process leave the method
            if not lenBuf:
                return
            
            # Check if we are currently processing a message
            if self._currentMsgLength == -1:        # Indicates that there is no message
                # Check if there is enough data available to parse necessary part of the header
                if lenBuf > MsgDef.POS_DEST + MsgDef.ADDRESS_LENGTH:
                    # Parse necessary parts of header
                    self._currentMsgLength, = MsgDef.I_STRUCT.unpack(data[:MsgDef.I_LEN])
                    self._msgDest = data[MsgDef.POS_DEST:MsgDef.POS_DEST + MsgDef.ADDRESS_LENGTH]
                    msgType = data[MsgDef.POS_MSG_TYPE:MsgDef.POS_MSG_TYPE + MsgDef.MSG_TYPE_LENGTH]
                    
                    if self._initialized:
                        protocol = None
                    else:
                        protocol = self
                    
                    # Get the correct message handler
                    self._msgHandler = receive( self.factory,
                                                msgType,
                                                self._currentMsgLength,
                                                self._dest,
                                                self._msgDest,
                                                protocol )

                    # Register this instance as a producer with the retrieved consumer
                    self._msgHandler.registerProducer(self, True)
                    continue    # Important: Start loop again to make sure that we are not on hold!
                else:
                    # Not enough data available to process header return
                    self._recvBuf = data
                    break
            
            # Check if we have reached the end of a message in the buffer
            if lenBuf + self._parsedBytes >= self._currentMsgLength:
                # We have reached the end
                self._msgHandler.write(data[:self._currentMsgLength - self._parsedBytes])
                data = data[self._currentMsgLength - self._parsedBytes:]
                
                # Process the message
                self._msgHandler.unregisterProducer()

                # Reset message specific variables
                self._currentMsgLength = -1
                self._msgHandler = None
                self._parsedBytes = 0
            else:
                # We haven't reached the end of the message yet
                self._msgHandler.write(data)

                # Update number of parsed bytes
                self._parsedBytes += lenBuf
    
    def processMessage(self, msg):
        """ Callback from MessageHandler. This method is used when a message has been
            received, but the connection is not yet initialized.

            @param msg:     Received message which should contain a InitRequest.
            @type  msg:     Message
        """
        self.factory.processInitMessage(msg, self)
    
    def requestSend(self):
        """ Request that this protocol instance sends a message.
        """
        if not self._producer and self._initialized:
            producer = self.factory.commManager.router.getNextProducer(self._dest)
            
            if producer:
                producer.send(self)

    def registerProducer(self, producer, streaming):
        """ Register a producer which be used to send a message. The
            producer should be a push producer, i.e. streaming should
            be true. If this is not the case an exception is raised.

            Important:
                The producers should not use this method without being
                asked to do so, i.e. the protocol will call 'send' as soon
                as it is ready to receive a new message.
                Instead the producer should add himself to the FIFOs
                of the router.
        """
        if not streaming:
            raise NotImplementedError('Pull Producer are not supported; use Push Producer instead.')

        if self._producer:
            raise InternalError('Tried to register a producer without unregister previous producer.')

        self._producer = producer
        self.transport.registerProducer(producer, True)

    def write(self, data):
        """ Method which is used by the producer to send data of this
            protocol instance.

            Important:
                Before the producer starts to write data he should do a
                check whether he can start producing or not!
        """
        self.transport.write(data)

    def unregisterProducer(self):
        """ Method which is used by the producer to signal that he has
            finished sending data.
        """
        self.transport.unregisterProducer()
        self._producer = None
        self.requestSend()

    def pauseProducing(self):
        """ Method for the consumer to signal that he can't accept (more)
            data at the moment.
        """
        self._paused = True
        self.transport.pauseProducing()

    def resumeProducing(self):
        """ Method for the consumer to signal that he can accept (again)
            data.
        """
        self._paused = False
        self.transport.resumeProducing()
        self.dataReceived('')

    def stopProducing(self):
        """ Method for the consumer to signal that the producer should
            stop sending data.
        """
        # TODO: Drop the current message; how could this be achieved ?!?
        #       Or in other words how can we figure out where the new message start in the buffer
        #       when not the full length of the previous message is received...
        self._paused = True
        self.transport.stopProducing()
    
    def connectionLost(self, reason):
        """ Method which is called when the connection is lost.
        """
        # Unregister the connection from the factory
        self.factory.unregisterConnection(self)
        
        # TODO: Is anything else necessary?
        if not reason.check(ConnectionDone):
            reason.printTraceback(detail='verbose')
