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

# twisted specific imports
from zope.interface import implements
from twisted.internet.protocol import Protocol
from twisted.internet.interfaces import IPushProducer, IConsumer

# Custom imports
from Exceptions import InternalError
from MessageDefinition import MessageDefinition as MsgDef
from Message import DestinationError, Sink

class ReappengineProtocol(Protocol):
    """ Reappengine Protocol.

        To send a message using this protocol a push producer should be
        registered with this protocol. The direct usage of the transport
        object is no permitted.
    """
    implements(IPushProducer, IConsumer)

    def __init__(self, manager):
        """ Instantiate the Reappengine Protocol.

            @param manager:     Manager which has the node specific handling
                                for Protocol callbacks.
            @type  manager:     Class which implements IReappengineManager
        """
        # Reference to parent for using persistent data
        self._manager = manager

        # Protocol variables
        self._dest = None
        self._authenticated = False
        self.paused = False
        self._recvBuf = ''

        # Variables to store current message specs for convenience
        self._currentMsgLength = -1
        self._msgDest = ''
        self._parsedBytes = 0

        # queue for waiting producers
        self._producer = None

    def dataReceived(self, data):
        """ Convert received raw data into appropriate messages.
        """
        # First add new data to existing buffer
        self._recvBuf += data

        # Run processing as long as there is no pause requested
        while not self.paused:
            # Calculate current length of buffer
            lenBuf = len(self._recvBuf)

            # If there is nothing to do return
            if not lenBuf:
                break

            # Next check if we are currently processing a message
            if self._currentMsgLength == -1:        # Indicates that there is no message
                # Check if there is enough data available to parse necessary part of the header
                if lenBuf > self.POS_DEST + MsgDef.ADDRESS_LENGTH:
                    # Parse header
                    self._currentMsgLength, = MsgDef.I_STRUCT.unpack(self._recvBuf[:MsgDef.I_LEN])
                    self._msgDest = self._recvBuf[MsgDef.POS_DEST:MsgDef.POS_DEST + MsgDef.ADDRESS_LENGTH]

                    if self._currentMsgLength > MsgDef.MAX_LENGTH:
                        # TODO: What to do with message which is too long?
                        #       At the moment the data is read but not saved or parsed.
                        self._msgDest = Sink()
                    else:
                        # Check first if the other side is authenticated
                        if not self._authenticated:
                            self._msgDest = MsgDef.NO_AUTH_ADDR

                        try:
                            self._msgDest = self._manager.nextDest(self._msgDest, self._dest)
                        except DestinationError:
                            # TODO: Resolve this DestinationError with specialized consumer/producer to process message.
                            #       At the moment the data is read but not saved or parsed.
                            self._msgDest = Sink()

                    # Register this instance as a producer with the retrieved consumer
                    self._manager.registerProducer(self, True)
                    continue    # Important: Start loop again to make sure that we are not on hold!
                else:
                    # Not enough data available return
                    break

            # Check if we have reached the end of a message in the buffer
            if lenBuf + self._parsedBytes > self._currentMsgLength:
                # We have reached the end
                self._msgDest.write(self._recvBuf[:self._currentMsgLength - self._parsedBytes])
                self._recvBuf = self._recvBuf[self._currentMsgLength - self._parsedBytes:]

                # Reset message specific variables
                self._currentMsgLength = -1
                self._msgDest = ''
                self._parsedBytes = 0
            else:
                # We haven't reached the end of the message yet
                self._msgDest.write(self._recvBuf)
                self._recvBuf = ''

                # Update number of parsed bytes
                self._parsedBytes += lenBuf

    def requestSend(self):
        """ Request that this protocol instance sends a message.
        """
        if not self._producer and self._authenticated and self._dest and self._manager.producerQueue[self._dest]:
            self._manager.producerQueue[self._dest].pop(0)[0].send(self)

    def registerProducer(self, producer, streaming):
        """ Register a producer which be used to send a message. The
            producer should be a push producer, i.e. streaming should
            be true. If this is not the case an exception is raised.

            Important:
                The producers should not use this method without being
                asked to do so, i.e. the protocol will call 'send' as soon
                as it is ready to receive a new message.
                Instead the producer should add himself to the producerQueue
                of the manager.
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
        self.paused = True
        self.transport.pauseProducing()

    def resumeProducing(self):
        """ Method for the consumer to signal that he can accept (again)
            data.
        """
        self.paused = False
        self.transport.resumeProducing()
        self.dataReceived('')

    def stopProducing(self):
        """ Method for the consumer to signal that the producer should
            stop sending data.
        """
        # TODO: Drop the current message; how could this be achieved ?!?
        #       Or in other words how can we figure out where the new message start in the buffer
        #       when not the full length of the previous message is received...
        self.paused = True
        self.transport.stopProducing()

    def connectionLost(self):
        """ Method which is called when the connection is lost.
        """
        # First of all remove the connection from the manager
        self._manager.unregisterConnection(self)

        # Now try to reconnect

        # TODO: What should be done in case the connection is lost?

