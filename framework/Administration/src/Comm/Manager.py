#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Manager.py
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
from twisted.internet.protocol import Factory
from twisted.internet.task import LoopingCall

# Python specific imports
from datetime import datetime, timedelta

# Custom imports
import settings
from Exceptions import InternalError
from MessageDefinition import MessageDefinition as MsgDef
from Message import MessageSender, MessageReceiver, MessageForwarder
from Protocol import ReappengineProtocol

class ReappengineManager(Factory):
    """ This class defines the necessary methods which have to be implemented
        to use the manager with the ReappengineProtocol, its Factory and its
        Message class.

        Do not use this manager directly; instead use a subclass of the manager
        which implements the necessary methods.
    """
    def __init__(self, reactor, commID):
        """ Initialize the necessary variables for the base manager.

            @param reactor:     TODO: Add description
            @type  reactor:     reactor

            @param commID:  CommID of this node which can be used by other
                            nodes to identify this node.
            @type  commID:  str
        """
        # Reference to the twisted reactor
        self.reactor = reactor

        # CommID of this node
        self.commID = commID

        # Storage of all connections
        self.connections = {}
        self.producerQueue = {}

        # Counter for the message number
        self._msgNr = 0

        # Flag to control the behavior a bit better
        self.isShutdown = False

        # Setup periodic calling of clean method of manager
        LoopingCall(self.clean).start(settings.MSG_QUQUE_TIMEOUT)
    
    def registerConnection(self, dest, conn):
        """ Callback for Protocol instance to register the connection.

            This method should not be overwritten.

            @param dest:    CommID of node to which the connection was
                            established.
            @type  dest:    str

            @param conn:    Protocol instance which has established the
                            connection.
            @type  conn:    ReappengineProtocol
        """
        if dest in self.connections:
            raise InternalError('There is already a connection registered for the same destination.')

        self.connections[dest] = conn

        if dest not in self.producerQueue:
            # If there is no queue yet for this destination create one
            self.producerQueue[dest] = []
        elif self.producerQueue[dest]:
            # If there are already messages which should be sent
            conn.requestSend()

    def unregisterConnection(self, conn):
        """ Callback for Protocol instance to unregister the connection.

            This method should not be overwritten.

            @param conn:    Protocol instance which should be unregistered.
            @type  conn:    ReappengineProtocol
        """
        for key, value in self.connections.iteritems():
            if value == conn:
                break
        else:
            return

        del self.connections[key]

    def buildProtocol(self, addr):
        """ Builds a new protocol instance.

            This method is called when a new connection should be
            established.

            This method should not be overwritten.
        """
        return ReappengineProtocol(self)

    def getCommID(self):
        """ Returns the CommID of this node.

            This method should not be overwritten.
        """
        return self.commID

    def getMessageNr(self):
        """ Returns a new message number.

            This method should not be overwritten.
        """
        self._msgNr = (self._msgNr + 1) % MsgDef.MAX_INT
        return self._msgNr

    def nextDest(self, dest, fromProtocol):
        """ This method is called by the protocol instances to get the
            next destination point for the received message.

            This method should not be overwritten.

            @param dest:    The destination address of the received message.
            @type  dest:    str

            @param fromProtocol:    The communication ID from which the received
                                    message originated.
            @type  fromProtocol:    str

            @return:    The correct Message handler to where the message
                        should be forwarded.

            @raise:     DestinationError if no route for the ID can be found or
                        if the message is returned to the same node.
        """
        # First check if connection is authenticated
        if dest == MsgDef.NO_AUTH_ADDR:
            return MessageReceiver(self, fromProtocol, False)

        # Now check if the message is for this communication node
        if dest == self.commID:
            return MessageReceiver(self, fromProtocol, True)

        # Try to find the next point to where the message should be sent
        dest = self.resolveDest(dest)

        if dest == fromProtocol:
            # The message is returned to the sender, report problem
            pass # TODO: What to do
        
        forwarder = MessageForwarder()
        self.registerProducer(forwarder, dest)
        return forwarder

    def resolveDest(self, dest):
        """ Resolve the received destination ID and decide what to do next.

            This method has to be overwritten.

            @return:    The destination address to where the message should be
                        send next.

            @raise:     DestinationError if no route for the ID can be found.
        """
        raise NotImplementedError('The manager instance has to overwrite this method.')
    
    def registerProducer(self, producer, dest):
        """ Register a producer for a given destination.
            
            @param producer:    Producer which should be registered.
            @type  producer:    MessageSender or MessageForwarder

            @param dest:    The destination address has to be the valid
                            CommID of the destination node.
            @type  dest:    str
        """
        if self.isShutdown:
            return
        
        dest = self.resolveDest(dest)
        
        if dest not in self.producerQueue:
            # If there is no message queue yet for this destination create one
            self.producerQueue[dest] = [(producer, datetime.now())]
        else:
            # Add message to the queue
            self.producerQueue[dest].append((producer, datetime.now()))

        if dest in self.connections:
            # If there is a connection to this destination request to send a message
            self.connections[dest].requestSend()
    
    def sendMessage(self, msg, dest):
        """ Send a message.

            @param msg:     Message which should be sent to destination.
            @type  msg:     Message

            @param dest:    The destination address has to be the valid
                            CommID of the destination node.
            @type  dest:    str
        """
        self.registerProducer(MessageSender(self, msg), self.resolveDest(dest))

    def processMessage(self, msg, conn, authenticated):
        """ Process the received message.

            This method should not be overwritten.

            @param msg:     Message.
            @type  msg:     Message

            @param conn:    Connection from with the message originated.
                            (This is only used for the authentication process.)
            @type  conn:    ReappengineProtocol

            @param authenticated:   Flag to indicate whether the message was
                                    received from an authenticated source.
            @type  authenticated:   bool
        """
        if not authenticated:
            # Handle message from not yet authenticated connection
            pass # TODO: What to do
        else:
            self.messageReceived(msg)

    def messageReceived(self, msg):
        """ This method is called with the received message if it has to be handled
            by the specialized Manager, i.e. the subclass.

            This method should be overwritten.

            @param msg:     Received message.
            @type  msg:     Message
        """
        import warnings
        warnings.warn('The manager does not implement a handler for received message.', UserWarning)

    def runTaskInSeparateThread(self, func, *args, **kw):
        """ Convenience method to run any function in a separate thread.

            This method should not be overwritten.

            @param func:    Callable which should be executed in separate
                            thread.
            @type  func:    callable

            @param *args:   Any positional arguments which will be passed
                            to 'func'.

            @param *kw:     Any keyworded arguments which will be passed
                            to 'func'.
        """
        self._reactor.callInThread(func, args, kw)

    def clean(self):
        """ Method is used to clean up producer queue to remove old entries.
        """
        limit = datetime.now() - timedelta(seconds=settings.MSG_QUQUE_TIMEOUT)
        
        for commID in self.producerQueue:
            queue = self.producerQueue[commID]
            prodList = queue[:]
            queue = []
            
            for (prod, timestamp) in prodList:
                if timestamp < limit:
                    queue(prod, timestamp)
            
            self.producerQueue[commID] = queue

    def stopFactory(self):
        """ Method is called when the manager/factory is stopped.
        """
        if self.isShutdown:
            return

        self.isShutdown = True
