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
from twisted.python import log
from twisted.internet.task import LoopingCall

# Python specific imports
from datetime import datetime, timedelta

# Custom imports
import settings
from Exceptions import InternalError
import Message.Definition as MsgDef
from Message.Base import DestinationError
from Message.Handler import MessageSender, MessageReceiver, MessageForwarder, Sink
from Message.StdProcessor import RouteProcessor

class ReappengineManager(object):
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
        
        # List containing all valid message processors for this node
        self.msgProcessors = []
        
        # Storage of all connections
        self.connections = {}
        self.producerQueue = {}
        
        # Storage of routing information
        self.routingTable = {}
        
        # List of all available basic message processors
        self.msgProcessors = [ RouteProcessor(self) ]

        # Counter for the message number
        self._msgNr = 0

        # Flag to control the behavior a bit better
        self.isShutdown = False

        # Setup periodic calling of clean method of manager
        LoopingCall(self.clean).start(settings.MSG_QUQUE_TIMEOUT)
    
    def registerConnection(self, conn):
        """ Callback for Protocol instance to register the connection.

            This method should not be overwritten.

            @param conn:    Protocol instance which has established the
                            connection.
            @type  conn:    ReappengineProtocol
        """
        if conn.dest in self.connections:
            raise InternalError('There is already a connection registered for the same destination.')
        
        # Register connection
        self.connections[conn.dest] = conn
        
        # Register connection in routing table
        self.routingTable[conn.dest] = conn.dest
        
        # TODO: Fire an routeInfoUpdate event (probably only for selected connection...)
        
        if conn.dest not in self.producerQueue:
            # If there is no queue yet for this destination create one
            self.producerQueue[conn.dest] = []
        elif self.producerQueue[conn.dest]:
            # If there are already messages which should be sent
            conn.requestSend()

    def unregisterConnection(self, conn):
        """ Callback for Protocol instance to unregister the connection.

            This method should not be overwritten.

            @param conn:    Protocol instance which should be unregistered.
            @type  conn:    ReappengineProtocol
        """
        for dest, value in self.connections.iteritems():
            if value == conn:
                break
        else:
            return
        
        # Unregister connection in routing table
        try:
            del self.routingTable[dest]
        except KeyError:
            log.msg('Tried to remove a routing information for a registered connection which was never added.')
        
        # TODO: Fire an routeInfoUpdate event (probably only for selected connection...)
        
        # Unregister connection
        del self.connections[dest]
    
    def updateRoutingInfo(self, info):
        """ Callback for the RoutingInfo processor to update the routing information
            in this manager.
            
            @param info:    Dictionary containing as keys the destinations and as value
                            the next waypoint of the route. The key represents a default
                            destination, which is used for all destinations which do not
                            have an explicit route. If the info dictionary contains None as 
                            value the corresponding key will be removed form the routing
                            table.
            @type  info:    { str/None : str/None }
        """
        for dest, route in info.iteritems():
            log.msg('Add route to {0} via {1}.'.format(dest, route))
            if route is None:
                self.routingTable.pop(dest, None)
            else:
                self.routingTable[dest] = route

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
        # Check if the message is for this communication node
        if dest == self.commID or dest == MsgDef.NEIGHBOR_ADDR:
            return MessageReceiver(self, True)

        # Try to find the next point to where the message should be sent
        dest = self.resolveDest(dest)

        if dest == fromProtocol:
            # The message is returned to the sender, report problem
            # TODO: Good idea to drop the message here?
            #       Most probably the message would be sent here again anyway...
            log.msg('Tried to return a message to the sender. Message will be dropped.')
            return Sink()
        
        forwarder = MessageForwarder()
        self.registerProducer(forwarder, dest)
        return forwarder

    def resolveDest(self, dest):
        """ Resolve the received destination ID and decide what to do next.

            This method uses the information set in the dictionary 'routingTable'.
            First this method tries to use the destination address as a key in the
            dictionary to get the next destination. If there is no matching key
            the key None is used which symbolizes the default address. If there
            is also no key None, than a DestinationError is raised.

            @return:    The destination address to where the message should be
                        send next.

            @raise:     DestinationError if no route for the ID can be found.
        """
        if dest in self.routingTable:
            return self.routingTable[dest]
        
        if None in self.routingTable:
            return self.routingTable[None]
        
        raise DestinationError('Can not route the destination "{0}".'.format(dest))
    
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
    
    def _sendMessage(self, msg):
        """ Send a message.
        """
        self.registerProducer(MessageSender(self, msg), self.resolveDest(msg.dest))
    
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
        if self.isShutdown:
            return
        
        for msgProcessor in self.msgProcessors:
            if msgProcessor.IDENTIFIER == msg.msgType:
                msgProcessor.processMessage(msg)
                break
        else:
            log.msg('Received a message whose type can not be handled by this node.')

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
                if timestamp > limit:
                    queue.append(prod, timestamp)
                else:
                    log.msg('Producer has been dropped from queue for destination "{0}".'.format(commID))
            
            self.producerQueue[commID] = queue

    def shutdown(self):
        """ Method is called when the manager/factory is stopped.
        """
        if self.isShutdown:
            return

        self.isShutdown = True
