#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Router.py
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

# Custom imports
import settings
from Exceptions import InternalError
from FIFO import ProducerFIFO
from CommUtil import validateAddress

class Router(object):
    """ Class which is responsible for routing the communication.
    """
    def __init__(self):
        """ Initialize the Router.
        """
        # Key:    Destination ID of connection endpoint
        # Value:  Connection/Protocol instances
        self._connections = {}
        
        # Key:    Destination ID of message endpoint
        # Value:  FIFOs where Producers are stored
        self._fifos = {}
        
        # Key:    Destination ID of message endpoint
        # Value:  Destination ID of connection endpoint which should be used as next waypoint
        self._routes = {}
        
        # Key:    Destination ID of connection endpoint
        # Value:  Set of Destination IDs of message endpoints which should be routed through this connection
        self._dest = {}
        
        # Destination ID of default connection endpoint or None
        self._default = None
        
        # Setup periodic calling of clean method of manager
        LoopingCall(self._clean).start(settings.MSG_QUQUE_TIMEOUT / 2)
    
    def registerConnection(self, conn):
        """ Callback for Protocol instance to register the connection.

            @param conn:    Protocol instance which has established the connection.
            @type  conn:    RCEProtocol
        """
        dest = conn.dest
        
        log.msg('Router: Register connection from {0}'.format(dest))
        
        if dest in self._connections:
            raise InternalError('There is already a connection registered for the same destination.')
        
        # Register connection
        self._connections[dest] = conn
        
        if dest not in self._fifos:
            # If there is no FIFO yet for this destination create one
            self._fifos[dest] = ProducerFIFO(dest)
        
        if dest not in self._dest:
            self._dest[dest] = set()
        
        # Add direct route to routing table
        self._routes[dest] = dest
        self._dest[dest].add(dest)
        
        # Try to start to send messages if it is necessary
        self._connections[dest].requestSend()
    
    def unregisterConnection(self, conn):
        """ Callback for Protocol instance to unregister the connection.

            @param conn:    Protocol instance which should be unregistered.
            @type  conn:    RCEProtocol
        """
        self._connections.pop(conn.dest, None)
        
        # TODO: Fire an routeInfoUpdate event (probably only for selected connections)
        #       Only necessary in Server node; check if this is not done there.
    
    def registerProducer(self, producer, dest):
        """ Register a producer for a given destination.
            
            @param producer:    Producer which should be registered.
            @type  producer:    MessageSender or MessageForwarder

            @param dest:    The destination address has to be the valid
                            CommID of the destination node.
            @type  dest:    str
        """
        if not validateAddress(dest, True):
            log.msg('Message could not be sent: "{0}" is not a valid address'.format(dest))
            return
        
        if dest not in self._fifos:
            # If there is no FIFO yet for this destination create one
            self._fifos[dest] = ProducerFIFO(dest)
        
        # Add message to the queue
        self._fifos[dest].add(producer)
        
        # Try to send the message
        if dest in self._routes:
            route = self._routes[dest]
        elif self._default:
            route = self._default
        else:
            return
        
        if route in self._connections:
            self._connections[route].requestSend()
    
    def updateRoutingInfo(self, route, dests):
        """ Callback for the RoutingInfo processor to update the routing information
            in this CommManager.
            
            @param route:   Destination address which should be used as next routing point
                            for all the destination in 'dests'.
            @type  route:   str
            
            @param dests:   List of 2-tuples with all destination IDs which should be routed
                            through 'route' as first element of the tuples. The destination IDs
                            have to be either valid CommIDs or None, which is used as a default
                            routing address. The second element of the tuple is a flag to indicate
                            whether the routing should be added (True) or removed (False).
            @type  dests:   [ (str, bool) ]
        """
        for (dest, add) in dests:
            if dest is None:
                if add:
                    log.msg('Add default route via {0}.'.format(route))
                    self._default = route
                else:
                    log.msg('Remove default route via {0}.'.format(route))
                    self._default = None
            else:
                if add:
                    log.msg('Add route to {0} via {1}.'.format(dest, route))
                    self._routes[dest] = route
                    self._dest[route].add(dest)
                else:
                    log.msg('Remove route to {0} via {1}.'.format(dest, route))
                    
                    try:
                        del self._routes[dest]
                    except KeyError:
                        pass
                    
                    try:
                        self._dest[route].remove(dest)
                    except KeyError:
                        pass
        
        if route in self._connections:
            self._connections[route].requestSend()
    
    def getNextProducer(self, route):
        """ Callback for the protocol instances which is used when a new producer can be
            processed.
            
            @param route:   Communication ID of destination of protocol.
            @type  route:   str
            
            @return:        Next producer which should be processed or None if no processor
                            are left to process.
            @rtype:         Producer (MessageSender/MessageForwarder) or None
        """
        oldestFifo = None
        oldestTimestamp = None
        
        if route == self._default:
            dests = self._fifos.keys()
        else:
            dests = self._dest[route]
        
        while 1:
            for dest in dests:
                fifo = self._fifos[dest]
                
                if not len(fifo):
                    continue
                
                timestamp = fifo.getTimeStamp()
                    
                if oldestTimestamp is None or timestamp < oldestTimestamp:
                    oldestTimestamp = timestamp
                    oldestFifo = fifo
            
            if oldestFifo:
                producer = oldestFifo.pop()
                
                if route != producer.origin:
                    return producer
                else:
                    # If the message would be sent back to where it came from, drop it and try to
                    # get the next producer
                    log.msg('Tried to return a message to the sender. Message will be dropped.')
            else:
                return None
    
    def _clean(self):
        """ Method is used to clean up producer queue to remove old entries.
        """
        for fifo in self._fifos.itervalues():
            fifo.clean()
