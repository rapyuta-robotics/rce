#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Factory.py
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
from twisted.internet.protocol import ServerFactory, ReconnectingClientFactory

# Custom imports
from Exceptions import SerializationError
import Message.Definition as MsgDef
from Message.TypeBase import MessageTypes as MsgTypes
from Protocol import ReappengineProtocol
from Message.Base import Message
from Message.Handler import MessageSender

class ReappengineFactory(object):
    """ Base factory which implements base methods for all Reappengine Factories.
    """
    def filterMessage(self, msgType):
        """ Method which is called by the protocol to filter the incoming messages.
                                
            @param msgType:     String identifier of the message type.
            @type  msgType:     str
            
            @return:        If the message should be filtered out True is returned and
                            False otherwise.
            @rtype:         str
        """
    
    def startInit(self):
        """ Method which is called when the connection has been made.
            
            @param conn:    Protocol instance which just has been made.
            @type  conn:    ReappengineProtocol
        """
    
    def processInitMessage(self, msg, conn):
        """ Method which is used to when a message is received and the initialized
            flag of the protocol instance is not yet set.

            @param msg:     Received message which should contain a InitRequest.
            @type  msg:     Message
            
            @param conn:    Protocol instance who received the initialization message.
            @type  conn:    ReappengineProtocol
        """
    
    def postInitTrigger(self, origin):
        """ This method is called once as soon the connection is successfully initialized.
            
            @param origin:  CommID of request origin.
            @type  origin:  str
        """
    
    def unregisterConnection(self, conn):
        """ This method should be called to remove any references to the given connection.
            
            @param conn:    Protocol instance who should be unregistered.
            @type  conn:    ReappengineProtocol
        """

class ReappengineClientFactory(ReappengineFactory, ReconnectingClientFactory):
    """ Factory which is used for client connections.
    """
    def __init__(self, manager, satelliteID, key, overrideCommID=False):
        """ Initialize the Reappengine Factory.

            @param manager:     Manager which has the node specific handling
                                for Protocol callbacks.
            @type  manager:     ReappengineManager
            
            @param satelliteID:     CommID of the satellite node.
            @type  satelliteID:     str
            
            @param key:     Key which should be sent together with the initialization
                            request.
            @type  key:     str
            
            @param overrideCommID:  Flag to indicate whether the received ID should
                                    be used as a new CommID for this node.
            @type  overrideCommID:  bool
        """
        self.manager = manager
        self._satelliteID = satelliteID
        self._key = key
        self._override = overrideCommID
    
    def buildProtocol(self, addr):
        """ Builds a new protocol instance.

            This method is called when a new connection should be
            established.

            This method should not be overwritten.
        """
        self.resetDelay()
        return ReappengineProtocol(self, addr)
    
    def startInit(self, conn):
        msg = Message()
        msg.msgType = MsgTypes.INIT_REQUEST
        msg.dest = MsgDef.NEIGHBOR_ADDR
        
        msg.content = { 'origin' : self.manager.commID,
                        'dest'   : self._satelliteID,
                        'key'    : self._key }
        
        try:
            MessageSender(self.manager, msg).send(conn)
        except SerializationError as e:
            log.msg('Could not serialize message: {0}'.format(e))
            conn.transport.loseConnection()
    
    def processInitMessage(self, msg, conn):
        # First some base checks of message header
        if msg.msgType != MsgTypes.INIT_REQUEST:
            log.msg('Received message type different from INIT_REQUEST before initialization of protocol instance has been completed.')
            conn.transport.loseConnection()
            return
        
        if msg.content['origin'] != self._satelliteID or msg.origin != self._satelliteID:
            log.msg('Received origin address does not match this node satellite address for initialization of protocol instance.')
            conn.transport.loseConnection()
            return
        
        self._key = msg.content['key']
        self._dest = msg.content['origin']
        
        if self._override:
            self.manager.commID = msg.content['dest']
        
        conn.initialized = True
        self.manager.registerConnection(conn)
        log.msg('Connection established to "{0}".'.format(self._dest))
        
        # Trigger the post init method
        self.postInitTrigger(self._dest)
    
    def unregisterConnection(self, conn):
        self.manager.unregisterConnection(conn)

class ReappengineServerFactory(ReappengineFactory, ServerFactory):
    """ Factory which is used for server connections.
    """
    def buildProtocol(self, addr):
        """ Builds a new protocol instance.

            This method is called when a new connection should be
            established.

            This method should not be overwritten.
        """
        return ReappengineProtocol(self, addr)
    
    def authOrigin(self, origin, key):
        """ Authenticate the origin using the attached key from the InitRequest.
            
            @param origin:  CommID of request origin.
            @type  origin:  str
            
            @param key:     Key which was sent with the request.
            @type  key:     str
            
            @return:        Return True if the origin was successfully authenticated.
                            (In default implementation True is always returned.)
        """
        return True
    
    def getResponse(self, origin):
        """ Generate the dictionary for the returned InitRequest message.
            
            This method has to be implemented.
            
            @param origin:  CommID of request origin.
            @type  origin:  str
            
            @return:        Dictionary containing all necessary fields for the InitRequest
                            message:    dest, origin, key
        """
        raise NotImplementedError('The method "getResponse" has to be implemented for all Server Factories.')
    
    def saveState(self, content):
        """ Save the state of the connection. This method is called after the response
            message has been successfully sent and before the connection is set to
            initialized.
            
            @param content:     Previously constructed message content.
            @type  content:     dict
        """
    
    def processInitMessage(self, msg, conn):
        # First some base checks of message header
        if msg.msgType != MsgTypes.INIT_REQUEST:
            log.msg('Received message type different from INIT_REQUEST before initialization of protocol instance has been completed.')
            conn.transport.loseConnection()
            return
        
        if msg.content['dest'] != self.manager.commID:
            log.msg('Received destination address does not match this node for initialization of protocol instance.')
            conn.transport.loseConnection()
            return
        
        origin = msg.content['origin']
        
        if msg.origin != origin:
            log.msg('Received origin address does not match the specification of the initialization request.')
            conn.transport.loseConnection()
            return
        
        # Authenticate origin with key
        if not self.authOrigin(origin, msg.content['key']):
            log.msg('Origin could not be authenticated.')
            conn.transport.loseConnection()
            return
        
        # Generate content
        content = self.getResponse(origin)
        
        msg = Message()
        msg.msgType = MsgTypes.INIT_REQUEST
        msg.dest = origin
        msg.content = content
        
        try:
            MessageSender(self.manager, msg).send(conn)
        except SerializationError as e:
            log.msg('Could not serialize message: {0}'.format(e))
            conn.transport.loseConnection()
            return
        
        # Save state
        self.saveState(content)
        
        # Set protocol to initialized and register connection in manager
        conn.dest = origin
        conn.initialized = True
        self.manager.registerConnection(conn)
        log.msg('Connection established to "{0}".'.format(origin))
        
        # Trigger the post init method
        self.postInitTrigger(origin)
    
    def unregisterConnection(self, conn):
        self.manager.unregisterConnection(conn)
