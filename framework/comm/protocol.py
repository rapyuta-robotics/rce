#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     protocol.py
#     
#     This file is part of the RoboEarth Cloud Engine framework.
#     
#     This file was originally created for RoboEearth
#     http://www.roboearth.org/
#     
#     The research leading to these results has received funding from
#     the European Union Seventh Framework Programme FP7/2007-2013 under
#     grant agreement no248942 RoboEarth.
#     
#     Copyright 2012 RoboEarth
#     
#     Licensed under the Apache License, Version 2.0 (the "License");
#     you may not use this file except in compliance with the License.
#     You may obtain a copy of the License at
#     
#     http://www.apache.org/licenses/LICENSE-2.0
#     
#     Unless required by applicable law or agreed to in writing, software
#     distributed under the License is distributed on an "AS IS" BASIS,
#     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#     See the License for the specific language governing permissions and
#     limitations under the License.
#     
#     \author/s: Dominique Hunziker 
#     
#     

# zope specific imports
from zope.interface import implements

# twisted specific imports
from twisted.python import log
from twisted.internet.interfaces import IPushProducer, IConsumer
from twisted.internet.error import ConnectionDone
from twisted.internet.protocol import Protocol, ServerFactory, \
    ReconnectingClientFactory

# Custom imports
from errors import InternalError, SerializationError
from util.interfaces import verifyObject
from comm import definition
from comm import types
from comm.interfaces import IPostInit, IPostClose
from comm.message import Message
from comm.handler import _Sender, receive


class _RCEProtocol(Protocol, object):
    """ RCE Protocol.

        To send a message using this protocol a push producer should be
        registered with this protocol. The direct usage of the transport
        object is no permitted.
    """
    implements(IPushProducer, IConsumer)

    def __init__(self, factory, addr):
        """ Initialize the RCEProtocol.
            
            @param factory:     Factory which created this connection.
            @type  factory:     comm.factory._RCEFactory
            
            @param addr:    Address object which is given as argument to
                            buildProtocol of the factory.
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
            raise InternalError('There is already a destination address '
                                'assigned to this connection.')
        
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
        """ This method is called by twisted once the connection is
            established.
        """
        self.factory.startInit(self)
    
    def dataReceived(self, data):
        """ Process received raw data. (Called by twisted.)
        """
        # If there is any data stored in receive buffer
        # add it to newly received data
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
            if self._currentMsgLength == -1:    # There is no message
                # Check if there is enough data available
                # to parse necessary part of the header
                if lenBuf > definition.POS_DEST + definition.ADDRESS_LENGTH:
                    # Parse necessary parts of header
                    self._currentMsgLength, = definition.I_STRUCT.unpack(
                                                  data[:definition.I_LEN])
                    self._msgDest = data[definition.POS_DEST:
                                         definition.POS_DEST
                                         + definition.ADDRESS_LENGTH]
                    msgType = data[definition.POS_MSG_TYPE:
                                   definition.POS_MSG_TYPE
                                   + definition.MSG_TYPE_LENGTH]
                    
                    # According to convention of receive set appropriate value
                    if self._initialized:
                        protocol = None
                    else:
                        protocol = self
                    
                    # Get the correct message handler
                    self._msgHandler = receive(self.factory,
                                               msgType,
                                               self._currentMsgLength,
                                               self._dest,
                                               self._msgDest,
                                               protocol)

                    # Register this instance as a producer
                    # with the retrieved consumer
                    self._msgHandler.registerProducer(self, True)
                    continue    # Important: Start loop again to make sure
                                # that we are not on hold!
                else:
                    # Not enough data available to process header return
                    self._recvBuf = data
                    return
            
            # Check if we have reached the end of a message in the buffer
            if lenBuf + self._parsedBytes >= self._currentMsgLength:
                # We have reached the end
                self._msgHandler.write(data[:self._currentMsgLength
                                             - self._parsedBytes])
                data = data[self._currentMsgLength-self._parsedBytes:]
                
                # Process the message
                self._msgHandler.unregisterProducer()

                # Reset message specific variables
                self._currentMsgLength = -1
                self._msgHandler = None
                self._parsedBytes = 0
            else:
                # We haven't reached the end of the message yet
                self._msgHandler.write(data)
                data = ''

                # Update number of parsed bytes
                self._parsedBytes += lenBuf
    
    def processMessage(self, msg, _):
        """ Callback from MessageHandler. This method is used when a message
            has been received, but the connection is not yet initialized.

            @param msg:     Received message which should contain a
                            InitRequest.
            @type  msg:     comm.message.Message
        """
        self.factory.processInitMessage(msg, self)
    
    def requestSend(self):
        """ Request that this protocol instance sends a message.
        """
        if not self._producer and self._initialized:
            producer = self.factory.commManager.router.getNextProducer(
                           self._dest)
            
            if producer:
                producer.send(self)

    def registerProducer(self, producer, streaming):
        """ Register a producer which be used to send a message. The producer
            should be a push producer, i.e. streaming should be true. If this
            is not the case an exception is raised.

            Important:
                The producers should not use this method without being asked to
                do so, i.e. the protocol will call 'send' as soon as it is
                ready to receive a new message.
                Instead the producer should add himself to the FIFOs of the
                router.
        """
        if not streaming:
            raise NotImplementedError('Pull Producer are not supported; '
                                      'use Push Producer instead.')

        if self._producer:
            raise InternalError('Tried to register a producer without '
                                'unregister previous producer.')

        self._producer = producer
        self.transport.registerProducer(producer, True)

    def write(self, data):
        """ Method which is used by the producer to send data of this protocol
            instance.

            Important:
                Before the producer starts to write data he should do a check
                whether he can start producing or not!
        """
        self.transport.write(data)

    def unregisterProducer(self):
        """ Method which is used by the producer to signal that he has finished
            sending data.
        """
        self.transport.unregisterProducer()
        self._producer = None
        self.requestSend()

    def pauseProducing(self):
        """ Method for the consumer to signal that he can't accept (more) data
            at the moment.
        """
        self._paused = True
        self.transport.pauseProducing()

    def resumeProducing(self):
        """ Method for the consumer to signal that he can accept (again) data.
        """
        self._paused = False
        self.transport.resumeProducing()
        self.dataReceived('')

    def stopProducing(self):
        """ Method for the consumer to signal that the producer should stop
            sending data.
        """
        # TODO: Drop the current message; how could this be achieved ?!?
        #       Or in other words how can we figure out where the new message
        #       start in the buffer when not the full length of the previous
        #       message is received...
        self._paused = True
        self.transport.stopProducing()
    
    def connectionLost(self, reason):
        """ Method which is called when the connection is lost.
        """
        # Unregister the connection from the factory
        self.factory.unregisterConnection(self)
        
        if not reason.check(ConnectionDone):
            reason.printTraceback(detail='verbose')


class _RCEFactory(object):
    """ Base class which implements base methods for all RCEFactories.
    """
    def __init__(self, commMngr, postInit=[], postClose=[]):
        """ Initialize the RCEFactory.
            
            @param commMngr:    CommManager instance which should be used with
                                this factory and its build protocols.
            @type  commMngr:    comm.manager.CommManager
            
            @param postInit:    Instances which should be informed about the
                                successful initialization of a connection.
            @type  postInit:    comm.interfaces.IPostInit
            
            @param postClose:   Instances which should be informed about the
                                closing of a connection.
            @type  postClose:   comm.interfaces.IPostClose
            
            @raise:     util.interfaces.InterfaceError if the callbacks from
                        postInit do not implement comm.interfaces.IPostInit 
                        and the callbacks from postClose do not implement
                        comm.interfaces.IPostClose.
        """
        for cb in postInit:
            verifyObject(IPostInit, cb)
        
        for cb in postClose:
            verifyObject(IPostClose, cb)
        
        self._commManager = commMngr
        self._filter = set([types.INIT_REQUEST, types.ROUTE_INFO])
        self._postInit = postInit
        self._postClose = postClose
    
    @property
    def commManager(self):
        """ CommManager instance used with this factory. """
        return self._commManager
    
    def addApprovedMessageTypes(self, msgTypes):
        """ Method which is used to add a MessageTypes to the list of approved
            types.
            
            If the message type is not in the list of approved types the
            message will not be processed in this node.
                                
            @param msgTypes:    List of the approved message types. Valid types
                                are available in comm.types.
            @type  msgTypes:    list
        """
        for msgType in msgTypes:
            self._filter.add(msgType)
    
    def filterMessage(self, msgType):
        """ Method which is called by the protocol to filter the incoming
            messages.
                                
            @param msgType:     String identifier of the message type.
            @type  msgType:     str
            
            @return:        If the message should be filtered out True is
                            returned and False otherwise.
            @rtype:         str
        """
        return msgType not in self._filter
    
    def startInit(self, conn):
        """ Method which is called when the connection has been made.
            
            @param conn:    Protocol instance which just has been made.
            @type  conn:    comm.factory_RCEProtocol
        """
    
    def getInitData(self):
        """ Generate the data which should be sent with the INIT message to
            the client.
            
            @return:        Data which should be returned.
            @rtype:         [str]
        """
        return []
    
    def processInitMessage(self, msg, conn):
        """ Method which is used to when a message is received and the
            initialized flag of the protocol instance is not yet set.

            @param msg:     Received message which should contain a
                            InitRequest.
            @type  msg:     comm.message.Message
            
            @param conn:    Protocol instance who received the initialization
                            message.
            @type  conn:    comm.factory._RCEProtocol
        """
    
    def unregisterConnection(self, conn):
        """ This method should be called to remove any references to the given
            connection.
            
            @param conn:    Protocol instance who should be unregistered.
            @type  conn:    comm.factory._RCEProtocol
        """
        self._commManager.router.unregisterConnection(conn)
        
        if conn.dest:
            for cb in self._postClose:
                cb.postClose(conn.dest, conn.ip)


class RCEClientFactory(_RCEFactory, ReconnectingClientFactory):
    """ Factory which is used for client connections.
    """
    def __init__(self, commMngr, serverID, postInit=[], postClose=[]):
        """ Initialize the RCEClientFactory.

            @param commMngr:    CommManager which is responsible for handling
                                the communication in this node.
            @type  commMngr:    comm.manager.CommManager
            
            @param serverID:    CommID of the server node.
            @type  serverID:    str
            
            @param postInit:    Instances which should be informed about the
                                successful initialization of a connection.
            @type  postInit:    comm.interfaces.IPostInit
            
            @param postClose:   Instances which should be informed about the
                                closing of a connection.
            @type  postClose:   comm.interfaces.IPostClose
            
            @raise:     util.interfaces.InterfaceError if the callbacks from
                        postInit do not implement comm.interfaces.IPostInit 
                        and the callbacks from postClose do not implement
                        comm.interfaces.IPostClose.
        """
        _RCEFactory.__init__(self, commMngr, postInit, postClose)
        
        self._serverID = serverID
    
    def buildProtocol(self, addr):
        """ Builds a new protocol instance.

            This method is called when a new connection should be established.

            This method should not be overwritten.
        """
        self.resetDelay()
        return _RCEProtocol(self, addr)
    
    def startInit(self, conn):
        msg = Message()
        msg.msgType = types.INIT_REQUEST
        msg.dest = definition.NEIGHBOR_ADDR
        
        msg.content = { 'data' : [self._serverID] + self.getInitData() }
        
        try:
            buf = msg.serialize(self._commManager)
            _Sender(len(buf),
                    self._commManager.commID,
                    msg.dest,
                    buf).send(conn)
        except SerializationError as e:
            log.msg('Could not serialize message: {0}'.format(e))
            conn.transport.loseConnection()
    
    def processInitMessage(self, msg, conn):
        # First some base checks of message header
        if msg.msgType != types.INIT_REQUEST:
            log.msg('Received message type different from INIT_REQUEST '
                    'before initialization of protocol instance has been '
                    'completed.')
            conn.transport.loseConnection()
            return
        
        origin = msg.origin
        
        if origin != self._serverID:
            log.msg('Received origin ID does not match this node server ID '
                    'for initialization of protocol instance.')
            conn.transport.loseConnection()
            return
        
        # Set protocol to initialized and register connection in manager
        conn.dest = origin
        conn.initialized = True
        self._commManager.router.registerConnection(conn)
        log.msg('Connection established to "{0}".'.format(origin))
        
        # Trigger the post init method
        for cb in self._postInit:
            cb.postInit(origin, conn.ip, msg.content['data'])


class RCEServerFactory(_RCEFactory, ServerFactory):
    """ Factory which is used for server connections.
    """
    def buildProtocol(self, addr):
        """ Builds a new protocol instance.

            This method is called when a new connection should be established.

            This method should not be overwritten.
        """
        return _RCEProtocol(self, addr)
    
    def processInitMessage(self, msg, conn):
        # First some base checks of message header
        if msg.msgType != types.INIT_REQUEST:
            log.msg('Received message type different from INIT_REQUEST '
                    'before initialization of protocol instance has been '
                    'completed.')
            conn.transport.loseConnection()
            return
        
        data = msg.content['data']
        
        if data[0] != self._commManager.commID:
            log.msg("Received target ID does not match this node's for "
                    'initialization of protocol instance.')
            conn.transport.loseConnection()
            return
        
        origin = msg.origin
        
        if not self.authOrigin(origin):
            log.msg('Origin could not be authenticated.')
            conn.transport.loseConnection()
            return
        
        msg = Message()
        msg.msgType = types.INIT_REQUEST
        msg.dest = origin
        msg.content = { 'data' : self.getInitData() }
        
        try:
            buf = msg.serialize(self._commManager)
            _Sender(len(buf), self._commManager.commID, msg.dest, buf).send(
                conn)
        except SerializationError as e:
            log.msg('Could not serialize message: {0}'.format(e))
            conn.transport.loseConnection()
            return
        
        # Set protocol to initialized and register connection in manager
        conn.dest = origin
        conn.initialized = True
        self._commManager.router.registerConnection(conn)
        log.msg('Connection established to "{0}".'.format(origin))
        
        # Trigger the post init method
        for cb in self._postInit:
            cb.postInit(origin, conn.ip, data[1:])
    
    def authOrigin(self, origin):
        """ Authenticate the origin of the InitRequest.
            
            @param origin:  CommID of request origin.
            @type  origin:  str
            
            @return:        Return True if the origin was successfully
                            authenticated.
        """
        return True
