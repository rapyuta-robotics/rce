#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       InitHandler.py
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

# Custom imports
from Exceptions import SerializationError
import Message.Definition as MsgDef
from Message.TypeBase import MessageTypes as MsgTypes
from Message.Base import Message
from Message.Handler import MessageSender

class InitBase(object):
    """ Base class which declares the necessary methods which all Init subclasses
        have to overwrite.
        
        The Init classes will be instantiated once and will be used for all
        Init messages. 
    """
    def __init__(self, manager, factory):
        """ @param manager:     Manager which is used in this node.
            @type  manager:     ReappengineManager
            
            @param factory:     Factory which handles the connection.
            @type  factory:     ReappengineFactory
        """
        self.manager = manager
    
    def sendInitMessage(self, conn):
        """ Send a initialization message to the specified connection.
        """
    
    def receivedInitMessage(self, msg, conn):
        """ Received an initialization message from the specified connection.
        """
    
class ContainerInit(InitBase):
    """ Init class which is used in the container node.
    """
    def __init__(self, manager, factory, satelliteID, key):
        """ @param manager:     Manager which is used in this node.
            @type  manager:     ReappengineManager
            
            @param factory:     Factory which handles the connection.
            @type  factory:     ReappengineFactory
            
            @param satelliteID:     ID which is used to authenticate the server.
            @type  satelliteID:     str
            
            @param key:         Key which should be sent with initial message
                                to authenticate this node.
            @type  key:         str
        """
    
    def sendInitMessage(self, conn):
        msg = Message()
        msg.msgType = MsgTypes.INIT_REQUEST
        msg.dest = '-' * MsgDef.ADDRESS_LENGTH
        
        msg.content = { 'origin' : self.manager.getCommID(),
                        'dest'   : self.factory._satelliteID,
                        'key'    : self.factory._key }
        
        try:
            MessageSender(self.manager, msg).send(conn)
        except SerializationError as e:
            log.msg('Could not serialize message: {0}'.format(e))
            conn.transport.loseConnection()
        