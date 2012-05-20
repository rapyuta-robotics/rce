#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Interfaces.py
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

from zope.interface import Attribute, Interface
from twisted.internet.interfaces import IPushProducer

class IReappengineProducer(IPushProducer):
    """ Interface for all producers in the reappengine.
    """
    origin = Attribute("""
    Comm ID from where the message originated,
    i.e. the nearest neighbor or the Comm ID of this node.
    """)
    
    def send(consumer): #@NoSelf
        """ Callback from consumer to register himself and request that the Producer starts
            to send the previously parsed message.
                                
            @param consumer:    Protocol instance which should consume this message.
                                It is also possible to use any consumer which implements the
                                IConsumer interface; however, in this case the method send has
                                to be called manually.
            @type  consumer:    ReappengineProtocol (or any IConsumer)
        """

class IContentSerializer(Interface):
    """ Interface which declares the necessary methods which all content serializer classes
        have to implement.
        
        The content serializer classes will be instantiated once and will be used for all
        received
        and sent messages. 
    """
    IDENTIFIER = Attribute("""
    Identifier which is used to select the appropriate Processor.
    Valid values are registered in MsgTypes.
    """)
    
    def serialize(data): #@NoSelf
        """ Serialize the content/data and return it.
            
            @raise:     SerializationError
        """
    
    def deserialize(data): #@NoSelf
        """ Deserialize the content/data and return it.
            
            @raise:     SerializationError
        """

class IMessageProcessor(Interface):
    """ Interface which declares the necessary methods which all processor classes
        have to implement.
        
        The Processor classes will be instantiated once and will be used for all
        received messages. 
    """
    IDENTIFIER = Attribute("""
    Identifier which is used to select the appropriate Processor.
    Valid values are registered in MsgTypes.
    """)
    
    def processMessage(msg): #@NoSelf
        """ Process the message.
            
            All references which are necessary to process the message have to available
            as instance variables as the arguments of this method can not be changed.
        """

class IPostInitTrigger(Interface):
    """ Interface which declares the necessary methods which all post init triggers
        have to implement.
    """
    def trigger(origin): #@NoSelf
        """ This method is called whenever the PostInitTrigger should be triggered.
            
            @param origin:  CommID of request origin.
            @type  origin:  str
        """

class IServerImplementation(Interface):
    """ Interface which declares the necessary methods which all server implementations
        have to implement.
    """
    def authOrigin(origin, key): #@NoSelf
        """ Authenticate the origin using the attached key from the InitRequest.
            
            @param origin:  CommID of request origin.
            @type  origin:  str
            
            @param key:     Key which was sent with the request.
            @type  key:     str
            
            @return:        Return True if the origin was successfully authenticated.
        """
    
    def getResponse(origin): #@NoSelf
        """ Generate the dictionary for the returned InitRequest message.
            
            This method has to be implemented.
            
            @param origin:  CommID of request origin.
            @type  origin:  str
            
            @return:        Dictionary containing all necessary fields for the InitRequest
                            message:    dest, origin, key
        """
    
    def saveState(content): #@NoSelf
        """ Save the state of the connection. This method is called after the response
            message has been successfully sent and before the connection is set to
            initialized.
            
            @param content:     Previously constructed message content.
            @type  content:     dict
        """
    
    def unregisterConnection(conn): #@NoSelf
        """ This method should be called to remove any references to the given connection.
            
            @param conn:    Protocol instance who should be unregistered.
            @type  conn:    ReappengineProtocol
        """
