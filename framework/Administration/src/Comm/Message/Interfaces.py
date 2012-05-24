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

# zope specific imports
from zope.interface import Attribute, Interface

# twisted specific imports
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
