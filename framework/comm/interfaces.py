#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     interfaces.py
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
from zope.interface import Attribute, Interface

# twisted specific imports
from twisted.internet.interfaces import IPushProducer


class IRCEProducer(IPushProducer):
    """ Interface for all producers in the RoboEarth Cloud Engine.
    """
    origin = Attribute("""
    Comm ID from where the message originated,
    i.e. the nearest neighbor or the Comm ID of this node.
    """)
    
    def send(consumer): #@NoSelf
        """ Callback from consumer to register himself and request that the
            Producer starts to send the previously parsed message.
                                
            @param consumer:    Protocol instance which should consume this
                                message. It is also possible to use any
                                consumer which implements the IConsumer
                                interface; however, in this case the method
                                send has to be called manually.
            @type  consumer:    comm.factory._RCEProtocol (or any IConsumer)
        """


class IContentSerializer(Interface):
    """ Interface which declares the necessary methods which all content
        serializer classes have to implement.
        
        The content serializer classes will be instantiated once and will be
        used for all received and sent messages. 
    """
    IDENTIFIER = Attribute("""
    Identifier which is used to select the appropriate Processor.
    """)
    
    def serialize(fifo, data): #@NoSelf
        """ Serialize the content/data and return it.
            
            @param fifo:    Message FIFO into which the message content should
                            be serialized.
            @type  fifo:    comm.fifo.MessageFIFO
            
            @raise:     errors.SerializationError
        """
    
    def deserialize(fifo): #@NoSelf
        """ Deserialize the content/data and return it.
            
            @param fifo:    Message FIFO containing the serialized message
                            content.
            @type  fifo:    comm.fifo.MessageFIFO
            
            @raise:     errors.SerializationError
        """


class IMessageProcessor(Interface):
    """ Interface which declares the necessary methods which all processor
        classes have to implement.
        
        The Processor classes will be instantiated once and will be used for
        all received messages. 
    """
    IDENTIFIER = Attribute("""
    Identifier which is used to select the appropriate Processor.
    Valid values are registered in comm.types.
    """)
    
    def processMessage(msg, resp): #@NoSelf
        """ Process the message.
            
            @param msg:     Received message which should be processed.
            @type  msg:     Message
            
            @param resp:    Deferred which will send a status message when
                            callback is used and a error message when errback
                            is used as a response to the sender.
            @type  resp:    twisted::Deferred
        """


class IPostInit(Interface):
    """ Interface which declares the necessary method which all callbacks have
        to implement which should be called after the connection has been
        successfully initialized.
    """
    def postInit(origin, ip, data): #@NoSelf
        """ This method is called when the connection has been initialized.
            
            @param origin:  CommID of initialized connection.
            @type  origin:  str
            
            @param ip:      IP address of initialized connection.
            @type  ip:      str
            
            @param data:    Data which was sent with the INIT message.
            @type  data:    str
        """


class IPostClose(Interface):
    """ Interface which declares the necessary method which all callbacks have
        to implement which should be called after the connection has been
        disconnected.
    """
    def postClose(origin, ip): #@NoSelf
        """ This method is called when the connection has been initialized.
            
            @param origin:  CommID of closed connection.
            @type  origin:  str
            
            @param ip:      IP address of closed connection.
            @type  ip:      str
        """
