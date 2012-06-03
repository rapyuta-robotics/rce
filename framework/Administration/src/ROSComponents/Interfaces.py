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
from zope.interface import Interface, Attribute

class ISerializable(Interface):
    """ Interface which declares the necessary methods which all serializable objects
        have to implement.
    """
    IDENTIFIER = Attribute("""
    Identifier which is used to select the appropriate object class.
    """)
    
    def serialize(serializer): #@NoSelf
        """ This method is called whenever the object should be serialized.
            
            @param serializer:  Serializer which provides methods to serialize base types
                                and into which the message has to be serialized.
            @type  serializer:  Comm.Message.SerializerUtil.Serializer
            
            @raise:     SerializationError
        """
    
    def deserialize(serializer): #@NoSelf
        """ This method is called whenever the object should be deserialized.
            
            This method is has to be implemented as a classmthod and should, therefore,
            return a new class instance!
            
            @param serializer:  Serializer which provides methods to deserialize base types
                                and from which the message has to be deserialized.
            @type  serializer:  Comm.Message.SerializerUtil.Serializer
            
            @return:    New class instance.
            
            @raise:     SerializationError
        """
