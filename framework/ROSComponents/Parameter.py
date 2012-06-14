#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Parameter.py
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
from zope.interface import implements

# Custom imports
import ComponentDefinition

from Interfaces import ISerializable

class _ParamBase(object):
    """ Base class which represents a parameter for a node.
    """
    implements(ISerializable)
    
    def __init__(self, name, value):
        """ Initialize the Parameter instance.

            @param name:    Name under which the parameter is stored.
            @type  name:    str

            @param value:   Value of the parameter which will be sent
                            to the parameter server.
            @type  value:   Depends on subclass
        """
        self._name = name
        self._value = value
    
    @property
    def name(self):
        """ ROS name of the parameter. """
        return self._name
    
    @property
    def value(self):
        """ Value of the parameter. """
        return self._value

class IntParam(_ParamBase):
    """ Class which represents an integer parameter.
    """
    IDENTIFIER = ComponentDefinition.PARAM_INT
    
    def serialize(self, s):
        """ Serialize the IntParam object.
            
            @param s:   Serializer instance into which the message should be serialized.
            @type  s:   Serializer
            
            @raise:     SerializationError
        """
        s.addElement(self._name)
        s.addInt(self._value)

    @classmethod
    def deserialize(cls, s):
        """ Deserialize the IntParam object.
            
            @param s:   Serializer instance from which the message should be deserialized.
            @type  s:   Serializer
            
            @raise:     SerializationError
        """
        return cls(s.getElement(), s.getInt())

class StrParam(_ParamBase):
    """ Class which represents a string parameter.
    """
    IDENTIFIER = ComponentDefinition.PARAM_STR
    
    def serialize(self, s):
        """ Serialize the StrParam object.
            
            @param s:   Serializer instance into which the message should be serialized.
            @type  s:   Serializer
            
            @raise:     SerializationError
        """
        s.addElement(self._name)
        s.addElement(self._value)

    @classmethod
    def deserialize(cls, s):
        """ Deserialize the StrParam object.
            
            @param s:   Serializer instance from which the message should be deserialized.
            @type  s:   Serializer
            
            @raise:     SerializationError
        """
        return cls(s.getElement(), s.getElement())

class FloatParam(_ParamBase):
    """ Class which represents a float parameter.
    """
    IDENTIFIER = ComponentDefinition.PARAM_FLOAT
    
    def serialize(self, s):
        """ Serialize the FloatParam object.
            
            @param s:   Serializer instance into which the message should be serialized.
            @type  s:   Serializer
            
            @raise:     SerializationError
        """
        s.addElement(self._name)
        s.addFloat(self._value)

    @classmethod
    def deserialize(cls, s):
        """ Deserialize the FloatParam object.
            
            @param s:   Serializer instance from which the message should be deserialized.
            @type  s:   Serializer
            
            @raise:     SerializationError
        """
        return cls(s.getElement(), s.getFloat())

class BoolParam(_ParamBase):
    """ Class which represents a bool parameter.
    """
    IDENTIFIER = ComponentDefinition.PARAM_BOOL
    
    def serialize(self, s):
        """ Serialize the BoolParam object.
            
            @param s:   Serializer instance into which the message should be serialized.
            @type  s:   Serializer
            
            @raise:     SerializationError
        """
        s.addElement(self._name)
        s.addFloat(self._value)

    @classmethod
    def deserialize(cls, s):
        """ Deserialize the BoolParam object.
            
            @param s:   Serializer instance from which the message should be deserialized.
            @type  s:   Serializer
            
            @raise:     SerializationError
        """
        return cls(s.getElement(), s.getBool())

class FileParam(_ParamBase):
    """ Class which represents a file parameter.
    """
    IDENTIFIER = ComponentDefinition.PARAM_FILE
    
    def serialize(self, s):
        """ Serialize the FileParam object.
            
            @param s:   Serializer instance into which the message should be serialized.
            @type  s:   Serializer
            
            @raise:     SerializationError
        """
        s.addElement(self._name)
        s.addElement(self._value)

    @classmethod
    def deserialize(cls, s):
        """ Deserialize the FileParam object.
            
            @param s:   Serializer instance from which the message should be deserialized.
            @type  s:   Serializer
            
            @raise:     SerializationError
        """
        return cls(s.getElement(), s.getElement())
