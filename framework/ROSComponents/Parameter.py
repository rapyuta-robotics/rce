#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Parameter.py
#       
#       This file is part of the RoboEarth Cloud Engine framework.
#       
#       This file was originally created for RoboEearth - http://www.roboearth.org/
#       The research leading to these results has received funding from the European Union 
#       Seventh Framework Programme FP7/2007-2013 under grant agreement no248942 RoboEarth.
#       
#       Copyright 2012 RoboEarth
#       
#       Licensed under the Apache License, Version 2.0 (the "License");
#       you may not use this file except in compliance with the License.
#       You may obtain a copy of the License at
#       
#       http://www.apache.org/licenses/LICENSE-2.0
#       
#       Unless required by applicable law or agreed to in writing, software
#       distributed under the License is distributed on an "AS IS" BASIS,
#       WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#       See the License for the specific language governing permissions and
#       limitations under the License.
#       
#       \author/s: Dominique Hunziker <dominique.hunziker@gmail.com> 
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
