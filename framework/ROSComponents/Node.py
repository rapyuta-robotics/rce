#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Node.py
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
#       \author/s: 
#       
#       

# zope specific imports
from zope.interface import implements

# Custom imports
import ComponentDefinition
from Interfaces import ISerializable

class Node(object):
    """ Class which represents a node.
    """
    implements(ISerializable)
    
    IDENTIFIER = ComponentDefinition.NODE
    
    def __init__(self, tag, pkg, exe, namespace):
        """ Initialize the Node instance.
            
            @param tag:     Tag which is to identify the node.
            @type  tag:     str

            @param pkg:     Name of the package where this node can be found.
            @type  pkg:     str

            @param exe:     Name of the executable which is used.
            @type  exe:     str
            
            @param namespace:   Namespace in which the node should be launched.
            @type  namespace:   str
        """
        self._tag = tag
        self._pkg = pkg
        self._exe = exe
        self._namespace = namespace
    
    @property
    def tag(self):
        """ Tag which is used to identify the node. """
        return self._tag
    
    @property
    def pkg(self):
        """ Package name in which the executable/node is located. """
        return self._pkg
    
    @property
    def exe(self):
        """ Name of the executable/node which should be launched. """
        return self._exe
    
    @property
    def namespace(self):
        """ Namespace of the node in the ROS environment. """
        return self._namespace
    
    def serialize(self, s):
        """ Serialize the Node object.
            
            @param s:   Serializer instance into which the message should be serialized.
            @type  s:   Serializer
            
            @raise:     SerializationError
        """
        s.addElement(self._tag)
        s.addElement(self._pkg)
        s.addElement(self._exe)
        s.addElement(self._namespace)
    
    @classmethod
    def deserialize(cls, s):
        """ Deserialize the Node object.
            
            @param s:   Serializer instance from which the message should be deserialized.
            @type  s:   Serializer
            
            @raise:     SerializationError
        """
        return cls( s.getElement(),
                    s.getElement(),
                    s.getElement(),
                    s.getElement() )

class NodeForwarder(object):
    """ Dummy class which represents a node. It is used to forward the data without deserializing
        it to the launcher.
    """
    implements(ISerializable)
    
    IDENTIFIER = ComponentDefinition.NODE
    
    def __init__(self, buf):
        """ Initialize the NodeForwarder.
            
            @param buf:     FIFO containing the serialized node data.
            @type  buf:     MessageFIFO
        """
        self._buf = buf
    
    def serialize(self, s):
        """ Add the buffered message data to the serializer.
            
            @param s:   Serializer instance into which the message should be serialized.
            @type  s:   Serializer
            
            @raise:     SerializationError
        """
        s.addFIFO(self._buf)
    
    @classmethod
    def deserialize(cls, s):
        """ Just save the serialized message.
            
            @param s:   Serializer instance from which the message should be deserialized.
            @type  s:   Serializer
            
            @raise:     SerializationError
        """
        return cls(s.getFIFO())
