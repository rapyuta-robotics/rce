#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Node.py
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
from Parameter import IntParam, StrParam, FloatParam, BoolParam, FileParam

class Node(object):
    """ Class which represents a node.
    """
    implements(ISerializable)
    
    IDENTIFIER = ComponentDefinition.NODE
    
    _PARAMS = { IntParam.IDENTIFIER   : IntParam,
                StrParam.IDENTIFIER   : StrParam,
                FloatParam.IDENTIFIER : FloatParam,
                BoolParam.IDENTIFIER  : BoolParam,
                FileParam.IDENTIFIER  : FileParam }
    
    def __init__(self, pkg, exe, namespace='', params=[]):
        """ Initialize the Node instance.

            @param pkg:     Name of the package where this node can be found.
            @type  pkg:     str

            @param exe:     Name of the executable which is used.
            @type  exe:     str

            @param namespace:   Namespace in which this node should be launched.
            @type  namespace:   str

            @param params:  Parameters which have to be set for this node.
            @type  params:  [ Parameter ]
        """
        self._pkg = pkg
        self._exe = exe
        self._namespace = namespace
        self._params = params
    
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
        """ Namespace in which the node should be launched. """
        return self._namespace
    
    @property
    def params(self):
        """ Parameters which are necessary to launch the node. """
        return tuple(self._params)
    
    def serialize(self, s):
        """ Serialize the Node object.
            
            @param s:   Serializer instance into which the message should be serialized.
            @type  s:   Serializer
            
            @raise:     SerializationError
        """
        s.addElement(self._pkg)
        s.addElement(self._exe)
        s.addElement(self._namespace)
        s.addInt(len(self._params))
        
        for param in self._params:
            s.addIdentifier(param.IDENTIFIER, 1)
            param.serialize(s)
    
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
                    [ cls._PARAMS[s.getIdentifier(1)].deserialize(s) for _ in xrange(s.getInt()) ] )

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
