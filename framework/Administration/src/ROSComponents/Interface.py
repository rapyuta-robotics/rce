#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Interface.py
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
from Exceptions import InternalError
import ComponentDefinition #@UnresolvedImport
from Interfaces import ISerializable #@UnresolvedImport

class _InterfaceBase(object):
    """ Base class which represents an interface for a node.
    """
    implements(ISerializable)
    
    def __init__(self, interfaceName):
        """ Initialize the Interface instance.

            @param interfaceName:   ROS name of the interface.
            @type  interfaceName:   str
        """
        if isinstance(interfaceName, unicode):
            try:
                interfaceName = str(interfaceName)
            except UnicodeEncodeError:
                raise InternalError('The interface name {0} is not valid.'.format(interfaceName))
        
        self._interfaceName = interfaceName
    
    @property
    def interfaceName(self):
        """ ROS name of the interface. """
        return self._interfaceName
    
    def serialize(self, s):
        """ Serialize the Interface object.
            
            @param s:   Serializer instance into which the message should be serialized.
            @type  s:   Serializer
            
            @raise:     SerializationError
        """
        s.addElement(self._interfaceName)

    @classmethod
    def deserialize(cls, s):
        """ Deserialize the Interface object.
            
            @param s:   Serializer instance from which the message should be deserialized.
            @type  s:   Serializer
            
            @raise:     SerializationError
        """
        return cls(s.getElement())

class ServiceInterface(_InterfaceBase):
    """ Class which represents a service interface.
    """
    IDENTIFIER = ComponentDefinition.INTERFACE_SRV
    
    def __init__(self, interfaceName, srvClass):
        """ Initialize the Interface instance.

            @param interfaceName:   ROS name of the interface.
            @type  interfaceName:   str

            @param interfaceType:   Interface type.
            @type  interfaceType:   str

            @param srvClass:        Service class (only used for service interfaces).
            @type  srvClass:        str
        """
        super(ServiceInterface, self).__init__(interfaceName)
        
        if isinstance(srvClass, unicode):
            try:
                srvClass = str(srvClass)
            except UnicodeEncodeError:
                raise InternalError('The communication class {0} is not valid.'.format(cls))
        
        self._srvClass = srvClass
    
    @property
    def srvClass(self):
        """ Service class of the interface. """
        return self._srvClass
    
    def serialize(self, s):
        """ Serialize the ServiceInterface object.
            
            @param s:   Serializer instance into which the message should be serialized.
            @type  s:   Serializer
            
            @raise:     SerializationError
        """
        super(ServiceInterface, self).serialize(s)
        s.addElement(self._srvClass)

    @classmethod
    def deserialize(cls, s):
        """ Deserialize the ServiceInterface object.
            
            @param s:   Serializer instance from which the message should be deserialized.
            @type  s:   Serializer
            
            @raise:     SerializationError
        """
        return cls(s.getElement(), s.getElement())

class PublisherInterface(_InterfaceBase):
    """ Class which represents a publisher interface.
    """
    IDENTIFIER = ComponentDefinition.INTERFACE_PUB

class SubscriberInterface(_InterfaceBase):
    """ Class which represents a subscriber interface.
    """
    IDENTIFIER = ComponentDefinition.INTERFACE_SUB
