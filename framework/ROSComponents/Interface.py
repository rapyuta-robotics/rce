#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Interface.py
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
from Exceptions import InternalError
import ComponentDefinition
from Interfaces import ISerializable

class _InterfaceBase(object):
    """ Base class which represents an interface for a node.
    """
    implements(ISerializable)
    
    def __init__(self, interfaceName, interfaceTag, interfaceClass):
        """ Initialize the Interface instance.

            @param interfaceName:   ROS address of the interface.
            @type  interfaceName:   str

            @param interfaceTag:    Tag which is used to identify the interface.
            @type  interfaceTag:    str
            
            @param interfaceClass:  Message/Service class which is necessary to
                                    use the interface.
            @type  interfaceClass:  str
        """
        if isinstance(interfaceName, unicode):
            try:
                interfaceName = str(interfaceName)
            except UnicodeEncodeError:
                raise InternalError('The interface name {0} is not valid.'.format(interfaceName))
        
        if isinstance(interfaceClass, unicode):
            try:
                interfaceClass = str(interfaceClass)
            except UnicodeEncodeError:
                raise InternalError('The communication class {0} is not valid.'.format(interfaceClass))
        
        self._interfaceName = interfaceName
        self._interfaceTag = interfaceTag
        self._interfaceClass = interfaceClass
    
    @property
    def name(self):
        """ ROS name of the interface. """
        return self._interfaceName
    
    @property
    def tag(self):
        """ Tag which is used to identify the interface. """
        return self._interfaceTag
    
    @property
    def msgCls(self):
        """ Message class of the interface (equal to srvCls). """
        return self._interfaceClass
    
    @property
    def srvCls(self):
        """ Message class of the interface (equal to msgCls). """
        return self._interfaceClass
    
    def serialize(self, s):
        """ Serialize the Interface object.
            
            @param s:   Serializer instance into which the message should be serialized.
            @type  s:   Serializer
            
            @raise:     SerializationError
        """
        s.addElement(self._interfaceName)
        s.addElement(self._interfaceTag)
        s.addElement(self._interfaceClass)
    
    @classmethod
    def deserialize(cls, s):
        """ Deserialize the Interface object.
            
            @param s:   Serializer instance from which the message should be deserialized.
            @type  s:   Serializer
            
            @raise:     SerializationError
        """
        return cls(s.getElement(), s.getElement(), s.getElement())

class ServiceInterface(_InterfaceBase):
    """ Class which represents a service interface.
    """
    IDENTIFIER = ComponentDefinition.INTERFACE_SRV

class PublisherInterface(_InterfaceBase):
    """ Class which represents a publisher interface.
    """
    IDENTIFIER = ComponentDefinition.INTERFACE_PUB

class SubscriberInterface(_InterfaceBase):
    """ Class which represents a subscriber interface.
    """
    IDENTIFIER = ComponentDefinition.INTERFACE_SUB
