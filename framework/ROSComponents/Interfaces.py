#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Interfaces.py
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
