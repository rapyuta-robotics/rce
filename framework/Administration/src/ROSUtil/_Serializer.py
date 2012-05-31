#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       _Serializer.py
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
from zope.interface.verify import verifyObject
from zope.interface.exceptions import Invalid

# Custom imports
from Exceptions import InternalError, SerializationError
from Interfaces import ISerializable #@UnresolvedImport

def serialize(serializer, obj):
    """ Serialize a ROS component instance.
        
        @param serializer:  Serializer instance into which the parser should be serialized.
        @type  serializer:  Comm.Message.SerializerUtil.Serializer

        @param obj:     Parser instance which should be serialized.
        @type  obj:     Subclass of ParserBase

        @raise:     InternalError if the object is not a subclass of ParserBase.
                    SerializationError can be raise by the serializer.
    """
    try:
        verifyObject(ISerializable, obj)
    except Invalid as e:
        raise InternalError(
            'Verification of the class "{0}" for the Interface "ISerializable" failed: {1}'.format(
                obj.__class__.__name__,
                e
            )
        )

    return obj.serialize(serializer)

def deserialize(serializer, clsMap):
    """ Deserializes a prior serialized string from a Parser.

        @param serializer:  Data string which contains the serialized object.
        @type  serializer:  Comm.Message.SerializerUtil.Serializer

        @param clsMap:  Map which contains the valid ContentDefinitions as key and
                        their matching classes as value.
        @type  clsMap:  dict

        @return:    Deserialized object

        @raise:     SerializationError if the message could not be deserialized.
    """
    cls = clsMap.get(serializer.getElement(1), None)
    
    if not cls:
        raise SerializationError('Data string does not contain a valid ROS element.')

    return cls.deserialize(serializer)
