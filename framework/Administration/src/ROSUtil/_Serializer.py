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

# Custom imports
from Exceptions import SerializationError

from ._ParserBase import ParserBase

def serialize(obj):
    """ Serialize a Parser instance.

        @param obj:     Parser instance which should be serialized.
        @type  obj:     Subclass of ParserBase

        @return:    Serialized object as a string.
        @rtype:     str

        @raise:     SerializationError if the object is not a subclass
                    of ParserBase.
    """
    if not isinstance(obj, ParserBase):
        raise SerializationError('The object is not a subclass of ParserBase.')

    return obj.serialize()

def deserialize(data, clsMap):
    """ Deserializes a prior serialized string from a Parser.

        @param data:    Data string which contains the serialized
                        object.
        @type  data:    str

        @param clsMap:  Map which contains the valid ContentDefinitions
                        as key and their matching classes as value.
        @type  clsMap:  dict

        @return:    Deserialized object

        @raise:     SerializationError if the data string could not
                    be deserialized.
    """
    cls = clsMap.get(data[:1], None)

    if not cls:
        raise SerializationError('Data string does not contain a valid ROS element.')

    return cls.deserialize(data[1:])
