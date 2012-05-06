#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       _Parameter.py
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

# ROS specific imports
import rospy

# Python specific imports
import os
from struct import error as StructError

# Custom imports
from Exceptions import InternalError, SerializationError
from ContentDefinition import STRUCT_I, LEN_I
from MiscUtility import mktempfile

class ParameterBase(object):
    """ Base class which represents the node configuration parameters.
        The parameters are constructed using the received information
        from ParameterBaseFactory.
    """
    __slots__ = ['_value', '_rosName', '_paramSet']

    def __init__(self, value, rosName):
        """ Initialize the Parameter instance.

            @param value:   Value of the parameter which will be sent
                            to the parameter server.
            @type  value:   Depends on subclass

            @param rosName:     Name under which the parameter is stored.
            @type  rosName:     str
        """
        self._value = value
        self._rosName = rosName
        self._paramSet = False

    @classmethod
    def deserialize(cls, data):
        """ Deserialize a Parameter.
        """
        try:
            start = 0
            end = LEN_I
            length, = STRUCT_I.unpack(data[start:end])
            start = end
            end += length
            val = data[start:end]

            start = end
            end += LEN_I
            length, = STRUCT_I.unpack(data[start:end])
            start = end
            end += length
            name = data[start:end]
        except StructError as e:
            raise SerializationError('Could not deserialize Parameter: {0}'.format(e))

        return cls(val, name)

    def setParam(self):
        """ Set the parameter value to the ROS Parameter Server.

            @raise:    InternalError if Parameter can not be added to server.
        """
        if self._paramSet:
            return

        try:
            if rospy.has_param(self._rosName):
                raise InternalError('Parameter already exists.')

            rospy.set_param(self._rosName, self._value)
        except rospy.ROSException as e:
            raise InternalError('ROS Parameter Server reported an error: {0}'.format(str(e)))

        self._paramSet = True

    def __del__(self):
        """ Destructor.
        """
        if self._paramSet and self._rosName:
            try:
                rospy.delete_param(self._rosName)
            except rospy.ROSException:
                pass

class IntParam(ParameterBase):
    """ Represents an int parameter for a node.
    """
    pass

class StrParam(ParameterBase):
    """ Represents a str parameter for a node.
    """
    pass

class FloatParam(ParameterBase):
    """ Represents a float parameter for a node.
    """
    pass

class BoolParam(ParameterBase):
    """ Represents a bool parameter for a node.
    """
    pass

class FileParam(ParameterBase):
    """ Represents a file parameter for a node.
    """
    __slots__ = ['_fileCreated']

    def __init__(self, value, rosName):
        """ Initialize the Parameter instance.

            @param value:   Value of the parameter which will be sent
                            to the parameter server.
            @type  value:   Depends on subclass

            @param rosName:     Name under which the parameter is stored.
            @type  rosName:     str
        """
        super(FileParam, self).__init__(value, rosName)

        self._fileCreated = False

    def setParam(self, rosName):
        if self._paramSet:
            return

        try:
            if rospy.has_param(self._rosName):
                raise InternalError('Parameter already exists.')
        except rospy.ROSException as e:
            raise InternalError('ROS Parameter Server reported an error: {0}'.format(str(e)))

        (f, fileName) = mktempfile()
        f.write(self.value)
        f.close()

        self._value = fileName

        if not os.path.isfile(self._value):
            raise InternalError('Could not find file.')

        self._fileCreated = True

        try:
            rospy.set_param(self._rosName, self._value)
        except rospy.ROSException as e:
            raise InternalError('ROS Parameter Server reported an error: {0}'.format(str(e)))

        self._paramSet = True

    setParam.__doc__ = ParameterBase.setParam.__doc__

    def __del__(self):
        """ Destructor.
        """
        if self._fileCreated:
            try:
                os.remove(self._value)
            except OSError:
                pass

        super(FileParam, self).__del__()
