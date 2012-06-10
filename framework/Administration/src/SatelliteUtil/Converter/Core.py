#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Core.py
#       
#       Copyright 2011 dominique hunziker <dominique.hunziker@gmail.com>
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
import genpy.message
import rospy.rostime

# zope specific imports
from zope.interface.verify import verifyClass
from zope.interface.exceptions import Invalid

# Python specific imports
import time
import datetime

try:
    from cStringIO import StringIO, InputType, OutputType
    
    def _checkIsStringIO(obj):
        return isinstance(obj, (InputType, OutputType))
except ImportError:
    from StringIO import StringIO
    
    def _checkIsStringIO(obj):
        return isinstance(obj, StringIO)

# Custom imports
import settings
from Exceptions import InternalError
from Interfaces import IROSConverter

class _DurationConverter(object):
    """ Convert ROS Duration type to django style and back.
    """
    def decode(self, rosMsgType, data):
        """ Generate a rospy.rostime.Duration instance based on the
            given data which should be a string representation of a
            float.
        """
        return rospy.rostime.Duration.from_sec(float(data))

    def encode(self, rosMsg):
        """ Transform the rospy.rostime.Duration instance to a float.
        """
        try:
            return (rosMsg.to_sec(), {})
        except AttributeError:
            raise TypeError('Received object is not a Duration instance.')

class _TimeConverter(object):
    """ Convert ROS Time type to django style and back.
    """
    def decode(self, rosMsgType, data):
        """ Generate a rospy.rostime.Time instance based on the given
            data of the form 'YYYY-MM-DDTHH:MM:SS.mmmmmm' (ISO 8601).
        """
        if '+' in data:
            data = data[:data.index('+')]

        try:
            dt = datetime.datetime(year=int(data[0:4]), month=int(data[5:7]), day=int(data[8:10]), \
                                    hour=int(data[11:13]), minute=int(data[14:16]), second=int(data[17:19]), \
                                    microsecond=int(data[20:]))
        except ValueError:
            return rospy.rostime.Time()

        return rospy.rostime.Time.from_sec(time.mktime(dt.timetuple()))

    def encode(self, rosMsg):
        """ Transform the rospy.rostime.Time instance to a string of the
            form 'YYYY-MM-DDTHH:MM:SS.mmmmmm' (ISO 8601).
        """
        try:
            dt = datetime.datetime.fromtimestamp(rosMsg.to_sec())
        except AttributeError:
            raise TypeError('Received object is not a Time instance.')

        return (dt.isoformat(), {})

class Converter(object):
    """ This class is used to provide a possibility to convert a ROS
        message to a Django compatible format and back.

        To add customized Converters add the corresponding class to the
        settings.CONVERTER_CLASSES tuple.
        The class must define the class variable MESSAGE_TYPE which has
        to contain the corresponding type which should be handled by the
        given Converter, i.e. 'std_msgs/String'.
        Furthermore, the class must provide the methods decode and encode
        which must have the same functionality, arguments and return
        values as the methods of this class.
        As an example view the classes _DurationConverter and _TimeConverter.
    """
    _BASE_TYPES = { 'bool' : bool,
                    'byte' : int,
                    'char' : int,
                    'uint8' : int,
                    'int8' : int,
                    'uint16' : int,
                    'int16' : int,
                    'uint32' : int,
                    'int32' : int,
                    'uint64' : long,
                    'int64' : long,
                    'float32' : float,
                    'float64' : float,
                    'string' : str }

    _SPECIAL_TYPES = {  'time' : _TimeConverter,
                        'duration' : _DurationConverter }
    
    _CUSTOM_TYPES = {} 
    
    def _stringify(self, objType, obj):
        """ Internally used method to make sure that strings are of type str and not unicode.
        """
        if isinstance(obj, unicode):
            return obj.encode('utf-8')
        else:
            return obj
    
    def encode(self, rosMsg):
        """ Generate django compatible data from a ROS message.

            @param rosMsg:  The ROS message instance which should be converted.
            @type  rosMsg:  ROS message instance

            @return:    Dictionary containing the parsed message. The basic form does map
                        each field in the ROS message to a key / value pair in the returned
                        data dict. Binaries are added as StringIO instances.
            @rtype:     {}

            @raise:     TypeError, ValueError
        """
        if not isinstance(rosMsg, genpy.message.Message):
            raise TypeError('Given rosMsg object is not an instance of genpy.message.Message.')

        data = {}

        for (slotName, slotType) in zip(rosMsg.__slots__, rosMsg._slot_types):
            if '[]' == slotType[-2:]:
                listBool = True
                slotType = slotType[:-2]
            else:
                listBool = False
            
            field = getattr(rosMsg, slotName)
            
            if slotType in Converter._BASE_TYPES:
                convFunc = Converter._BASE_TYPES[slotType]
            elif slotType in Converter._SPECIAL_TYPES:
                convFunc = Converter._SPECIAL_TYPES[slotType]().encode
            elif slotType in Converter._CUSTOM_TYPES and _checkIsStringIO(field):
                convFunc = Converter._CUSTOM_TYPES[slotType]().encode
            else:
                convFunc = self.encode

            if listBool:
                data[slotName] = map(convFunc, field)
            else:
                data[slotName] = convFunc(field)

        return data

    def decode(self, MsgCls, data):
        """ Generate a ROS message from django compatible data.

            @param MsgCls:  ROS message class which should into which the decoded
                            data should filled.
            @type  MsgCls:  ROS Message class

            @param data:    Dictionary with keys matching the fields in
                            the desired ROS message.
                            Binary files should be included as StringIO instances.
            @param data:    { str : {} }

            @return:    ROS message of type @param rosMsg containing the
                        given data.
            @rtype:     ROS message of type @param rosMsg

            @raise:     TypeError, ValueError
        """
        rosMsg = MsgCls()

        for (slotName, slotType) in zip(rosMsg.__slots__, rosMsg._slot_types):
            if slotName not in data:
                continue

            if '[]' == slotType[-2:]:
                listBool = True
                slotType = slotType[:-2]
            else:
                listBool = False
            
            field = data[slotName]
            
            if listBool and not isinstance(field, list):
                raise TypeError('Given data does not match the definition of the ROS message.')
            
            if slotType in Converter._BASE_TYPES:
                convFunc = self._stringify
            elif slotType in Converter._SPECIAL_TYPES:
                convFunc = Converter._SPECIAL_TYPES[slotType]().decode
            elif slotType in Converter._CUSTOM_TYPES:
                convFunc = Converter._CUSTOM_TYPES[slotType]().decode
            else:
                convFunc = self.decode

            if listBool:
                msgData = map(lambda ele: convFunc(slotType, ele), field)
            else:
                msgData = convFunc(slotType, field)
            
            setattr(rosMsg, slotName, msgData)

        return rosMsg

def _initialize_Converter():
    for converter in settings.CONVERTER_CLASSES:
        # Get correct path/name of the converter
        convList = converter.rsplit('.', 1)
        module = convList[0]
        className = convList[1]

        # Load the converter
        mod = __import__(module, fromlist=[className])
        convClass = getattr(mod, className)
        
        try:
            verifyClass(IROSConverter, convClass)
        except Invalid as e:
            raise InternalError(
                'Verification of the class "{0}" for the Interface "IROSConverter" failed: {1}'.format(
                    convClass.__name__,
                    e
                )
            )
        
        if convClass.MESSAGE_TYPE in Converter._CUSTOM_TYPES:
            raise InternalError(
                'There are multiple Converters given for message type "{0}".'.format(
                    convClass.MESSAGE_TYPE
                )
            )
        
        Converter._CUSTOM_TYPES[convClass.MESSAGE_TYPE] = convClass

_initialize_Converter()

del _initialize_Converter
