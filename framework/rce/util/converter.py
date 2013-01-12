#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     converter.py
#     
#     This file is part of the RoboEarth Cloud Engine framework.
#     
#     This file was originally created for RoboEearth
#     http://www.roboearth.org/
#     
#     The research leading to these results has received funding from
#     the European Union Seventh Framework Programme FP7/2007-2013 under
#     grant agreement no248942 RoboEarth.
#     
#     Copyright 2012 RoboEarth
#     
#     Licensed under the Apache License, Version 2.0 (the "License");
#     you may not use this file except in compliance with the License.
#     You may obtain a copy of the License at
#     
#     http://www.apache.org/licenses/LICENSE-2.0
#     
#     Unless required by applicable law or agreed to in writing, software
#     distributed under the License is distributed on an "AS IS" BASIS,
#     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#     See the License for the specific language governing permissions and
#     limitations under the License.
#     
#     \author/s: Dominique Hunziker 
#     
#     

# Python specific imports
import time
import datetime

try:
    from cStringIO import StringIO, InputType, OutputType
    from StringIO import StringIO as pyStringIO
    
    def _checkIsStringIO(obj):
        return isinstance(obj, (InputType, OutputType, pyStringIO))
except ImportError:
    from StringIO import StringIO
    
    def _checkIsStringIO(obj):
        return isinstance(obj, StringIO)

# ROS specific imports
import genpy.message
import rospy.rostime

# Custom imports
from rce.error import InternalError
from rce.util.interface import verifyClass
from rce.util.converters.interfaces import IROSConverter


class _DurationConverter(object):
    """ Convert ROS Duration type to JSON style and back.
    """
    def decode(self, _, data):
        """ Generate a rospy.rostime.Duration instance based on the given data
            which should be a string representation of a float.
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
    """ Convert ROS Time type to JSON style and back.
    """
    def decode(self, _, data):
        """ Generate a rospy.rostime.Time instance based on the given data of
            the form 'YYYY-MM-DDTHH:MM:SS.mmmmmm' (ISO 8601).
        """
        if '+' in data:
            data = data[:data.index('+')]

        try:
            dt = datetime.datetime(year=int(data[0:4]),
                                   month=int(data[5:7]),
                                   day=int(data[8:10]),
                                   hour=int(data[11:13]),
                                   minute=int(data[14:16]),
                                   second=int(data[17:19]),
                                   microsecond=int(data[20:]))
        except ValueError:
            return rospy.rostime.Time()

        return rospy.rostime.Time.from_sec(time.mktime(dt.timetuple()))

    def encode(self, rosMsg):
        """ Transform the rospy.rostime.Time instance to a string of the form
            'YYYY-MM-DDTHH:MM:SS.mmmmmm' (ISO 8601).
        """
        try:
            dt = datetime.datetime.fromtimestamp(rosMsg.to_sec())
        except AttributeError:
            raise TypeError('Received object is not a Time instance.')

        return (dt.isoformat(), {})


class Converter(object):
    """ This class is used to provide a possibility to convert a ROS message to
        a JSON compatible format and back.

        To add customized Converters use the method 'addCustomConverter' and
        the class must implement the interface 'IROSConverter'.
        As an example view the class ImageConverter.
    """
    _BASE_TYPES = { 'bool'    : bool,
                    'byte'    : int,
                    'char'    : int,
                    'uint8'   : int,
                    'int8'    : int,
                    'uint16'  : int,
                    'int16'   : int,
                    'uint32'  : int,
                    'int32'   : int,
                    'uint64'  : long,
                    'int64'   : long,
                    'float32' : float,
                    'float64' : float,
                    'string'  : str }

    _SPECIAL_TYPES = {  'time'     : _TimeConverter,
                        'duration' : _DurationConverter }
    
    def __init__(self, loader):
        """ Initialize the Converter.
            
            @param loader:      Used loader for ROS resources.
            @type  loader:      Loader
        """
        self._loader = loader
        self._customTypes = {}
    
    def addCustomConverter(self, converter):
        """ Register a new custom Converter.
            
            @raise:     errors.InternalError, util.interfaces.InterfaceError
        """
        verifyClass(IROSConverter, converter)
        
        if converter.MESSAGE_TYPE in self._customTypes:
            raise InternalError('There are multiple Converters given for '
                                'message type "{0}".'.format(
                                    converter.MESSAGE_TYPE))
        
        args = converter.MESSAGE_TYPE.split('/')
        
        if len(args) != 2:
            raise InternalError('msg type is not valid. Has to be of the from '
                                'pkg/msg, i.e. std_msgs/Int8.')
        
        self._customTypes[converter.MESSAGE_TYPE] = (converter,
            self._loader.loadMsg(*args))
    
    def removeCustomConverter(self, msgType):
        """ Unregister a custom Converter.
            
            @param msgType:     Message type of ROS message as a string, i.e.
                                'std_msgs/Int8', for which the converter should
                                be removed.
            @type  msgType:     str
        """
        try:
            del self._customTypes[msgType]
        except KeyError:
            InternalError('Tried to remove a custom converter which was '
                          'never added.')
    
    def _stringify(self, _, obj):
        """ Internally used method to make sure that strings are of type str
            and not of type unicode.
        """
        if isinstance(obj, unicode):
            return obj.encode('utf-8')
        else:
            return obj
    
    def _encode(self, rosMsg):
        """ Internally used method which is responsible for the heavy lifting.
        """
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
            elif slotType in self._customTypes:
                convFunc = self._customTypes[slotType][0]().encode
            else:
                convFunc = self._encode
            
            try:
                if listBool:
                    data[slotName] = map(convFunc, field)
                else:
                    data[slotName] = convFunc(field)
            except ValueError as e:
                raise ValueError('{0}.{1}: {2}'.format(
                                     rosMsg.__class__.__name__, slotName, e))

        return data
    
    def encode(self, rosMsg):
        """ Generate JSON compatible data from a ROS message.

            @param rosMsg:  The ROS message instance which should be converted.
            @type  rosMsg:  ROS message instance

            @return:    Dictionary containing the parsed message. The basic
                        form does map each field in the ROS message to a key /
                        value pair in the returned data dict. Binaries are
                        added as StringIO instances.
            @rtype:     {}

            @raise:     TypeError, ValueError
        """
        if not isinstance(rosMsg, genpy.message.Message):
            raise TypeError('Given rosMsg object is not an instance of '
                            'genpy.message.Message.')
        
        for converter, cls in self._customTypes.itervalues():
            if isinstance(rosMsg, cls):
                return converter().encode(rosMsg)
        
        return self._encode(rosMsg)
    
    def _decode(self, msgCls, data):
        """ Internally used method which is responsible for the heavy lifting.
        """
        rosMsg = msgCls()

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
                raise TypeError('Given data does not match the definition of '
                                'the ROS message.')
            
            if slotType in Converter._BASE_TYPES:
                convFunc = self._stringify
                slotCls = None
            elif slotType in Converter._SPECIAL_TYPES:
                convFunc = Converter._SPECIAL_TYPES[slotType]().decode
                slotCls = None
            elif slotType in self._customTypes and _checkIsStringIO(field):
                convFunc = self._customTypes[slotType][0]().decode
                slotCls = None
            else:
                convFunc = self._decode
                slotCls = self._loader.loadMsg(*slotType.split('/'))
            
            if listBool:
                msgData = map(lambda ele: convFunc(slotCls, ele), field)
            else:
                msgData = convFunc(slotCls, field)
            
            setattr(rosMsg, slotName, msgData)

        return rosMsg
    
    def decode(self, msgCls, data):
        """ Generate a ROS message from JSON compatible data.

            @param msgCls:  ROS message class into which the decoded data
                            should filled.
            @type  msgCls:  ROS Message class

            @param data:    Dictionary with keys matching the fields in the
                            desired ROS message. Binary files should be
                            included as StringIO instances.
            @param data:    { str : {} }

            @return:    ROS message of type rosMsg containing the given data.
            @rtype:     ROS message of type rosMsg

            @raise:     TypeError, ValueError
        """
        if _checkIsStringIO(data):
            for converter, cls in self._customTypes.itervalues():
                if msgCls == cls:
                    return converter().decode(msgCls, data)
        
        return self._decode(msgCls, data)
