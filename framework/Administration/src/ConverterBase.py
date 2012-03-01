#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       ConverterBase.py
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
import roslib.message
import roslib.rostime

# Python specific imports
import datetime

# Custom imports
import settings

class _DurationConverter(object):
    """ Convert ROS Duration type to django style and back.
    """
    def decode(self, rosMsgType, data, files):
        """ Generate a roslib.rostime.Duration instance based on the
            given data which should be a string representation of a
            float.
        """
        return roslib.rostime.Duration.from_sec(float(data))
    
    def encode(self, rosMsg, basename):
        """ Transform the roslib.rostime.Duration instance to a float.
        """
        try:
            return (rosMsg.to_sec(), {})
        except AttributeError:
            raise TypeError('Received object is not a Duration instance.')

class _TimeConverter(object):
    """ Convert ROS Time type to django style and back.
    """
    def decode(self, rosMsgType, data, files):
        """ Generate a roslib.rostime.Time instance based on the given
            data of the form 'YYYY-MM-DDTHH:MM:SS.mmmmmm' (ISO 8601). 
        """
        if '+' in data:
            data = data[:timeStr.index('+')]
        
        try:
            dt = datetime.datetime( year=int(data[0:4]), month=int(data[5:7]), day=int(data[8:10]), \
                                    hour=int(data[11:13]), minute=int(data[14:16]), second=int(data[17:19]), \
                                    microsecond=int(data[20:]) )
        except ValueError:
            return roslib.rostime.Time()
        
        return roslib.rostime.Time.from_sec(time.mktime(dt.timetuple()))
    
    def encode(self, rosMsg, basename):
        """ Transform the roslib.rostime.Time instance to a string of the
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
    
    def encode(self, rosMsg, basename=settings.REFERENCE_SEPARATOR):
        """ Generate django compatible data from a ROS message.
            
            @param rosMsg:  The ROS message instance which should be
                            converted.
            @type  rosMsg:  ROS message instance
            
            @param basename:    The keys used for the files dict have to
                                be generated with basename+individualKey.
            @type  basename:    str
            
            @return:    Tuple of the form (data, files) where data is
                        json encoding compatible string. The basic form
                        does map each field in the ROS message to a key-
                        value pair in the returned data dict. files is
                        a dict containing cStringIO.StringO objects as
                        values. The keys of the files dict are used as
                        references in the data string.
            @rtype:     ({}, {})
            
            @raise:     TypeError, ValueError
        """
        if not isinstance(rosMsg, roslib.message.Message):
            raise TypeError('Given rosMsg object is not an instance of roslib.message.Message.')
        
        data = {}
        files = {}
        
        for (slotName, slotType) in zip(rosMsg.__slots__, rosMsg._slot_types):
            if '[]' == slotType[-2:]:
                listBool = True
                slotType = slotType[:-2]
            else:
                listBool = False
            
            if slotType in Converter._BASE_TYPES:
                if listBool:
                    data[slotName] = map(Converter._BASE_TYPES[slotType], getattr(rosMsg, slotName))
                else:
                    data[slotName] = Converter._BASE_TYPES[slotType](getattr(rosMsg, slotName))
            else:
                try:
                    conv = Converter._SPECIAL_TYPES[slotType]()
                    convFunc = conv.encode
                except KeyError:
                    convFunc = self.encode
                
                if listBool:
                    fieldData = getattr(rosMsg, slotName)
                    data[slotName] = [None]*len(fieldData)
                    
                    for i in xrange(len(fieldData)):
                        data[slotName][i], subFiles = convFunc(fieldData[i], '{0}{1}_{2}{3}'.format(basename, slotName, i, settings.REFERENCE_SEPARATOR))
                        
                        for key in subFiles:
                            if key in files:
                                raise ValueError('Multiple values for the same key.')
                            
                            files[key] = subFiles[key]
                else:
                    data[slotName], subFiles = convFunc(getattr(rosMsg, slotName), '{0}{1}{2}'.format(basename, slotName, settings.REFERENCE_SEPARATOR))
                    
                    for key in subFiles:
                        if key in files:
                            raise ValueError('Multiple values for the same key.')
                        
                        files[key] = subFiles[key]
        
        return (data, files)
    
    def decode(self, rosMsgType, data, files):
        """ Generate a ROS message from django compatible data.
            
            @param rosMsgType:  String defining the ROS message type, i.e.
                                std_msgs/String.
            @type  rosMsgType:  str
            
            @param data:    Dictionary with keys matching the fields in
                            the desired ROS message.
            @param data:    { str : {} }
            
            @param files:   Dictionary with keys matching references from
                            the data dict and values are cStringIO.StringO
                            instances. 
            @param files:   { str : cStringIO.StringO instance }
            
            @return:    ROS message of type @param rosMsg containing the
                        given data.
            @rtype:     ROS message of type @param rosMsg
            
            @raise:     TypeError, ValueError
        """
        msgCls = roslib.message.get_message_class(rosMsgType)
        
        if not msgCls:
            msgCls = roslib.message.get_service_class(rosMsgType)
        
        if not msgCls:
            raise ValueError('Given message class could not be loaded.')
        
        rosMsg = msgCls()
        
        for (slotName, slotType) in zip(rosMsg.__slots__, rosMsg._slot_types):
            if slotName not in data:
                continue
            
            if '[]' == slotType[-2:]:
                listBool = True
                slotType = slotType[:-2]
            else:
                listBool = False
            
            if slotType in Converter._BASE_TYPES:
                if listBool:
                    dataList = data[slotName]
                    
                    if not isinstance(dataList, list):
                        raise TypeError('Given data does not match the definition of the ROS message.')
                    
                    field = [None]*len(dataList)
                    
                    for i in xrange(len(dataList)):
                        if isinstance(dataList[i], unicode):
                            field[i] = dataList[i].encode('utf-8')
                        else:
                            field[i] = dataList[i]
                else:
                    if isinstance(data[slotName], unicode):
                        field = data[slotName].encode('utf-8')
                    else:
                        field = data[slotName]
                    
                    setattr(rosMsg, slotName, field)
            else:
                try:
                    conv = Converter._SPECIAL_TYPES[slotType]()
                    convFunc = conv.decode
                except KeyError:
                    convFunc = self.decode
                
                if listBool:
                    msgData = [None]*len(data[slotName])
                    
                    for i in xrange(len(msgData)):
                        msgData[i] = convFunc(slotType, data[slotName][i], files)
                    
                    setattr(rosMsg, slotName, msgData)
                else:
                    setattr(rosMsg, slotName, convFunc(slotType, data[slotName], files))
        
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
        
        # Create an instance of the converter and remember it
        if convClass.MESSAGE_TYPE in Converter._SPECIAL_TYPES:
            raise ValueError('There are multiple Converters given for message type {0}'.format(convClass.MESSAGE_TYPE))
        
        Converter._SPECIAL_TYPES[convClass.MESSAGE_TYPE] = convClass

_initialize_Converter()

del _initialize_Converter
