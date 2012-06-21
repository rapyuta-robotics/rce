#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       StdType.py
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
from Exceptions import SerializationError
from Interfaces import IContentSerializer
import MsgTypes

class InitMessage(object):
    """ Message type to initialize the communication.
        
        The fields are:
            dest    CommID of destination
            origin  CommID of origin
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.INIT_REQUEST
    
    def serialize(self, s, data):
        try:
            s.addElement(data['remoteID'])
        except KeyError as e:
            raise SerializationError('Could not serialize message of type InitMessage: {0}'.format(e))
    
    def deserialize(self, s):
        return { 'remoteID' : s.getElement() }

class RouteMessage(object):
    """ Message type to provide other nodes with routing information.
        
        Message format is a list of 2-tuples (DestID, Add (True) / Remove (False) flag)
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.ROUTE_INFO
    
    def serialize(self, s, data):
        if not isinstance(data, list):
            raise SerializationError('Content of the message has to be a list.')
        
        s.addInt(len(data))
        
        for element in data:
            if len(element) != 2:
                raise SerializationError('List element is not a 2-tuple.')
            
            try:
                s.addElement(element[0])
                s.addBool(element[1])
            except IndexError as e:
                raise SerializationError('Could not serialize element: {0}'.format(e))
    
    def deserialize(self, s):
        return [(s.getElement(), s.getBool()) for _ in xrange(s.getInt())]
