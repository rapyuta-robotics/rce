#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Type.py
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

# zope specific imports
from zope.interface import implements

# Custom imports
from Exceptions import SerializationError
from Comm.Message.Interfaces import IContentSerializer
from Comm.Message import MsgTypes

class StartContainerMessage(object):
    """ Message type to start a container.
        
        The fields are:
            commID  CommID which is used to identify the container
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.CONTAINER_START
    
    def serialize(self, s, data):
        try:
            s.addElement(data['commID'])
        except KeyError as e:
            raise SerializationError('Could not serialize message of type StartContainer. Missing key: {0}'.format(e))
    
    def deserialize(self, s):
        return { 'commID' : s.getElement() }

class StopContainerMessage(object):
    """ Message type to stop a container.
        
        The fields are:
            commID  CommID which is used to identify the container
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.CONTAINER_STOP
    
    def serialize(self, s, data):
        try:
            s.addElement(data['commID'])
        except KeyError as e:
            raise SerializationError('Could not serialize message of type StopContainer. Missing key: {0}'.format(e))
    
    def deserialize(self, s):
        return { 'commID' : s.getElement() }
