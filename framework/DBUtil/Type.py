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
#       \author/s: Dominique Hunziker <dominique.hunziker@gmail.com> 
#       
#       

# zope specific imports
from zope.interface import implements

# Custom imports
from Exceptions import SerializationError
from Comm.Message.Interfaces import IContentSerializer
from Comm.Message import MsgTypes

class DBRequestMessage(object):
    """ Message type to request a database query.
        
        The fields are:
            method  Method which should be called in the DBHandler
            args    Keyworded arguments as a serialized json string
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.DB_REQUEST
    
    def serialize(self, s, data):
        try:
            s.addElement(data['id'])
            s.addElement(data['method'])
            s.addElement(data['args'])
            s.addElement(data['kw'])
        except KeyError as e:
            raise SerializationError('Could not serialize message of type DBRequestMessage: {0}'.format(e))
    
    def deserialize(self, s):
        return { 'id'     : s.getElement(),
                 'method' : s.getElement(),
                 'args'   : s.getElement(),
                 'kw'     : s.getElement() }

class DBResponseMessage(object):
    """ Message type to send back a response for a database query.
        
        The fields are:
            resp    Response of the database query as a serialized json string
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.DB_RESPONSE
    
    def serialize(self, s, data):
        try:
            s.addElement(data['id'])
            s.addBool(data['status'])
            s.addElement(data['resp'])
        except KeyError as e:
            raise SerializationError('Could not serialize message of type DBResponseMessage: {0}'.format(e))
    
    def deserialize(self, s):
        return { 'id'     : s.getElement(),
                 'status' : s.getBool(),
                 'resp'   : s.getElement() }
