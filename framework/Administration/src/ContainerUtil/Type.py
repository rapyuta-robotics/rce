#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Type.py
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
from zope.interface import implements

# Custom imports
from Exceptions import SerializationError
from Comm.Message.Interfaces import IContentSerializer #@UnresolvedImport
from Comm.Message import MsgTypes

class StartContainerMessage(object):
    """ Message type to start a container.
        
        The fields are:
            commID  CommID which is used to identify the container
            home    Home directory which should be used
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.CONTAINER_START
    
    def serialize(self, s, data):
        try:
            s.addElement(data['commID'])
            s.addElement(data['home'])
        except KeyError as e:
            raise SerializationError('Could not serialize message of type StartContainer: {0}'.format(e))
    
    def deserialize(self, s):
        return { 'commID' : s.getElement(),
                 'home'   : s.getElement() }

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
            raise SerializationError('Could not serialize message of type StopContainer: {0}'.format(e))
    
    def deserialize(self, s):
        return { 'commID' : s.getElement() }

class ContainerStatusMessage(object):
    """ Message type to # TODO: What exactly is this message used for?
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.CONTAINER_STATUS
    
    def serialize(self, s, data):
        # TODO: What is part of the message
        return ''
    
    def deserialize(self, data):
        # TODO: What is part of the message
        msg = {}
        
        #try:
        #    start = 0
        #    end = MsgDef.I_LEN
        #    length, = MsgDef.I_STRUCT.unpack(data[start:end])
        #    start = end
        #    end += length
        #    msg['commID'] = data[start:end]
        #except StructError as e:
        #    raise SerializationError('Could not deserialize message of type ContainerStatus: {0}'.format(e))
        
        return msg
