#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       ContainerType.py
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

# twisted specific imports
from zope.interface import implements

# Python specific imports
from struct import error as StructError

try:
    from cStringIO import StringIO
except ImportError:
    from StringIO import StringIO

# Custom imports
from Exceptions import SerializationError
from Comm.Message.Interfaces import IContentSerializer
from Comm.Message.SerializerUtil import serializeDict, deserializeDict, serializeList, deserializeList
from Comm.Message import MsgDef
from Comm.Message import MsgTypes

class CreateEnvMessage(object):
    """ Message type to create a new environment.
        
        The fields are:
            commID  CommID which is used to identify the environment
            home    Home directory which should be used
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.ENV_CREATE
    
    def serialize(self, data):
        buf = StringIO()
        
        try:
            buf.write(MsgDef.I_STRUCT.pack(len(data['commID'])))
            buf.write(data['commID'])
            
            buf.write(MsgDef.I_STRUCT.pack(len(data['home'])))
            buf.write(data['home'])
        except KeyError as e:
            raise SerializationError('Could not serialize message of type CreateEnv: {0}'.format(e))
        
        return buf.getvalue()
    
    def deserialize(self, data):
        msg = {}
        
        try:
            start = 0
            end = MsgDef.I_LEN
            length, = MsgDef.I_STRUCT.unpack(data[start:end])
            start = end
            end += length
            msg['commID'] = data[start:end]
            
            start = end
            end += MsgDef.I_LEN
            length, = MsgDef.I_STRUCT.unpack(data[start:end])
            start = end
            end += length
            msg['home'] = data[start:end]
        except StructError as e:
            raise SerializationError('Could not deserialize message of type CreateEnv: {0}'.format(e))
        
        return msg

class DestroyEnvMessage(object):
    """ Message type to destroy an existing environment.
        
        The fields are:
            commID  CommID which is used to identify the environment
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.ENV_DESTROY
    
    def serialize(self, data):
        buf = StringIO()
        
        try:
            buf.write(MsgDef.I_STRUCT.pack(len(data['commID'])))
            buf.write(data['commID'])
        except KeyError as e:
            raise SerializationError('Could not serialize message of type DestroyEnv: {0}'.format(e))
        
        return buf.getvalue()
    
    def deserialize(self, data):
        msg = {}
        
        try:
            start = 0
            end = MsgDef.I_LEN
            length, = MsgDef.I_STRUCT.unpack(data[start:end])
            start = end
            end += length
            msg['commID'] = data[start:end]
        except StructError as e:
            raise SerializationError('Could not deserialize message of type DestroyEnv: {0}'.format(e))
        
        return msg

class StartContainerMessage(object):
    """ Message type to start a container.
        
        The fields are:
            commID  CommID which is used to identify the container
            home    Home directory which should be used
            # TODO: Add fields
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.CONTAINER_START
    
    def serialize(self, data):
        buf = StringIO()
        
        try:
            buf.write(MsgDef.I_STRUCT.pack(len(data['commID'])))
            buf.write(data['commID'])
            
            buf.write(MsgDef.I_STRUCT.pack(len(data['home'])))
            buf.write(data['home'])
            
            # TODO: Add additional fields
        except KeyError as e:
            raise SerializationError('Could not serialize message of type StartContainer: {0}'.format(e))
        
        return buf.getvalue()
    
    def deserialize(self, data):
        msg = {}
        
        try:
            start = 0
            end = MsgDef.I_LEN
            length, = MsgDef.I_STRUCT.unpack(data[start:end])
            start = end
            end += length
            msg['commID'] = data[start:end]
            
            start = end
            end += MsgDef.I_LEN
            length, = MsgDef.I_STRUCT.unpack(data[start:end])
            start = end
            end += length
            msg['home'] = data[start:end]
            
            # TODO: Add additional fields
        except StructError as e:
            raise SerializationError('Could not deserialize message of type StartContainer: {0}'.format(e))
        
        return msg

class StopContainerMessage(object):
    """ Message type to stop a container.
        
        The fields are:
            commID  CommID which is used to identify the container
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.CONTAINER_STOP
    
    def serialize(self, data):
        buf = StringIO()
        
        try:
            buf.write(MsgDef.I_STRUCT.pack(len(data['commID'])))
            buf.write(data['commID'])
        except KeyError as e:
            raise SerializationError('Could not serialize message of type StopContainer: {0}'.format(e))
        
        return buf.getvalue()
    
    def deserialize(self, data):
        msg = {}
        
        try:
            start = 0
            end = MsgDef.I_LEN
            length, = MsgDef.I_STRUCT.unpack(data[start:end])
            start = end
            end += length
            msg['commID'] = data[start:end]
        except StructError as e:
            raise SerializationError('Could not deserialize message of type StopContainer: {0}'.format(e))
        
        return msg

class ContainerStatusMessage(object):
    """ Message type to # TODO: What exactly is this message used for?
        
        The fields are:
            commID  CommID which is used to identify the environment
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.CONTAINER_STATUS
    
    def serialize(self, data):
        buf = StringIO()
        
        try:
            buf.write(MsgDef.I_STRUCT.pack(len(data['commID'])))
            buf.write(data['commID'])
        except KeyError as e:
            raise SerializationError('Could not serialize message of type ContainerStatus: {0}'.format(e))
        
        return buf.getvalue()
    
    def deserialize(self, data):
        msg = {}
        
        try:
            start = 0
            end = MsgDef.I_LEN
            length, = MsgDef.I_STRUCT.unpack(data[start:end])
            start = end
            end += length
            msg['commID'] = data[start:end]
        except StructError as e:
            raise SerializationError('Could not deserialize message of type ContainerStatus: {0}'.format(e))
        
        return msg
