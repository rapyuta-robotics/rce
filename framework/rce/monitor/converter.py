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
#     Copyright 2013 RoboEarth
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
import zlib

try:
    from cStringIO import StringIO, InputType, OutputType
    from StringIO import StringIO as pyStringIO
    
    def _checkIsStringIO(obj):
        return isinstance(obj, (InputType, OutputType, pyStringIO))
except ImportError:
    from StringIO import StringIO
    
    def _checkIsStringIO(obj):
        return isinstance(obj, StringIO)

# Custom imports
from rce.error import InvalidRequest, InternalError
from rce.slave.interface import Interface
import settings


# Compression level used
GZIP_LVL = settings.GZIP_LVL


class _AbstractConverter(Interface):
    """ Base class which is used to handle and monitor a converter.
    """
    def __init__(self, owner, uid, msgType, tag):
        """ Initialize the Converter monitor.
        """
        self._owner = owner
        self._msgType = msgType
        self._tag = tag
        
        Interface.__init__(self, owner, uid)
    
    @property
    def tag(self):
        """ Interface tag of the converter. """
        return self._tag


class _ConverterBase(_AbstractConverter):
    """ Base class which is used to handle and monitor a converter.
    """
    def __init__(self, owner, uid, msgType, tag):
        _AbstractConverter.__init__(self, owner, uid, msgType)
        
        self._converter = owner.converter
        
        self._inputMsgCls = None
        self._outputMsgCls = None
        
        self._loadClass(owner.loader)
    
    __init__.__doc__ = _AbstractConverter.__init__.__doc__
    
    def _loadClass(self, loader):
        """ This method is used to load the necessary ROS message class
            definition. And is called as last step in the constructor.
            
            This method has to be overwritten!
            
            References to the loaded classes have to be stored in the instance
            attributes '_inputMsgCls' and '_outputMsgCls'.
            If there is no reference stored to an attribute it is assumed that
            the converter can not convert corresponding messages and and error
            is raised.
        """
        raise NotImplementedError("The method 'loadClass' has to "
                                  'be implemented.')
    
    def receive(self, msgType, msgID, msg):
        """ Convert a JSON encoded message into a ROS message.
            
            @param msgType:     Message type of the received message.
            @type  msgType:     str
            
            @param msgID:       Identifier which is used to match request /
                                response message.
            @type  msgID:       str
            
            @param msg:         JSON compatible message which should be
                                processed.
            @type  msg:         dict
            
            @raise:         errors.InternalError if the Converter can not
                            convert incoming messages or
                            erors.InvalidRequest if the message can not be
                            converted the message due to an invalid format
        """
        if not self._inputMsgCls:
            raise InternalError('This converter can not handle incoming'
                                ' messages.')
        
        if msgType != self._msgType:
            raise InvalidRequest('Sent message type does not match the used '
                                 'message type for this interface.')
        
        try:
            msg = self._converter.decode(self._inputMsgCls, msg)
        except (TypeError, ValueError) as e:
            raise InvalidRequest(str(e))
        
        buf = StringIO()
        msg.serialize(buf)
        msg = buf.getvalue()
        
        self.received(msg, msgID)
    
    def _send(self, msg, msgID, protocol, remoteID):
        """ Convert a ROS message into a JSON encoded message.
            
            @param msg:     Received ROS message in serialized form.
            @type  msg:     str
            
            @param msgID:   Unique ID to identify the message.
            @type  msgID:   str
            
            @raise:         errors.InternalError if the Converter can not
                            convert outgoing messages or
                            erors.InvalidRequest if the message can not be
                            converted the message due to an invalid format
        """
        if not self._outputMsgCls:
            raise InternalError('This converter can not handle outgoing '
                                'messages.')
        
        rosMsg = self._outputMsgCls()
        rosMsg.deserialize(msg)
        
        try:
            jsonMsg = self._converter.encode(rosMsg)
        except (TypeError, ValueError) as e:
            raise InvalidRequest(str(e))
        
        self._owner.sendToClient(self._tag, self._msgType, msgID, jsonMsg)


class ServiceClientConverter(_ConverterBase):
    """ Class which is used to handle and monitor a Service Converter.
    """
    def _loadClass(self, loader):
        args = self._msgType.split('/')
        
        if len(args) != 2:
            raise InvalidRequest('srv type is not valid. Has to be of the '
                                 'from pkg/msg, i.e. std_msgs/Int8.')
        
        srvCls = loader.loadSrv(*args)
        self._inputMsgCls = srvCls._response_class
        self._outputMsgCls = srvCls._request_class


class ServiceProviderConverter(_ConverterBase):
    """ Class which is used to handle and monitor a Service-Provider Converter.
    """
    def _loadClass(self, loader):
        args = self._msgType.split('/')
        
        if len(args) != 2:
            raise InvalidRequest('srv type is not valid. Has to be of the '
                                 'from pkg/msg, i.e. std_msgs/Int8.')
        
        srvCls = loader.loadSrv(*args)
        self._inputMsgCls = srvCls._request_class
        self._outputMsgCls = srvCls._response_class


class PublisherConverter(_ConverterBase):
    """ Class which is used to handle and monitor a Publisher Converter.
    """
    def _loadClass(self, loader):
        args = self._msgType.split('/')
        
        if len(args) != 2:
            raise InvalidRequest('msg type is not valid. Has to be of the '
                                 'from pkg/msg, i.e. std_msgs/Int8.')
        
        self._outputMsgCls = loader.loadMsg(*args)


class SubscriberConverter(_ConverterBase):
    """ Class which is used to handle and monitor a Subscriber Converter.
    """
    def _loadClass(self, loader):
        args = self._msgType.split('/')
        
        if len(args) != 2:
            raise InvalidRequest('msg type is not valid. Has to be of the '
                                 'from pkg/msg, i.e. std_msgs/Int8.')
        
        self._inputMsgCls = loader.loadMsg(*args)


class Forwarder(_AbstractConverter):
    """ Class which is used to handle and monitor a forwarder.
    """
    def receive(self, msgType, msgID, msg):
        """ Convert a JSON encoded message into a ROS message.
            
            @param msgType:     Message type of the received message.
            @type  msgType:     str
            
            @param msgID:       Identifier which is used to match request /
                                response message.
            @type  msgID:       str
            
            @param msg:         JSON compatible message which should be
                                processed.
            @type  msg:         dict
            
            @raise:         erors.InvalidRequest if the message can not be
                            forwarded the message due to an invalid format
        """
        if msgType != self._msgType:
            raise InvalidRequest('Sent message type does not match the used '
                                 'message type for this interface.')
        
        if not _checkIsStringIO(msg):
            raise InvalidRequest('Sent message is not a binary message.')
        
        self.received(zlib.decompress(msg.getvalue()), msgID)
    
    def _send(self, msg, msgID, protocol, remoteID):
        """ Convert a ROS message into a JSON encoded message.
            
            @param msg:     Received ROS message in serialized form.
            @type  msg:     str
            
            @param msgID:   Unique ID to identify the message.
            @type  msgID:   str
        """
        self._owner.sendToClient(self._tag, self._msgType, msgID,
                                 StringIO(zlib.compress(msg, GZIP_LVL)))
