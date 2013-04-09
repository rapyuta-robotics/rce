#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     rce-core/rce/monitor/interface/robot.py
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
from uuid import uuid4

try:
    from cStringIO import StringIO, InputType, OutputType
    from StringIO import StringIO as pyStringIO
    
    def _checkIsStringIO(obj):
        return isinstance(obj, (InputType, OutputType, pyStringIO))
except ImportError:
    from StringIO import StringIO
    
    def _checkIsStringIO(obj):
        return isinstance(obj, StringIO)

# rce specific imports
from rce.util.error import InternalError
from rce.slave.interface import Interface, InvalidResoureName
from rce.util.settings import RCESettingsManager
settings = RCESettingsManager()

# Compression level used
GZIP_LVL = settings.GZIP_LVL


class ConversionError(Exception):
    """ Exception is raised in case a ROS message could not be converted.
    """


class ServiceError(Exception):
    """ Exception is raised in case a response message was received, but not
        corresponding request is available.
    """


class _AbstractConverter(Interface):
    """ Abstract base class which provides the basics for the robot-side
        interfaces.
    """
    def __init__(self, owner, status, uid, clsName, tag):
        """ Initialize the robot-side Interface.
            
            @param owner:       Namespace to which this interface belongs.
            @type  owner:       rce.robot.Robot
            
            @param status:      Status observer which is used to inform the
                                Master of the interface's status.
            @type  status:      twisted.spread.pb.RemoteReference
            
            @param uid:         Unique ID which is used to identify the
                                interface in the internal communication.
            @type  uid:         uuid.UUID
            
            @param clsName:     Message type/Service type consisting of the
                                package and the name of the message/service,
                                i.e. 'std_msgs/Int32'.
            @type  clsName:     str
            
            @param tag:         Unique ID which is used to identify the
                                interface in the external communication.
            @type  tag:         str
        """
        self._owner = owner
        self._clsName = clsName
        self._tag = tag
        
        Interface.__init__(self, owner, status, uid)
    
    @property
    def tag(self):
        """ Interface tag of the converter. """
        return self._tag
    
    def _receive(self, msg, msgID):
        """ This method is used as a hook to send the message received from the
            robot to the appropriate protocol.
        """
        raise NotImplementedError("The method '_receive' has to "
                                  'be implemented.')
    
    def _sendToClient(self, msg, msgID, protocol, remoteID):
        """ This method is used as a hook to send the message received from the
            protocol to the robot.
        """
        raise NotImplementedError("The method '_sendToClient' has to "
                                  'be implemented.')


class _ConverterBase(_AbstractConverter):
    """ Class which implements the basic functionality of a Converter.
    """
    def __init__(self, owner, status, uid, clsName, tag):
        _AbstractConverter.__init__(self, owner, status, uid, clsName, tag)
        
        self._converter = owner.converter
        
        self._inputMsgCls = None
        self._outputMsgCls = None
        
        self._loadClass(owner.loader)
    
    __init__.__doc__ = _AbstractConverter.__init__.__doc__
    
    def _loadClass(self, loader):
        """ This method is used as a hook to load the necessary ROS class
            for the interface. And is called as last step in the constructor.
            
            This method has to be overwritten!
            
            References to the loaded classes have to be stored in the instance
            attributes '_inputMsgCls' and '_outputMsgCls'.
            If there is no reference stored, i.e. None, it is assumed that
            the converter can not convert the messages in the given direction
            and an error is raised.
        """
        raise NotImplementedError("The method 'loadClass' has to "
                                  'be implemented.')
    
    def receive(self, clsName, msgID, msg):
        """ Convert a JSON encoded message into a ROS message.
            
            @param clsName:     Message/Service type of the received message.
            @type  clsName:     str
            
            @param msgID:       Identifier which is used to match request /
                                response message.
            @type  msgID:       str
            
            @param msg:         JSON compatible message which should be
                                processed.
            @type  msg:         dict
        """
        if not self._inputMsgCls:
            raise InternalError('This converter can not handle incoming'
                                ' messages.')
        
        if clsName != self._clsName:
            raise InvalidResoureName('Sent message type does not match the '
                                     'used message type for this interface.')
        
        try:
            msg = self._converter.decode(self._inputMsgCls, msg)
        except (TypeError, ValueError) as e:
            raise ConversionError(str(e))
        
        buf = StringIO()
        msg.serialize(buf)
        msg = buf.getvalue()
        
        self._receive(msg, msgID)
    
    def _send(self, msg, msgID, protocol, remoteID):
        """ Convert a ROS message into a JSON encoded message.
            
            @param msg:         Received ROS message in serialized form.
            @type  msg:         str
            
            @param msgID:       Unique ID to identify the message.
            @type  msgID:       str
            
            @param protocol:    Protocol instance through which the message
                                was sent.
            @type  protocol:    rce.slave.protocol._Protocol
            
            @param remoteID:    Unique ID of the Interface which sent the
                                message.
            @type  remoteID:    uuid.UUID
        """
        if not self._outputMsgCls:
            raise InternalError('This converter can not handle outgoing '
                                'messages.')
        
        rosMsg = self._outputMsgCls()
        rosMsg.deserialize(msg)
        
        try:
            jsonMsg = self._converter.encode(rosMsg)
        except (TypeError, ValueError) as e:
            raise ConversionError(str(e))
        
        self._sendToClient(jsonMsg, msgID, protocol, remoteID)


class ServiceClientConverter(_ConverterBase):
    """ Class which is used as a Service-Client Converter.
    """
    def __init__(self, owner, status, uid, clsName, tag):
        _ConverterBase.__init__(self, owner, status, uid, clsName, tag)
        
        self._pendingRequests = {}
    
    __init__.__doc__ = _ConverterBase.__init__.__doc__
    
    def _loadClass(self, loader):
        args = self._clsName.split('/')
        
        if len(args) != 2:
            raise InvalidResoureName('srv type is not valid. Has to be of the '
                                     'from pkg/msg, i.e. std_msgs/Int8.')
        
        srvCls = loader.loadSrv(*args)
        self._inputMsgCls = srvCls._response_class
        self._outputMsgCls = srvCls._request_class
    
    def _receive(self, msg, msgID):
        try:
            msgID, protocol, remoteID = self._pendingRequests.pop(msgID)
        except KeyError:
            raise ServiceError('Service Client does not wait for a response '
                               'with message ID {0}.'.format(msgID))
        
        self.respond(msg, msgID, protocol, remoteID)
    
    def _sendToClient(self, msg, msgID, protocol, remoteID):
        while 1:
            uid = uuid4().hex
            
            if uid not in self._pendingRequests:
                break
        
        self._pendingRequests[uid] = (msgID, protocol, remoteID)
        self._owner.sendToClient(self._tag, self._clsName, msgID, msg)


class ServiceProviderConverter(_ConverterBase):
    """ Class which is used as a Service-Provider Converter.
    """
    def remote_connect(self, protocol, remoteID):
        if self._protocols:
            raise InternalError('Can not register more than one interface '
                                'at a time with a Service-Provider.')
        
        return _ConverterBase.remote_connect(self, protocol, remoteID)
    
    remote_connect.__doc__ = _ConverterBase.remote_connect.__doc__
    
    def _loadClass(self, loader):
        args = self._clsName.split('/')
        
        if len(args) != 2:
            raise InvalidResoureName('srv type is not valid. Has to be of the '
                                     'from pkg/msg, i.e. std_msgs/Int8.')
        
        srvCls = loader.loadSrv(*args)
        self._inputMsgCls = srvCls._request_class
        self._outputMsgCls = srvCls._response_class
    
    def _receive(self, msg, msgID):
        self.received(msg, msgID)
    
    def _sendToClient(self, msg, msgID, protocol, remoteID):
        self._owner.sendToClient(self._tag, self._clsName, msgID, msg)


class PublisherConverter(_ConverterBase):
    """ Class which is used as a Publisher Converter.
    """
    def _loadClass(self, loader):
        args = self._clsName.split('/')
        
        if len(args) != 2:
            raise InvalidResoureName('msg type is not valid. Has to be of the '
                                     'from pkg/msg, i.e. std_msgs/Int8.')
        
        self._outputMsgCls = loader.loadMsg(*args)
    
    def _receive(self, msg, msgID):
        self.received(msg, msgID)
    
    def _sendToClient(self, msg, msgID, protocol, remoteID):
        self._owner.sendToClient(self._tag, self._clsName, msgID, msg)


class SubscriberConverter(_ConverterBase):
    """ Class which is used as a Subscriber Converter.
    """
    def _loadClass(self, loader):
        args = self._clsName.split('/')
        
        if len(args) != 2:
            raise InvalidResoureName('msg type is not valid. Has to be of the '
                                     'from pkg/msg, i.e. std_msgs/Int8.')
        
        self._inputMsgCls = loader.loadMsg(*args)
    
    def _receive(self, msg, msgID):
        self.received(msg, msgID)
    
    def _sendToClient(self, msg, msgID, protocol, remoteID):
        self._owner.sendToClient(self._tag, self._clsName, msgID, msg)


class _ForwarderBase(_AbstractConverter):
    """ Class which implements the basic functionality of a Forwarder.
    """
    def receive(self, clsName, msgID, msg):
        """ Unwrap and inflate a JSON encoded ROS message.
            
            @param clsName:     Message/Service type of the received message.
            @type  clsName:     str
            
            @param msgID:       Identifier which is used to match request /
                                response message.
            @type  msgID:       str
            
            @param msg:         JSON compatible message which should be
                                processed.
            @type  msg:         dict
        """
        if clsName != self._clsName:
            raise InvalidResoureName('Sent message type does not match the '
                                     'used message type for this interface.')
        
        if not _checkIsStringIO(msg):
            raise ConversionError('Sent message is not a binary message.')
        
        self._receive(zlib.decompress(msg.getvalue()), msgID)
    
    def _send(self, msg, msgID, protocol, remoteID):
        """ Wrap and deflate a ROS message in a JSON encoded message.
            
            @param msg:         Received ROS message in serialized form.
            @type  msg:         str
            
            @param msgID:       Unique ID to identify the message.
            @type  msgID:       str
            
            @param protocol:    Protocol instance through which the message
                                was sent.
            @type  protocol:    rce.slave.protocol._Protocol
            
            @param remoteID:    Unique ID of the Interface which sent the
                                message.
            @type  remoteID:    uuid.UUID
        """
        self._sendToClient(StringIO(zlib.compress(msg, GZIP_LVL)), msgID,
                           protocol, remoteID)


class ServiceClientForwarder(_ForwarderBase):
    """ Class which is used as a Service-Client Forwarder.
    """
    def __init__(self, owner, status, uid, clsName, tag):
        _ForwarderBase.__init__(self, owner, status, uid, clsName, tag)
        
        self._pendingRequests = {}
    
    __init__.__doc__ = _ForwarderBase.__init__.__doc__
    
    def _receive(self, msg, msgID):
        try:
            msgID, protocol, remoteID = self._pendingRequests.pop(msgID)
        except KeyError:
            raise ServiceError('Service Client does not wait for a response '
                               'with message ID {0}.'.format(msgID))
        
        self.respond(msg, msgID, protocol, remoteID)
    
    def _sendToClient(self, msg, msgID, protocol, remoteID):
        while 1:
            uid = uuid4().hex
            
            if uid not in self._pendingRequests:
                break
        
        self._pendingRequests[uid] = (msgID, protocol, remoteID)
        self._owner.sendToClient(self._tag, self._clsName, msgID, msg)


class ServiceProviderForwarder(_ForwarderBase):
    """ Class which is used as a Service-Provider Forwarder.
    """
    def remote_connect(self, protocol, remoteID):
        if self._protocols:
            raise InternalError('Can not register more than one interface '
                                'at a time with a Service-Provider.')
        
        return _ForwarderBase.remote_connect(self, protocol, remoteID)
    
    remote_connect.__doc__ = _ForwarderBase.remote_connect.__doc__
    
    def _receive(self, msg, msgID):
        self.received(msg, msgID)
    
    def _sendToClient(self, msg, msgID, protocol, remoteID):
        self._owner.sendToClient(self._tag, self._clsName, msgID, msg)


class PublisherForwarder(_ForwarderBase):
    """ Class which is used as a Publisher Forwarder.
    """
    def _receive(self, msg, msgID):
        self.received(msg, msgID)
    
    def _sendToClient(self, msg, msgID, protocol, remoteID):
        self._owner.sendToClient(self._tag, self._clsName, msgID, msg)


class SubscriberForwarder(_ForwarderBase):
    """ Class which is used as a Subscriber Forwarder.
    """
    def _receive(self, msg, msgID):
        self.received(msg, msgID)
    
    def _sendToClient(self, msg, msgID, protocol, remoteID):
        self._owner.sendToClient(self._tag, self._clsName, msgID, msg)
