#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       Interface.py
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

# Custom imports
from Exceptions import InvalidRequest
from Comm.Message import MsgTypes
from Comm.Message.Base import Message
from Comm.Message.FIFO import MessageFIFO

from ROSComponents import ComponentDefinition
from ROSComponents.Interface import ServiceInterface, PublisherInterface, SubscriberInterface

class Interface(object):
    """ Class which represents an interface. It is associated with a container.
    """
    _MAP = { 'service'    : ServiceInterface,
             'publisher'  : PublisherInterface,
             'subscriber' : SubscriberInterface }
    
    def __init__(self, satelliteMngr, container, tag, rosAddr, msgType, interfaceType):
        """ Initialize the Interface.
            
             # TODO: Add description
        """
        self._container = container
        self._tag = tag
        self._rosAddr = rosAddr
        self._msgType = msgType
        self._interfaceType = interfaceType
        self._converter = satelliteMngr.converter
        
        try:
            container.reserveAddr(rosAddr)
        except ValueError:
            raise InvalidRequest('Another interface with the same ROS address already exists.')
        
        args = msgType.split('/')
        
        if len(args) != 2:
            raise InvalidRequest('msg/srv type is not valid. Has to be of the from pkg/msg, i.e. std_msgs/Int8.')
        
        if interfaceType == 'service':
            srvCls = satelliteMngr.loader.loadSrv(*args)
            self._toMsgCls = srvCls._request_class
            self._fromMsgCls = srvCls._response_class
        elif interfaceType == 'publisher':
            self._toMsgCls = satelliteMngr.loader.loadMsg(*args)
            self._fromMsgCls = None
        elif interfaceType == 'subscriber':
            self._toMsgCls = None
            self._fromMsgCls = satelliteMngr.loader.loadMsg(*args)
        else:
            raise ValueError('"{0}" is not a valid interface type.'.format(interfaceType))
        
        self._ref = []
    
    def validate(self, tag, rosAddr, msgType, interfaceType):
        """ # TODO: Add description
        """
        return ( tag == self._tag and
                 rosAddr == self._rosAddr and
                 msgType == self._msgType and
                 interfaceType == self._interfaceType )
    
    def _start(self):
        """ Internal method to send a request to start the interface.
        """
        msg = Message()
        msg.msgType = MsgTypes.ROS_ADD
        msg.content = Interface._MAP[self._interfaceType](self._rosAddr, self._tag, self._msgType)
        self._container.send(msg)
    
    def _stop(self):
        """ Internal method to send a request to stop the interface.
        """
        msg = Message()
        msg.msgType = MsgTypes.ROS_REMOVE
        msg.content = { 'type' : ComponentDefinition.RM_INTERFACE,
                        'tag'  : self._tag }
        self._container.send(msg)
    
    def registerUser(self, target, commID):
        """ # TODO: Add description
        """
        if not self._ref:
            # There are no references to this interface so far; therefore, add it
            self._start()
        
        msg = Message()
        msg.msgType = MsgTypes.ROS_USER
        msg.content = { 'tag'    : self._tag,
                        'target' : target,
                        'commID' : commID,
                        'add'    : True }
        self._container.send(msg)
        
        self._ref.append((target, commID))
    
    def unregisterUser(self, target, commID):
        """ # TODO: Add description
        """
        self._ref.remove((target, commID))
        
        if not self._ref:
            # There are no more references to this interface; therefore, remove it
            self._stop()
        else:
            msg = Message()
            msg.msgType = MsgTypes.ROS_USER
            msg.content = { 'tag'    : self._tag,
                            'target' : target,
                            'commID' : commID,
                            'add'    : False }
            self._container.send(msg)
    
    def send(self, clientMsg, sender):
        """ # TODO: Add description
            
            @param clientMsg:   Corresponds to the dictionary of the field 'data' of the received
                                message. (Necessary keys: type, msgID, msg)
            @type  clientMsg:   { str : ... }
            
            @param sender:  Identifier of the sender.
            @type  sender:  str
        """
        if clientMsg['type'] != self._msgType:
            raise InvalidRequest('Sent message type does not match the used message for this interface.')
        
        try:
            rosMsg = self._converter.decode(self._toMsgCls, clientMsg['msg'])
        except (TypeError, ValueError) as e:
            raise InvalidRequest(str(e))
        
        fifo = MessageFIFO()
        rosMsg.serialize(fifo)
        
        msg = Message()
        msg.msgType = MsgTypes.ROS_MSG
        msg.content = { 'msg'  : fifo,
                        'tag'  : self._tag,
                        'user' : sender,
                        'push' : True,
                        'uid'  : clientMsg['msgID'] }
        self._container.send(msg)
    
    def receive(self, msg):
        """ # TODO: Add description
            
            @param msg:     Content of received ROS message.
            @type  msg:     { str : ... }
        """
        rosMsg = self._fromMsgCls()
        rosMsg.deserialize(msg['msg'])
        
        try:
            jsonMsg = self._converter.encode(rosMsg)
        except (TypeError, ValueError) as e:
            raise InvalidRequest(str(e))
        
        self._container.receivedFromInterface({ 'type'         : self._msgType, 
                                                'msgID'        : msg['id'],
                                                'interfaceTag' : msg['tag'],
                                                'msg'          : jsonMsg })
    
    def __del__(self):
        """ Destructor.
        """
        if self._ref:
            self._stop()
        
        self._container.freeAddr(self._rosAddr)
