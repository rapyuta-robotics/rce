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

# ROS specific imports
import genpy

# Custom imports
from Exceptions import InvalidRequest
from Comm.Message import MsgTypes
from Comm.Message.Base import Message
from Comm.Message.FIFO import MessageFIFO

class Interface(object):
    """ Class which represents an interface. It is associated with a container.
    """
    def __init__(self, container, tag, rosAddr, msgType, interfaceType):
        """ Initialize the Interface.
            
             # TODO: Add description
        """
        self._container = container
        self._tag = tag
        self._rosAddr = rosAddr
        self._msgType = msgType
        self._interfaceType = interfaceType
        
        self._MsgCls = genpy.message.get_message_class(msgType)
        
        # TODO: Check if necessary...
        #if not self._MsgCls:
        #    self._MsgCls = genpy.message.get_service_class(msgType)

        if not self._MsgCls:
            raise ValueError('Message class for the interface could not be loaded.')
        
        self._ref = []
    
    def validate(self, tag, rosAddr, msgType, interfaceType):
        """ # TODO: Add description
        """
        return ( tag == self._tag and
                 rosAddr == self._rosAddr and
                 msgType == self._msgType and
                 interfaceType == self._interfaceType )
    
    def _stop(self):
        """ Internal method to send a request to stop the interface.
        """
        msg = Message()
        msg.msgType = MsgTypes.ROS_REMOVE
        msg.content = {} # TODO: Add correct content
        self._container.send(msg)
    
    def registerUser(self, uid):
        """ # TODO: Add description
        """
        if not self._ref:
            # There are no references to this interface so far;
            # therefore, add it
            msg = Message()
            msg.msgType = MsgTypes.ROS_ADD
            msg.content = {} # TODO: Add correct content
            self._container.send(msg)
        
        msg = Message()
        msg.msgType = MsgTypes.ROS_ADD_USER
        msg.content = {} # TODO: Add correct content
        self._container.send(msg)
        
        self._ref.append(uid)
    
    def unregisterUser(self, uid):
        """ # TODO: Add description
        """
        self._reg.remove(uid)
        
        if not self._ref:
            # There are no more references to this interface;
            # therefore, remove it
            self._stop()
        else:
            msg = Message()
            msg.msgType = MsgTypes.ROS_REMOVE_USER
            msg.content = {} # TODO: Add correct content
            self._container.send(msg)
    
    def send(self, msg, sender):
        """ # TODO: Add description
            
            @param msg:     Corresponds to the dictionary of the field 'data' of the received
                            message. (Necessary keys: type, msgID, msg)
            @type  msg:     { str : ... }
        """
        if msg['type'] != self._msgType:
            raise InvalidRequest('Sent message type does not match the used message for this interface.')
        
        try:
            rosMsg = self._converter.decode(self._MsgCls, msg['msg'])
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
                        'id'   : msg['msgID'] }
        
        self._container.send(msg)
    
    def receive(self, msg):
        """ # TODO: Add description
            
            @param msg:     Content of received ROS message.
            @type  msg:     { str : ... }
        """
        rosMsg = self._MsgCls()
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
