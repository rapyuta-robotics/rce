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
    def __init__(self, commMngr, name, msgType, interfaceType):
        """ Initialize the Interface.
            
             # TODO: Add description
        """
        self._commManager = commMngr
        self._name = name
        self._msgType = msgType
        self._interfaceType = interfaceType
        
        self._MsgCls = genpy.message.get_message_class(msgType)
        
        # TODO: Check if necessary...
        #if not self._MsgCls:
        #    self._MsgCls = genpy.message.get_service_class(msgType)

        if not self._MsgCls:
            raise ValueError('Message class for the interface could not be loaded.')
        
        self._ref = []
    
    def validate(self, name, msgType, interfaceType):
        """ # TODO: Add description
        """
        return name == self._name and msgType == self._msgType and interfaceType == self._interfaceType
    
    def registerUser(self, uid):
        """ # TODO: Add description
        """
        if not self._ref:
            # There are no references to this interface so far;
            # therefore, add it
            msg = Message()
            msg.msgType = MsgTypes.ROS_ADD
            msg.dest = self._commID
            msg.content = {} # TODO: Add correct content
            self._commManager.sendMessage(msg)
        
        self._ref.append(uid)
    
    def unregisterUser(self, uid):
        """ # TODO: Add description
        """
        self._reg.remove(uid)
        
        if not self._ref:
            # There are no more references to this interface;
            # therefore, remove it
            msg = Message()
            msg.msgType = MsgTypes.ROS_REMOVE
            msg.dest = self._commID
            msg.content = {} # TODO: Add correct content
            self._commManager.sendMessage(msg)
    
    def send(self, msg, containerID, sender):
        """ # TODO: Add description
        """
        try:
            rosMsg = self._converter.decode(self._MsgCls, msg)
        except (TypeError, ValueError) as e:
            raise InvalidRequest(str(e))
        
        fifo = MessageFIFO()
        rosMsg.serialize(fifo)
        
        msg = Message()
        msg.msgType = MsgTypes.ROS_MSG
        msg.dest = containerID
        msg.content = { 'msg'  : fifo,
                        'name' : self._name,
                        'uid'  : sender,
                        'push' : True }
        
        self._commMngr.sendMessage(msg)
    
    def receive(self, msg, receiver):
        """ # TODO: Add description
        """
        rosMsg = self._MsgCls()
        rosMsg.deserialize(msg)
        
        try:
            msg = self._converter.encode(rosMsg)
        except (TypeError, ValueError) as e:
            raise InvalidRequest(str(e))
        
        # TODO: Send to robot using receiver
