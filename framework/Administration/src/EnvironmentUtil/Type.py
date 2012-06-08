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
from zope.interface.verify import verifyClass
from zope.interface.exceptions import Invalid

from twisted.python import log

# Custom imports
from Exceptions import InternalError, SerializationError
from Comm.Message.Interfaces import IContentSerializer
from Comm.Message import MsgTypes
from ROSComponents.Interfaces import ISerializable

class ROSAddMessage(object):
    """ Message type to add a node.
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.ROS_ADD
    
    def __init__(self):
        self._componentCls = {}
    
    def registerComponents(self, componentList):
        """ Register a component class which should be used to (de-)serialize the
            component which should be added.
            
            @raise:     InternalError if the component class does not implement the
                        "ISerializable" interface.
        """
        for component in componentList:
            try:
                verifyClass(ISerializable, component)
            except Invalid as e:
                raise InternalError(
                    'Verification of the class "{0}" for the Interface "ISerializable" failed: {1}'.format(
                        component.__name__,
                        e
                    )
                )
            
            self._componentCls[component.IDENTIFIER] = component
    
    # TODO: Not really necessary; just for completeness
    def unregisterComponent(self, component):
        """ Unregister a component class.
            
            @param component:   # TODO:
            @type  component:   # TODO:
        """
        del self._componentCls[" TODO: Depends on type of component"]
    
    def serialize(self, s, data):
        try:
            Cls = self._componentCls[data.IDENTIFIER]
        except AttributeError:
            raise SerializationError('''The object's class does not implement the interface "ISerializable".''')
        except KeyError:
            raise SerializationError('''The object's class is not registered.''')
        
        if not isinstance(data, Cls):
            raise SerializationError('The object is of invalid type.')
        
        s.addIdentifier(data.IDENTIFIER, 1)
        data.serialize(s)
    
    def deserialize(self, s):
        try:
            Cls = self._componentCls[s.getIdentifier(1)]
        except KeyError:
            raise SerializationError('''The object's class is not registered.''')
        
        return Cls.deserialize(s)

class ROSRemoveMessage(object):
    """ Message type to remove a node.
        
        The fields are:
            tag     Tag/Name which should be used to identify the object which should
                    be removed
            type    Name to identify the type of object
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.ROS_REMOVE
    
    def serialize(self, s, msg):
        s.addElement(msg['tag'])
        s.addElement(msg['type'])
    
    def deserialize(self, s):
        return { 'tag' : s.getElement(), 'type' : s.getElement() }

class ROSAddUserMessage(object):
    """ Message type to register a new user with an interface.
        
        The fields are:
            tag     Tag which should be used to identify the interface to which the
                    user should be added
            commID  Communication ID of the node to which the messages should be sent
            target  Tag of the receiving interface/user
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.ROS_ADD_USER
    
    def serialize(self, s, msg):
        s.addElement(msg['tag'])
        s.addElement(msg['commID'])
        s.addElement(msg['target'])
    
    def deserialize(self, s):
        return { 'tag'    : s.getElement,
                 'commID' : s.getElement,
                 'target' : s.getElement }

class ROSRemoveUserMessage(object):
    """ Message type to unregister an user from an interface.
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.ROS_ADD_USER
    
    def serialize(self, s, msg):
        s.addElement(msg[''])
    
    def deserialize(self, s):
        return { '' : s.getElement }

class ROSMsgMessage(object):
    """ Message type for a single ROS message.
        
        The fields are:
            msg     Serialized ROS Message (Serialize: MessageFIFO; Deserialize: str)
            tag     Interface Tag which should be used to send the message or from which
                    the message was received
            user    Identifier for the robot/container who sent the message (only necessary
                    if the message is for a service)
            push    Flag to indicate whether the response message, if it exists,
                    should be pushed back or stored for polling
            uid     Unique ID to identify the message
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.ROS_MSG
    
    def serialize(self, s, data):
        if not isinstance(data, dict):
            raise SerializationError('Content of the message ROSMessage has to be a dictionary.')
        
        try:
            s.addInt(len(data['msg']))
            s.addFIFO(data['msg'])
            s.addElement(data['tag'])
            s.addElement(data['user'])
            s.addElement(data['uid'])
            s.addBool(data['push'])
        except KeyError as e:
            raise SerializationError('Could not serialize message of type ROSMessage: {0}'.format(e))
    
    def deserialize(self, s):
        return { 'msg'  : s.getElement(),
                 'tag'  : s.getElement(),
                 'user' : s.getElement(),
                 'uid'  : s.getElement(),
                 'push' : s.getBool() }

### TODO: Not used for pure push implementation
class ROSResponseMessage(object):
    """ Message type to send back an ID which can be used to retrieve a result later.
        
        The fields are:
            msg     String which should be sent; if no error occurred it should contain
                    the ID and if an error occurred it should contain the error message
            error   Flag to indicate whether msg contains an error message or not
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.ROS_RESPONSE
    
    def serialize(self, s, data):
        if not isinstance(data, dict):
            raise SerializationError('Content of the message ROSResponse has to be a dictionary.')
        
        try:
            s.addElement(data['msg'])
            s.addBool(data['error'])
        except KeyError as e:
            raise SerializationError('Could not serialize message of type ROSResponse: {0}'.format(e))
    
    def deserialize(self, s):
        return { 'msg'   : s.getElement(),
                 'error' : s.getBool() }

### TODO: Not used for pure push implementation
class ROSGetMessage(object):
    """ Message type to request a result using an ID.
        
        The fields are:
            name    Interface name from where the message should be retrieved
            uid     ID which identifies the message which should be retrieved
    """
    implements(IContentSerializer)
    
    IDENTIFIER = MsgTypes.ROS_GET
    
    def serialize(self,s,  data):
        if not isinstance(data, dict):
            raise SerializationError('Content of the message ROSResponse has to be a dictionary.')
        
        try:
            s.addElement(data['name'])
            s.addElement(data['uid'])
        except KeyError as e:
            raise SerializationError('Could not serialize message of type ROSGet: {0}'.format(e))
    
    def deserialize(self, s):
        return { 'name' : s.getElement(),
                 'uid'  : s.getElement() }
