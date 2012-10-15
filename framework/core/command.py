#!/usr/bin/env python
# -*- coding: utf-8 -*-
#     
#     command.py
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
#     Copyright 2012 RoboEarth
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

# zope specific imports
from zope.interface import implements

# twisted specific imports
from twisted.python import log

# Custom imports
from errors import InternalError
from core.interfaces import ISerializable
from core.types import cmd as types
from core.interfaces import IEndpointConverterCommand, \
    IEndpointInterfaceCommand, IDistributor


class ControlDistributor(object):
    """ Class which is used to collect "incoming" control messages (either
        Command message or Tag message) and distribute them to the
        appropriate handlers.
    """
    implements(IDistributor)
    
    def __init__(self):
        """ Initialize the control distributor.
        """
        self._handlers = {}
    
    def addHandler(self, identifier, handler):
        """ Add a handler for a content type.
            
            If there is already a handler registered for the same identifier,
            then the old handler is removed and only the new handler is kept.
            
            @param identifier:  Identifier which is used to match the content
                                type to the handler.
            @type  identifier:  str
            
            @param handler:     Callable which is called when matching content
                                type is encountered.
            @type  handler:     Callable
        """
        if not callable(handler):
            raise InternalError('Tried to add a handler which is not '
                                'callable.')
        
        self._handlers[identifier] = handler
    
    def removeHandler(self, identifier):
        """ Remove a handler for a content type.
            
            @param identifier:  Identifier which is used to match the content
                                type to the handler which should be removed.
            @type  identifier:  str
        """
        if identifier not in self._handlers:
            raise InternalError('Tried to remove a handler which does not '
                                'exist.')
        
        del self._handlers[identifier]
    
    def processCommand(self, user, cmd):
        """ Process a command message.
            
            @param user:    UserID of the user who is responsible for the
                            request.
            @type  user:    str
            
            @param cmd:     Command which should be processed.
        """
        try:
            identifier = cmd.IDENTIFIER
        except AttributeError:
            raise InternalError('Received command of command message '
                                'does not have a valid IDENTIFIER.')
        
        if identifier not in self._handlers:
            log.msg('Received command type "{0}", which can not be '
                    'handled.'.format(identifier))
        else:
            self._handlers[identifier](user, cmd)
    
    def processTag(self, user, identifier, tag):
        """ Process a tag message.
            
            @param user:            UserID of the user who is responsible for
                                    the request.
            @type  user:            str
            
            @param identifier:      Identifier which is used to get the correct
                                    handler.
            @type  identifier:      str
            
            @param tag:             Tag which should be processed.
        """
        if identifier not in self._handlers:
            log.msg('Received tag type "{0}", which can not be '
                    'handled.'.format(identifier))
        else:
            self._handlers[identifier](user, tag)


class ContainerCommand(object):
    """ Class which represents a container command.
    """
    implements(ISerializable)
    
    IDENTIFIER = types.CONTAINER
    
    def __init__(self, cTag, commID):
        """ Initialize the container command.
            
            @param cTag:    Tag which is used to identify the container.
            @type  cTag:    str
            
            @param commID:  CommID which will be used for the container.
            @type  commID:  str
        """
        self._tag = cTag
        self._commID = commID
    
    @property
    def tag(self):
        """ Container tag. """
        return self._tag
    
    @property
    def commID(self):
        """ Communication ID of the Container. """
        return self._commID
    
    def serialize(self, s):
        """ Serialize the container command.
            
            @param s:   Serializer instance into which the message should be
                        serialized.
            @type  s:   comm.serializer.Serializer
            
            @raise:     errors.SerializationError
        """
        s.addElement(self._tag)
        s.addElement(self._commID)
    
    @classmethod
    def deserialize(cls, s):
        """ Deserialize the container command.
            
            @param s:   Serializer instance from which the message should be
                        deserialized.
            @type  s:   comm.serializer.Serializer
            
            @raise:     errors.SerializationError
        """
        return cls(s.getElement(), s.getElement())
    

class RobotCommand(object):
    """ Class which represents a robot command.
    """
    implements(ISerializable)
    
    IDENTIFIER = types.ROBOT
    
    def __init__(self, robotID, key):
        """ Initialize the robot command.
            
            @param robotID:     Tag which is used to identify the robot.
            @type  robotID:     str
            
            @param key:         32 byte string which is used to verify the
                                connection from the robot.
            @type  key:         str
        """
        self._robotID = robotID
        self._key = key
    
    @property
    def robotID(self):
        """ Robot ID. """
        return self._robotID
    
    @property
    def key(self):
        """ Key to verify connection. """
        return self._key
    
    def serialize(self, s):
        """ Serialize the robot command.
            
            @param s:   Serializer instance into which the message should be
                        serialized.
            @type  s:   comm.serializer.Serializer
            
            @raise:     errors.SerializationError
        """
        s.addElement(self._robotID)
        s.addElement(self._key)
    
    @classmethod
    def deserialize(cls, s):
        """ Deserialize the robot command.
            
            @param s:   Serializer instance from which the message should be
                        deserialized.
            @type  s:   comm.serializer.Serializer
            
            @raise:     errors.SerializationError
        """
        return cls(s.getElement(), s.getElement())


class NodeCommand(object):
    """ Class which represents a node command.
    """
    implements(ISerializable)
    
    IDENTIFIER = types.NODE
    
    def __init__(self, tag, pkg, exe, args, name, namespace):
        """ Initialize the node command.
            
            @param tag:     Tag which is to identify the node.
            @type  tag:     str

            @param pkg:     Name of the package where this node can be found.
            @type  pkg:     str

            @param exe:     Name of the executable which is used.
            @type  exe:     str
            
            @param args:    List of arguments which should be used with the
                            executable.
            @type  args:    str
            
            @param name:    Name of the node under which it should be launched.
            @type  name:    str
            
            @param namespace:   Namespace in which the node should be launched.
            @type  namespace:   str
        """
        self._tag = tag
        self._pkg = pkg
        self._exe = exe
        self._args = args
        self._name = name
        self._namespace = namespace
    
    @property
    def tag(self):
        """ Tag which is used to identify the node. """
        return self._tag
    
    @property
    def pkg(self):
        """ Package name in which the executable/node is located. """
        return self._pkg
    
    @property
    def exe(self):
        """ Name of the executable/node which should be launched. """
        return self._exe
    
    @property
    def args(self):
        """ Arguments which should be used with the executable. """
        return self._args
    
    @property
    def name(self):
        """ Name of the node in the ROS environment. """
        return self._name
    
    @property
    def namespace(self):
        """ Namespace of the node in the ROS environment. """
        return self._namespace
    
    def serialize(self, s):
        """ Serialize the node command.
            
            @param s:   Serializer instance into which the message should be
                        serialized.
            @type  s:   comm.serializer.Serializer
            
            @raise:     errors.SerializationError
        """
        s.addElement(self._tag)
        s.addElement(self._pkg)
        s.addElement(self._exe)
        s.addElement(self._args)
        s.addElement(self._name)
        s.addElement(self._namespace)
    
    @classmethod
    def deserialize(cls, s):
        """ Deserialize the node command.
            
            @param s:   Serializer instance from which the message should be
                        deserialized.
            @type  s:   comm.serializer.Serializer
            
            @raise:     errors.SerializationError
        """
        return cls(s.getElement(), s.getElement(), s.getElement(), 
                   s.getElement(), s.getElement(), s.getElement())


class NodeForwarderCommand(object):
    """ Dummy class which represents a node. It is used to forward the node
        command without deserializing it to the launcher.
    """
    implements(ISerializable)
    
    IDENTIFIER = types.NODE
    
    def __init__(self, buf):
        """ Initialize the node forwarder command.
            
            @param buf:     FIFO containing the serialized node data.
            @type  buf:     MessageFIFO
        """
        self._buf = buf
    
    def serialize(self, s):
        """ Add the buffered message data to the serializer.
            
            @param s:   Serializer instance into which the message should be
                        serialized.
            @type  s:   comm.serializer.Serializer
            
            @raise:     errors.SerializationError
        """
        s.addFIFO(self._buf)
    
    @classmethod
    def deserialize(cls, s):
        """ Just return the serialized message.
            
            @param s:   Serializer instance from which the message should be
                        deserialized.
            @type  s:   comm.serializer.Serializer
            
            @raise:     errors.SerializationError
        """
        return cls(s.getFIFO())


class _ParameterCommand(object):
    """ Abstract base class which represents a parameter command for a node.
    """
    implements(ISerializable)
    
    def __init__(self, name, value):
        """ Initialize the parameter command.

            @param name:    Name under which the parameter is stored.
            @type  name:    str

            @param value:   Value of the parameter which will be sent to the
                            parameter server.
            @type  value:   Depends on subclass
            
            @raise:         ValueError, if validation of value fails.
        """
        self._name = name
        self._value = self._validate(value)
    
    @property
    def name(self):
        """ ROS name of the parameter. """
        return self._name
    
    @property
    def value(self):
        """ Value of the parameter. """
        return self._value
    
    def _validate(self, value):
        """ Validation of the value.
            
            @return:        Value as correct type.
            
            @raise:         ValueError, if validation of value fails.
        """
        raise ValueError('No validation method implemented.')


class IntCommand(_ParameterCommand):
    """ Class which represents an integer parameter command.
    """
    IDENTIFIER = types.PARAM_INT
    
    def serialize(self, s):
        """ Serialize the integer parameter command.
            
            @param s:   Serializer instance into which the message should be
                        serialized.
            @type  s:   comm.serializer.Serializer
            
            @raise:     errors.SerializationError
        """
        s.addElement(self._name)
        s.addInt(self._value)

    @classmethod
    def deserialize(cls, s):
        """ Deserialize the integer parameter command.
            
            @param s:   Serializer instance from which the message should be
                        deserialized.
            @type  s:   comm.serializer.Serializer
            
            @raise:     errors.SerializationError
        """
        return cls(s.getElement(), s.getInt())
    
    def _validate(self, value):
        return int(value)


class StrCommand(_ParameterCommand):
    """ Class which represents a string parameter command.
    """
    IDENTIFIER = types.PARAM_STR
    
    def serialize(self, s):
        """ Serialize the string parameter command.
            
            @param s:   Serializer instance into which the message should be
                        serialized.
            @type  s:   comm.serializer.Serializer
            
            @raise:     errors.SerializationError
        """
        s.addElement(self._name)
        s.addElement(self._value)

    @classmethod
    def deserialize(cls, s):
        """ Deserialize the string parameter command.
            
            @param s:   Serializer instance from which the message should be
                        deserialized.
            @type  s:   comm.serializer.Serializer
            
            @raise:     errors.SerializationError
        """
        return cls(s.getElement(), s.getElement())
    
    def _validate(self, value):
        if isinstance(value, str):
            return str
        elif isinstance(value, unicode):
            return str(value)
        
        raise ValueError('String is invalid.')


class FloatCommand(_ParameterCommand):
    """ Class which represents a float parameter command.
    """
    IDENTIFIER = types.PARAM_FLOAT
    
    def serialize(self, s):
        """ Serialize the float parameter command.
            
            @param s:   Serializer instance into which the message should be
                        serialized.
            @type  s:   comm.serializer.Serializer
            
            @raise:     errors.SerializationError
        """
        s.addElement(self._name)
        s.addFloat(self._value)

    @classmethod
    def deserialize(cls, s):
        """ Deserialize the float parameter command.
            
            @param s:   Serializer instance from which the message should be
                        deserialized.
            @type  s:   comm.serializer.Serializer
            
            @raise:     errors.SerializationError
        """
        return cls(s.getElement(), s.getFloat())
    
    def _validate(self, value):
        return float(value)


class BoolCommand(_ParameterCommand):
    """ Class which represents a bool parameter command.
    """
    IDENTIFIER = types.PARAM_BOOL
    
    def serialize(self, s):
        """ Serialize the bool parameter command.
            
            @param s:   Serializer instance into which the message should be
                        serialized.
            @type  s:   comm.serializer.Serializer
            
            @raise:     errors.SerializationError
        """
        s.addElement(self._name)
        s.addFloat(self._value)

    @classmethod
    def deserialize(cls, s):
        """ Deserialize the bool parameter command.
            
            @param s:   Serializer instance from which the message should be
                        deserialized.
            @type  s:   comm.serializer.Serializer
            
            @raise:     errors.SerializationError
        """
        return cls(s.getElement(), s.getBool())
    
    def _validate(self, value):
        if isinstance(value, basestring):
            value = value.strip().lower()
            
            if value == 'true':
                return True
            elif value == 'false':
                return False
        else:
            try:
                return bool(int(value))
            except ValueError:
                pass
        
        raise ValueError('Bool is invalid.')


class FileCommand(_ParameterCommand):
    """ Class which represents a file parameter command.
    """
    IDENTIFIER = types.PARAM_FILE
    
    def serialize(self, s):
        """ Serialize the file parameter command.
            
            @param s:   Serializer instance into which the message should be
                        serialized.
            @type  s:   comm.serializer.Serializer
            
            @raise:     errors.SerializationError
        """
        s.addElement(self._name)
        s.addElement(self._value)

    @classmethod
    def deserialize(cls, s):
        """ Deserialize the file parameter command.
            
            @param s:   Serializer instance from which the message should be
                        deserialized.
            @type  s:   comm.serializer.Serializer
            
            @raise:     errors.SerializationError
        """
        return cls(s.getElement(), s.getElement())
    
    def _validate(self, value):
        if isinstance(value, str):
            return str
        elif isinstance(value, unicode):
            return str(value)
        
        raise ValueError('File is invalid.')


class _EndpointInterfaceCommand(object):
    """ Abstract base class which is used for all endpoint interface commands.
        
        This class has to be subclassed and the class attribute 'IDENTIFIER'
        has to be defined.
    """
    implements(ISerializable)
    
    def __init__(self, tag, iClass, endID, name):
        """ Initialize the interface command.

            @param tag:     Tag which is used to identify the interface.
            @type  tag:     str
            
            @param iClass:  Message/Service class which is necessary to use the
                            interface.
            @type  iClass:  str
            
            @param endID:   Identifier of the endpoint to which this interface
                            should be added.
            @type  endID:   str

            @param name:    ROS address of the interface.
            @type  name:    str
        """
        if isinstance(iClass, unicode):
            try:
                iClass = str(iClass)
            except UnicodeEncodeError:
                raise InternalError('The communication class {0} is not '
                                    'valid.'.format(iClass))
        
        if isinstance(name, unicode):
            try:
                name = str(name)
            except UnicodeEncodeError:
                raise InternalError('The interface name {0} is not '
                                    'valid.'.format(name))
        
        self._tag = tag
        self._interfaceClass = iClass
        self._endID = endID
        self._name = name
    
    @property
    def tag(self):
        """ Tag which is used to identify the interface. """
        return self._tag
    
    @property
    def msgType(self):
        """ Message class of the interface. """
        return self._interfaceClass
    
    @property
    def endpointID(self):
        """ Identifier of the endpoint to which this interfaces belongs. """
        return self._endID
    
    @property
    def name(self):
        """ ROS name of the interface. """
        return self._name
    
    def serialize(self, s):
        """ Serialize the interface command.
            
            @param s:   Serializer instance into which the message should be
                        serialized.
            @type  s:   comm.serializer.Serializer
            
            @raise:     errors.SerializationError
        """
        s.addElement(self._tag)
        s.addElement(self._interfaceClass)
        s.addElement(self._endID)
        s.addElement(self._name)
    
    @classmethod
    def deserialize(cls, s):
        """ Deserialize the interface command.
            
            @param s:   Serializer instance from which the message should be
                        deserialized.
            @type  s:   comm.serializer.Serializer
            
            @raise:     errors.SerializationError
        """
        return cls(s.getElement(), s.getElement(),
                   s.getElement(), s.getElement())


class ServiceInterfaceCommand(_EndpointInterfaceCommand):
    """ Class which represents a service interface command.
    """
    implements(IEndpointInterfaceCommand)
    
    IDENTIFIER = types.INTERFACE_SRV


class ServiceProviderInterfaceCommand(_EndpointInterfaceCommand):
    """ Class which represents a service-provider interface command.
    """
    implements(IEndpointInterfaceCommand)
    
    IDENTIFIER = types.INTERFACE_PRO


class PublisherInterfaceCommand(_EndpointInterfaceCommand):
    """ Class which represents a publisher interface command.
    """
    implements(IEndpointInterfaceCommand)
    
    IDENTIFIER = types.INTERFACE_PUB


class SubscriberInterfaceCommand(_EndpointInterfaceCommand):
    """ Class which represents a subscriber interface command.
    """
    implements(IEndpointInterfaceCommand)
    
    IDENTIFIER = types.INTERFACE_SUB


class ServiceConverterCommand(_EndpointInterfaceCommand):
    """ Class which represents a service converter command.
    """
    implements(IEndpointConverterCommand)
    
    IDENTIFIER = types.CONVERTER_SRV


class ServiceProviderConverterCommand(_EndpointInterfaceCommand):
    """ Class which represents a service-provider converter command.
    """
    implements(IEndpointConverterCommand)
    
    IDENTIFIER = types.CONVERTER_PRO


class PublisherConverterCommand(_EndpointInterfaceCommand):
    """ Class which represents a publisher converter command.
    """
    implements(IEndpointConverterCommand)
    
    IDENTIFIER = types.CONVERTER_PUB


class SubscriberConverterCommand(_EndpointInterfaceCommand):
    """ Class which represents a subscriber converter command.
    """
    implements(IEndpointConverterCommand)
    
    IDENTIFIER = types.CONVERTER_SUB


class ServiceForwarderCommand(_EndpointInterfaceCommand):
    """ Class which represents a service forwarder command.
    """
    implements(IEndpointConverterCommand)
    
    IDENTIFIER = types.FORWARDER_SRV


class ServiceProviderForwarderCommand(_EndpointInterfaceCommand):
    """ Class which represents a service-provider forwarder command.
    """
    implements(IEndpointConverterCommand)
    
    IDENTIFIER = types.FORWARDER_PRO


class PublisherForwarderCommand(_EndpointInterfaceCommand):
    """ Class which represents a publisher forwarder command.
    """
    implements(IEndpointConverterCommand)
    
    IDENTIFIER = types.FORWARDER_PUB


class SubscriberForwarderCommand(_EndpointInterfaceCommand):
    """ Class which represents a subscriber forwarder command.
    """
    implements(IEndpointConverterCommand)
    
    IDENTIFIER = types.FORWARDER_SUB


class ConnectionCommand(object):
    """ Class which represents a connection command.
    """
    implements(ISerializable)
    
    IDENTIFIER = types.CONNECTION
    
    def __init__(self, tag, commID, target, add):
        """ Initialize the connection command.
            
            @param tag:         Tag which is used to identify the interface
                                to which the message is sent and which is one
                                endpoint point of the connection which should
                                be manipulated.
            @type  tag:         str
            
            @param commID:      Communication ID of the node where the target
                                is.
            @type  commID:      str
            
            @param target:      Tag which is used to identify the interface
                                which is the other endpoint of the connection
                                which should be manipulated.
            @type  target:      str
            
            @param add:         Flag which indicates whether connection should
                                be created (True) or destroyed (False).
            @type  add:         bool
        """
        self._tag = tag
        self._commID = commID
        self._target = target
        self._add = add
    
    @property
    def tag(self):
        """ Tag of interface who is the receiver of the command. """
        return self._tag
    
    @property
    def commID(self):
        """ Communication ID of the connected interface.. """
        return self._commID
    
    @property
    def target(self):
        """ Tag of the connected interface. """
        return self._target
    
    @property
    def add(self):
        """ Flag to indicate whether connection should be created (True) or
            destroyed (False).
        """
        return self._add
    
    def serialize(self, s):
        """ Serialize the connection command.
            
            @param s:   Serializer instance into which the message should be
                        serialized.
            @type  s:   comm.serializer.Serializer
            
            @raise:     errors.SerializationError
        """
        s.addElement(self._tag)
        s.addElement(self._commID)
        s.addElement(self._target)
        s.addBool(self._add)
    
    @classmethod
    def deserialize(cls, s):
        """ Deserialize the connection command.
            
            @param s:   Serializer instance from which the message should be
                        deserialized.
            @type  s:   comm.serializer.Serializer
            
            @raise:     errors.SerializationError
        """
        return cls(s.getElement(), s.getElement(), s.getElement(), s.getBool())
